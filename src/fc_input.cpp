#include "fc_input.h"
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"

// 诊断开关：0 = 完全禁用 RMT RX（不开通道、不接收）
// 用于二分定位"中断风暴饿死 IDLE"的风暴源是否来自飞控输入 RX
#ifndef FC_INPUT_ENABLE
#define FC_INPUT_ENABLE 1
#endif

// ---------- 共享接收缓冲（ISR 与主循环之间） ----------
static rmt_symbol_word_t s_rxAcc[RX_ACC_MAX];      // 累积的原始符号
static volatile uint16_t s_rxAccLen = 0;           // 已累积符号数
static volatile uint32_t s_rxLastUs = 0;           // 最后一次接收时间(µs)
static portMUX_TYPE s_rxLock = portMUX_INITIALIZER_UNLOCKED;

// 双缓冲交替接收（保持 buffer 在接收期间有效）
static rmt_symbol_word_t s_rxBuffs[2][RX_MEM_BLOCK_SYMBOLS];
static volatile int s_rxBufIdx = 0;

// 诊断统计
volatile uint32_t g_rxFrames = 0;
volatile uint32_t g_rxLastLeds = 0;
volatile uint32_t g_rxDone = 0;   // RMT RX done 回调累计次数（是否收到任何数据）
volatile uint32_t g_rxSyms = 0;   // 累计接收符号数

// 噪声风暴防护（GPIO 悬空时 RMT RX 会被环境噪声触发中断风暴，饿死 IDLE 任务）
static volatile bool s_rxStorm = false;      // 风暴标志：ISR 已停止挂接，等主循环恢复
static volatile uint32_t s_stormAtUs = 0;    // 风暴发生时间（重新挂接前冷却的起点）
// 窗口计数判据：50ms 内 done 次数超限判风暴
// （不能用"两次 done 间隔"判：正常飞控帧每 64 符号触发一次 done，间隔仅约 80µs）
static volatile uint32_t s_doneWinUs = 0;    // 窗口开始时间
static volatile uint32_t s_doneInWin = 0;    // 窗口内 done 计数

static rmt_channel_handle_t s_rxCh = NULL;
static rmt_receive_config_t s_rxRecvCfg;
static esp_err_t s_rxErr = ESP_OK; // RX 初始化失败错误码（延迟到 USB 就绪后打印）

bool fcRxOk() { return s_rxCh != NULL; }
int  fcRxErr() { return (int)s_rxErr; }

// ---------- RMT 接收完成回调（ISR 上下文） ----------
static bool IRAM_ATTR rxDoneCb(rmt_channel_handle_t ch,
                               const rmt_rx_done_event_data_t* ed,
                               void* arg) {
    (void)ch; (void)arg;
    uint32_t t = (uint32_t)esp_timer_get_time();

    // 噪声风暴检测（窗口计数）：
    // 正常帧：50ms 内约 2~3 帧 × 12 段 ≈ 36 次 done；噪声风暴：远超 200 次
    // 判定为风暴 → 丢弃数据、不再挂接接收，让 RMT 停下来，等待主循环恢复
    if (t - s_doneWinUs > 50000) { s_doneWinUs = t; s_doneInWin = 0; }
    if (++s_doneInWin > 200) {
        s_rxStorm = true;
        s_stormAtUs = t;   // 记录风暴时刻（供主循环冷却后重新挂接）
        s_rxLastUs = 0;
        s_rxAccLen = 0;
        return false;
    }

    int n = (int)ed->num_symbols;
    g_rxDone++;                 // 诊断：done 次数
    if (n > 0) {
        g_rxSyms += (uint32_t)n; // 诊断：累计符号
        if (s_rxAccLen + n <= RX_ACC_MAX) {
            const rmt_symbol_word_t* src = ed->received_symbols;
            uint16_t base = s_rxAccLen;
            for (int i = 0; i < n; i++) s_rxAcc[base + i] = src[i];
            s_rxAccLen = (uint16_t)(s_rxAccLen + n);
        } else {
            s_rxAccLen = 0; // 溢出：丢弃从头再来
        }
        s_rxLastUs = t;
    }
    // 立即挂起下一轮接收（ping-pong 缓冲，续接拼接长帧）
    s_rxBufIdx = 1 - s_rxBufIdx;
    rmt_receive(ch, (void*)s_rxBuffs[s_rxBufIdx],
                RX_MEM_BLOCK_SYMBOLS * sizeof(rmt_symbol_word_t), &s_rxRecvCfg);
    return true;
}

// ---------- 初始化 ----------
void fcInputInit() {
#if FC_INPUT_ENABLE == 0
    // 诊断模式：RX 完全禁用
    printf("[RX] DISABLED (FC_INPUT_ENABLE=0)\n");
    return;
#endif
    rmt_rx_channel_config_t cfg = {};
    cfg.gpio_num = PIN_FC_IN;
    cfg.clk_src = RMT_CLK_SRC;          // 40MHz XTAL（C3 不支持 80MHz 分辨率）
    cfg.resolution_hz = RMT_RES_HZ;
    cfg.mem_block_symbols = RX_MEM_BLOCK_SYMBOLS; // 256：减少续接次数，防长帧丢段
    // 关键：ESP32-C3 的 RMT 不支持 DMA（with_dma=true 返回 ESP_ERR_NOT_SUPPORTED 0x106）。
    // 去掉 DMA：ISR 每收到 64 符号触发一次 done，回调里续接接收并拼接成完整帧。
    // cfg.flags.with_dma = false;（默认即 false，不设置）
    esp_err_t err = rmt_new_rx_channel(&cfg, &s_rxCh);
    if (err != ESP_OK) {
        s_rxErr = err; // 记录错误码，USB 就绪后由 main 打印
        return;
    }

    // 重要：RMT 通道创建会重配 GPIO（可能清除此前设置的上拉）。
    // 未接飞控时 GPIO4 悬空、无上拉会被环境噪声触发 RX 中断风暴，
    // 饿死 IDLE 任务导致任务看门狗超时。
    // 因此上拉必须在【全部 RMT 初始化完成之后】最后再设置一次：
    // 通道创建 / enable / receive 都可能重置 GPIO 配置。
    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rxDoneCb;
    rmt_rx_register_event_callbacks(s_rxCh, &cbs, NULL);

    rmt_enable(s_rxCh);

    s_rxRecvCfg = {};
    s_rxRecvCfg.signal_range_min_ns = RX_SIGNAL_MIN_NS;
    s_rxRecvCfg.signal_range_max_ns = RX_SIGNAL_MAX_NS;
    rmt_receive(s_rxCh, (void*)s_rxBuffs[0],
                RX_MEM_BLOCK_SYMBOLS * sizeof(rmt_symbol_word_t), &s_rxRecvCfg);

    // 最后再确保一次上拉（GPIO 配置以此次为准）
    gpio_set_pull_mode(PIN_FC_IN, GPIO_PULLUP_ONLY);
}

// ---------- 解码：符号流 → GRB 灯色 ----------
// 每个 symbol 对应 1 bit：取其中【高电平段】的时长（≥0.6µs/24 tick 为 1），
// 兼容 RMT RX 捕获的 (高,低) 或 (低,高) 两种电平对方向；24 bit = 1 颗灯，顺序 G-R-B
static int decodeFrame(const rmt_symbol_word_t* sym, int n, RGB* out, int maxLeds) {
    uint8_t byte = 0;
    int bits = 0, byteIdx = 0, ledIdx = 0;
    uint8_t grb[3] = {0, 0, 0};
    int start = 0;

    // 跳过帧前空闲段（两段都不含高电平：reset 低电平等）
    while (start < n && sym[start].level0 == 0 && sym[start].level1 == 0) start++;

    for (int i = start; i < n && ledIdx < maxLeds; i++) {
        // 高电平段的时长：level0==1 则取 duration0，否则取 duration1
        uint32_t hiDur = (sym[i].level0 == 1) ? sym[i].duration0 : sym[i].duration1;
        bool one = (hiDur >= TICK_HI_THRESHOLD);
        byte = (uint8_t)((byte << 1) | (one ? 1 : 0));
        bits++;
        if (bits == 8) {
            grb[byteIdx++] = byte;
            byte = 0; bits = 0;
            if (byteIdx == 3) {
                out[ledIdx].g = grb[0];
                out[ledIdx].r = grb[1];
                out[ledIdx].b = grb[2];
                ledIdx++;
                byteIdx = 0;
            }
        }
    }
    return ledIdx;
}

// ---------- 轮询：取完整帧 ----------
// 帧结束判定：距最后一次接收超过 4ms（单帧约 1ms，帧间隔约 10~20ms）
bool pollFcInput(FcFrame* out) {
#if FC_INPUT_ENABLE == 0
    (void)out;
    return false;
#endif
    static rmt_symbol_word_t s_local[RX_ACC_MAX]; // 主循环私有拷贝

    // 若 ISR 因噪声风暴停止了接收，冷却 500ms 后再重新挂接：
    // 避免"挂接→立即风暴→又挂接"的循环持续占用 CPU、饿死 IDLE。
    // 即使 GPIO4 持续有噪声，也只会每 500ms 产生一波短风暴（≤50ms），系统不卡死。
    if (s_rxStorm) {
        uint32_t nowUs2 = (uint32_t)esp_timer_get_time();
        if (nowUs2 - s_stormAtUs > 500000) {
            s_rxStorm = false;
            s_doneWinUs = 0;
            s_doneInWin = 0;
            rmt_receive(s_rxCh, (void*)s_rxBuffs[s_rxBufIdx],
                        RX_MEM_BLOCK_SYMBOLS * sizeof(rmt_symbol_word_t), &s_rxRecvCfg);
        }
    }

    uint32_t nowUs = (uint32_t)esp_timer_get_time();

    uint16_t len = 0;
    bool frameDone = false;
    portENTER_CRITICAL(&s_rxLock);
    len = s_rxAccLen;
    if (len > 0 && (nowUs - s_rxLastUs) > RX_FRAME_GAP_US) {
        memcpy(s_local, (const void*)s_rxAcc, len * sizeof(rmt_symbol_word_t));
        s_rxAccLen = 0;
        frameDone = true;
    }
    portEXIT_CRITICAL(&s_rxLock);

    if (!frameDone) return false;
    out->count = decodeFrame(s_local, (int)len, out->leds, FC_MAX_LEDS);
    if (out->count > 0) {
        g_rxFrames++;
        g_rxLastLeds = (uint32_t)out->count;
    }
    return out->count > 0;
}
