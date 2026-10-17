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
static rmt_symbol_word_t s_rxBuffs[2][RMT_MEM_BLOCK];
static volatile int s_rxBufIdx = 0;

// 噪声风暴防护（GPIO 悬空时 RMT RX 会被环境噪声触发中断风暴，饿死 IDLE 任务）
static volatile uint32_t s_lastDoneUs = 0;   // 上次 RX done 时间(µs)
static volatile bool s_rxStorm = false;      // 风暴标志：ISR 已停止挂接，等主循环恢复
// 窗口计数判据：50ms 内 done 次数超限判风暴
// （不能用"两次 done 间隔"判：正常飞控帧每 64 符号触发一次 done，间隔仅约 80µs）
static volatile uint32_t s_doneWinUs = 0;    // 窗口开始时间
static volatile uint32_t s_doneInWin = 0;    // 窗口内 done 计数

static rmt_channel_handle_t s_rxCh = NULL;
static rmt_receive_config_t s_rxRecvCfg;

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
        s_rxLastUs = 0;
        s_rxAccLen = 0;
        return false;
    }
    s_lastDoneUs = t;

    int n = (int)ed->num_symbols;
    if (n > 0) {
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
    // 立即挂起下一轮接收（ping-pong 缓冲）
    s_rxBufIdx = 1 - s_rxBufIdx;
    rmt_receive(ch, (void*)s_rxBuffs[s_rxBufIdx],
                RMT_MEM_BLOCK * sizeof(rmt_symbol_word_t), &s_rxRecvCfg);
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
    cfg.mem_block_symbols = RMT_MEM_BLOCK;
    cfg.flags.with_dma = true; // DMA 流式，符号数不受内存块限制
    esp_err_t err = rmt_new_rx_channel(&cfg, &s_rxCh);
    if (err != ESP_OK) {
        // 初始化失败诊断：打印具体错误码
        printf("[RX] rmt_new_rx_channel FAILED: %s (0x%x)\n", esp_err_to_name(err), err);
        return;
    }

    // 重要：RMT 通道创建会重配 GPIO（可能清除此前设置的上拉）。
    // 未接飞控时 GPIO4 悬空、无上拉会被环境噪声触发 RX 中断风暴，
    // 饿死 IDLE 任务导致任务看门狗超时——必须在通道创建后显式重新上拉。
    gpio_set_pull_mode(PIN_FC_IN, GPIO_PULLUP_ONLY);

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = rxDoneCb;
    rmt_rx_register_event_callbacks(s_rxCh, &cbs, NULL);

    rmt_enable(s_rxCh);

    s_rxRecvCfg = {};
    s_rxRecvCfg.signal_range_min_ns = RX_SIGNAL_MIN_NS;
    s_rxRecvCfg.signal_range_max_ns = RX_SIGNAL_MAX_NS;
    rmt_receive(s_rxCh, (void*)s_rxBuffs[0],
                RMT_MEM_BLOCK * sizeof(rmt_symbol_word_t), &s_rxRecvCfg);
}

// ---------- 解码：符号流 → GRB 灯色 ----------
// 每个 symbol 对应 1 bit：高电平时长 >= 0.6µs(24 tick @40MHz) 为 1，否则 0
// 24 bit = 1 颗灯，顺序 G-R-B（WS2812/Betaflight 原生顺序）
static int decodeFrame(const rmt_symbol_word_t* sym, int n, RGB* out, int maxLeds) {
    uint8_t byte = 0;
    int bits = 0, byteIdx = 0, ledIdx = 0;
    uint8_t grb[3] = {0, 0, 0};
    int start = 0;

    // 跳过可能捕获到的空闲低电平开头（首个完整高电平 symbol 才开始）
    while (start < n && sym[start].level0 == 0) start++;

    for (int i = start; i < n && ledIdx < maxLeds; i++) {
        bool one = (sym[i].duration0 >= TICK_HI_THRESHOLD);
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

    // 若 ISR 因噪声风暴停止了接收，这里重新挂接（风暴被抑制，不会再风暴）
    if (s_rxStorm) {
        s_rxStorm = false;
        s_doneWinUs = 0;
        s_doneInWin = 0;
        rmt_receive(s_rxCh, (void*)s_rxBuffs[s_rxBufIdx],
                    RMT_MEM_BLOCK * sizeof(rmt_symbol_word_t), &s_rxRecvCfg);
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
    return out->count > 0;
}
