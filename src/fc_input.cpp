#include "fc_input.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "driver/rmt_rx.h"

// ---------- 共享接收缓冲（ISR 与主循环之间） ----------
static rmt_symbol_word_t s_rxAcc[RX_ACC_MAX];      // 累积的原始符号
static volatile uint16_t s_rxAccLen = 0;           // 已累积符号数
static volatile uint32_t s_rxLastUs = 0;           // 最后一次接收时间(µs)
static portMUX_TYPE s_rxLock = portMUX_INITIALIZER_UNLOCKED;

// 双缓冲交替接收（保持 buffer 在接收期间有效）
static rmt_symbol_word_t s_rxBuffs[2][RMT_MEM_BLOCK];
static volatile int s_rxBufIdx = 0;

static rmt_channel_handle_t s_rxCh = NULL;
static rmt_receive_config_t s_rxRecvCfg;

// ---------- RMT 接收完成回调（ISR 上下文） ----------
static bool IRAM_ATTR rxDoneCb(rmt_channel_handle_t ch,
                               const rmt_rx_done_event_data_t* ed,
                               void* arg) {
    (void)ch; (void)arg;
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
        s_rxLastUs = (uint32_t)esp_timer_get_time();
    }
    // 立即挂起下一轮接收（ping-pong 缓冲）
    s_rxBufIdx = 1 - s_rxBufIdx;
    rmt_receive(ch, (void*)s_rxBuffs[s_rxBufIdx],
                RMT_MEM_BLOCK * sizeof(rmt_symbol_word_t), &s_rxRecvCfg);
    return true;
}

// ---------- 初始化 ----------
void fcInputInit() {
    rmt_rx_channel_config_t cfg = {};
    cfg.gpio_num = PIN_FC_IN;
    cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    cfg.resolution_hz = RMT_RES_HZ;
    cfg.mem_block_symbols = RMT_MEM_BLOCK;
    cfg.flags.with_dma = true; // DMA 流式，符号数不受内存块限制
    rmt_new_rx_channel(&cfg, &s_rxCh);

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
// 每个 symbol 对应 1 bit：高电平时长 >= 0.6µs(48 tick) 为 1，否则 0
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
    static rmt_symbol_word_t s_local[RX_ACC_MAX]; // 主循环私有拷贝
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
