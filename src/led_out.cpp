#include "led_out.h"
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

static rmt_channel_handle_t s_txCh = NULL;
static rmt_encoder_handle_t s_copyEnc = NULL;
static rmt_symbol_word_t s_txSymbols[STRIP_MAX_LEDS * 24 + 1];

// ---------- 编码：RGB 数组 → WS2812 符号流（GRB，MSB first） ----------
static int encodeStripToSymbols(const RGB* leds, int n, rmt_symbol_word_t* sym) {
    int idx = 0;
    for (int i = 0; i < n; i++) {
        uint8_t bytes[3] = { leds[i].g, leds[i].r, leds[i].b }; // GRB
        for (int b = 0; b < 3; b++) {
            uint8_t v = bytes[b];
            for (int bit = 7; bit >= 0; bit--) {
                if (v & (1u << bit)) {
                    sym[idx].duration0 = T1H_TICKS; sym[idx].level0 = 1;
                    sym[idx].duration1 = T1L_TICKS; sym[idx].level1 = 0;
                } else {
                    sym[idx].duration0 = T0H_TICKS; sym[idx].level0 = 1;
                    sym[idx].duration1 = T0L_TICKS; sym[idx].level1 = 0;
                }
                idx++;
            }
        }
    }
    // 帧复位脉冲（低电平 80µs）
    sym[idx].duration0 = 0;   sym[idx].level0 = 0;
    sym[idx].duration1 = RESET_TICKS; sym[idx].level1 = 0;
    idx++;
    return idx;
}

// ---------- 初始化 ----------
void ledOutInit() {
    rmt_tx_channel_config_t cfg = {};
    cfg.gpio_num = PIN_LED_OUT;
    cfg.clk_src = RMT_CLK_SRC;          // 40MHz XTAL（C3 不支持 80MHz 分辨率）
    cfg.resolution_hz = RMT_RES_HZ;
    cfg.mem_block_symbols = RMT_MEM_BLOCK;
    cfg.trans_queue_depth = 4;
    cfg.flags.with_dma = true; // DMA 流式，符号数不受内存块限制
    esp_err_t err = rmt_new_tx_channel(&cfg, &s_txCh);
    if (err != ESP_OK) {
        // 初始化失败诊断：打印具体错误码（ESP_ERR_INVALID_ARG=0x102 等）
        printf("[LED] rmt_new_tx_channel FAILED: %s (0x%x)\n", esp_err_to_name(err), err);
        return;
    }

    rmt_copy_encoder_config_t encCfg = {};
    rmt_new_copy_encoder(&encCfg, &s_copyEnc);

    rmt_enable(s_txCh);
}

// ---------- 发送一帧 ----------
void showStrip(const RGB* leds, int n) {
    if (n <= 0) n = 1;
    if (n > STRIP_MAX_LEDS) n = STRIP_MAX_LEDS;
    int len = encodeStripToSymbols(leds, n, s_txSymbols);

    rmt_tx_wait_all_done(s_txCh, 20); // 等上一帧发完，避免队列积压
    rmt_transmit_config_t tcfg = {};
    tcfg.loop_count = 0;
    // 注意：payload_bytes 是字节数，不是符号数（copy 编码器按字节数解析符号数组）
    rmt_transmit(s_txCh, s_copyEnc, s_txSymbols,
                 (size_t)len * sizeof(rmt_symbol_word_t), &tcfg);
}
