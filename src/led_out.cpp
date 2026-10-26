#include "led_out.h"
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

static rmt_channel_handle_t s_txCh = NULL;
static rmt_encoder_handle_t s_copyEnc = NULL;
static rmt_symbol_word_t s_txSymbols[STRIP_MAX_LEDS * 24 + 1];

volatile uint32_t g_txCount = 0; // 成功发送帧计数（心跳诊断）
volatile uint32_t g_txFail = 0;  // v32.29：发送失败次数（通道卡死的直接证据）
volatile int      g_txLastErr = 0; // v32.29：最近一次失败的错误码
static esp_err_t s_ledErr = ESP_OK; // 初始化失败错误码（延迟到 USB 就绪后打印）

bool ledOutOk() { return s_txCh != NULL && s_copyEnc != NULL; }
int  ledOutErr() { return (int)s_ledErr; }

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
    // 帧复位脉冲（低电平 80µs）。
    // 注意：RMT copy 编码器不接受 duration=0 的符号（会返回 invalid argument，
    // 且旧代码没检查返回值导致灯带从未收到有效信号）——duration0 必须 >= 1。
    sym[idx].duration0 = 1;          sym[idx].level0 = 0;
    sym[idx].duration1 = RESET_TICKS - 1; sym[idx].level1 = 0;
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
    // 关键：ESP32-C3 的 RMT 不支持 DMA（with_dma=true 会返回 ESP_ERR_NOT_SUPPORTED 0x106，
    // 通道创建失败 → 灯带永远收不到信号）。去掉 DMA 后 IDF 用中断分段编码发送，
    // 任意长度（>64 符号的整帧）都能发。
    // cfg.flags.with_dma = false;（默认即 false，不设置）
    esp_err_t err = rmt_new_tx_channel(&cfg, &s_txCh);
    if (err != ESP_OK) {
        s_ledErr = err; // 记录错误码，USB 就绪后由 main 打印（开机早期 printf 会丢）
        return;
    }

    rmt_copy_encoder_config_t encCfg = {};
    esp_err_t err2 = rmt_new_copy_encoder(&encCfg, &s_copyEnc);
    if (err2 != ESP_OK) {
        s_ledErr = err2; // 记录错误码
        return;
    }

    rmt_enable(s_txCh);
}

// ---------- 等待发送彻底结束 ----------
// v32.29：fc_input 的采集窗要在这个之后才开。rmt_transmit 只是把事务排进队列就
// 返回了，硬件还要 1.4ms 才发完；这期间开采集（屏蔽全部中断 1.3ms）会把续填
// FIFO 的中断一起屏蔽掉 → 灯带收到残帧。
void ledOutWaitIdle() {
    if (s_txCh != NULL) rmt_tx_wait_all_done(s_txCh, pdMS_TO_TICKS(30));
}

// ---------- 发送一帧 ----------
// v32.29：发送失败必须自愈，不能靠重启。
// 现场症状「跑一段时间灯全灭、重启才恢复」完全对得上 RMT 通道卡死：
// 本板 RMT 无 DMA，长帧靠中断每 80µs 续填一次 FIFO；只要有一次续填被推迟
// （飞控采集那 1.3ms 的中断屏蔽就够），输出就断流，队列里残留的事务永远
// 等不到完成中断 → rmt_transmit 一直返回 ESP_ERR_INVALID_STATE → 灯带再也不动。
// 现在：失败就把通道关掉再打开（清空队列与状态机），下一帧继续发。
static void txRecover() {
    rmt_disable(s_txCh);
    rmt_enable(s_txCh);
}

void showStrip(const RGB* leds, int n) {
    if (s_txCh == NULL || s_copyEnc == NULL) return; // 通道/编码器未就绪：跳过，避免报错刷屏
    if (n <= 0) n = 1;
    if (n > STRIP_MAX_LEDS) n = STRIP_MAX_LEDS;
    int len = encodeStripToSymbols(leds, n, s_txSymbols);

    rmt_tx_wait_all_done(s_txCh, pdMS_TO_TICKS(20)); // 等上一帧发完，避免队列积压
    rmt_transmit_config_t tcfg = {};
    tcfg.loop_count = 0;
    // 注意：payload_bytes 是字节数，不是符号数（copy 编码器按字节数解析符号数组）
    esp_err_t e = rmt_transmit(s_txCh, s_copyEnc, s_txSymbols,
                               (size_t)len * sizeof(rmt_symbol_word_t), &tcfg);
    if (e != ESP_OK) {
        g_txFail++;
        g_txLastErr = (int)e;
        txRecover();
        // 发送失败诊断：错误码 + 本帧符号数
        printf("[LED] rmt_transmit err %s (0x%x) syms=%d\n",
               esp_err_to_name(e), e, len);
    } else {
        g_txCount++;
    }
}
