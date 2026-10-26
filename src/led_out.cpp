#include "led_out.h"
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"

// ============================================================
// v32.32：WS2812 输出从 RMT 换成 SPI2 + DMA
//
// 【为什么必须换 —— 这是从 v32.24 追到 v32.31 所有怪象的同一个根】
//   ESP32-C3 的 RMT **没有 DMA**：一帧 45 颗灯要 1.4ms，芯片内部只缓 48 个符号
//   （约 60µs），必须靠中断每 60µs 续填一次 FIFO。于是：
//     · WiFi beacon / BLE 广播 / 网页 HTTP，任何一次关中断超过 60µs，
//       FIFO 就空了 → 灯珠收到残帧 → 整条灯带闪一下（用户看到的「频闪」）；
//       更糟时灯珠按残帧锁存，前段新色后段旧色（「亮部杂色」）。
//     · 飞控采集为了测准位宽要屏蔽全部中断 1.3ms → FIFO 必然断流，
//       狠了事务永远等不到完成中断 → rmt_transmit 一直返回 INVALID_STATE
//       → 灯带定格/全灭，只有重启能恢复（「死机」）。
//   这两件事无论怎么改判据、怎么串行化都消不掉，因为它是硬件通路决定的。
//
//   SPI + DMA 的差别是本质的：DMA 引擎独立搬运，**不依赖任何中断**。
//   CPU 被屏蔽中断 1.3ms，DMA 照样把 1.35ms 的数据一线不差地推出去。
//
// 【编码方式】WS2812 over SPI（业界通用做法）
//   1 个灯珠 bit = 3 个 SPI bit；SPI 时钟 2.4MHz（1 SPI bit ≈ 417ns）
//     0 码 → 100 → 高 417ns / 低 833ns（标准 400 / 850）
//     1 码 → 110 → 高 833ns / 低 417ns（标准 800 / 450）
//   每颗灯 bit 总长恒为 1250ns，与 WS2812 位周期完全一致。
//   帧尾补若干 0x00 字节当复位（80µs 以上；1 字节 ≈ 3.3µs → 40 字节 ≈ 132µs）。
// ============================================================

#define SPI_LED_BYTES    (STRIP_MAX_LEDS * 9)  // 每颗灯 3 字节 × 8 bit × 3 SPI bit
#define SPI_RESET_BYTES  40                    // 复位：40 × 3.3µs ≈ 132µs（要 > 80µs）
#define SPI_ALL_BYTES    (SPI_LED_BYTES + SPI_RESET_BYTES + 8)

static spi_device_handle_t s_spi = NULL;
// v32.32：主循环与 HTTP 任务都会调 showStrip（网页改参数要立即上屏），
// SPI 一次事务不能被两个任务同时用 —— 用互斥量排队。
// 注意不能用 portENTER_CRITICAL：一次传输 1.35ms，关中断那么久会把一切都搞乱。
static SemaphoreHandle_t s_txLock = NULL;
// DMA 源缓冲：必须在内部 RAM（静态数组满足），传输期间不能被改写
static uint8_t s_spiBuf[SPI_ALL_BYTES];
static esp_err_t s_ledErr = ESP_OK; // 初始化失败错误码（延迟到 USB 就绪后打印）

volatile uint32_t g_txCount = 0;   // 成功发送帧计数（心跳诊断）
volatile uint32_t g_txFail = 0;    // 发送失败次数
volatile int      g_txLastErr = 0; // 最近一次失败的错误码

bool ledOutOk() { return s_spi != NULL; }
int  ledOutErr() { return (int)s_ledErr; }

// ---------- 编码：RGB 数组 → SPI 字节流 ----------
// 灯带自身要的是 GRB 顺序；飞控色序（BGR 等）在 fc_input 里已经换好，
// 这里只管把 RGB 结构体里的东西按 GRB 发出去，与旧 RMT 版行为完全一致。
static int encodeStrip(const RGB* leds, int n, uint8_t* buf) {
    uint32_t acc = 0;   // 攒 bit 的累加器
    int nb = 0;         // acc 里还没输出的 bit 数（< 8）
    int o = 0;

    for (int i = 0; i < n; i++) {
        uint8_t bytes[3] = { leds[i].g, leds[i].r, leds[i].b }; // GRB
        for (int b = 0; b < 3; b++) {
            uint8_t v = bytes[b];
            for (int k = 7; k >= 0; k--) {
                // 0 码 → 100，1 码 → 110
                acc = (acc << 3) | ((v & (1u << k)) ? 0x6u : 0x4u);
                nb += 3;
                if (nb >= 8) {
                    nb -= 8;
                    buf[o++] = (uint8_t)(acc >> nb);
                }
            }
        }
    }
    if (nb > 0) buf[o++] = (uint8_t)(acc << (8 - nb)); // 末尾补齐一字节（低位补 0 = 低电平）
    memset(buf + o, 0, SPI_RESET_BYTES);               // 帧复位（低电平拉长）
    return o + SPI_RESET_BYTES;
}

// ---------- 初始化 ----------
void ledOutInit() {
    spi_bus_config_t bus = {};
    bus.mosi_io_num = PIN_LED_OUT;   // 只用到 MOSI，时钟不用（WS2812 是单线）
    bus.miso_io_num = -1;
    bus.sclk_io_num = -1;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = sizeof(s_spiBuf);

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        s_ledErr = err;   // 记录错误码，USB 就绪后由 main 打印
        return;
    }

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = 2400000;    // 2.4MHz：1 SPI bit ≈ 417ns，3 bit = 1250ns
    dev.mode = 0;                    // 空闲低电平（WS2812 复位态就是低）
    dev.spics_io_num = -1;           // 无片选
    dev.queue_size = 2;
    err = spi_bus_add_device(SPI2_HOST, &dev, &s_spi);
    if (err != ESP_OK) {
        s_ledErr = err;
        s_spi = NULL;
        return;
    }
    s_txLock = xSemaphoreCreateMutex();
}

// ---------- 等待发送彻底结束 ----------
// DMA 传输是同步完成的（polling_transmit 返回时数据已全部推到线上），
// 所以这里什么都不用做。保留接口是为了不改主循环的时序结构。
void ledOutWaitIdle() {
}

// ---------- 发送一帧 ----------
// v32.32：SPI 没有「队列卡死」这回事 —— 每次事务都是独立的 DMA 描述符，
// 失败了下次重来就行，不存在旧版 RMT 里「事务永远等不到完成中断」的死锁。
void showStrip(const RGB* leds, int n) {
    if (s_spi == NULL) return;          // 未就绪：跳过，避免报错刷屏
    if (n <= 0) n = 1;
    if (n > STRIP_MAX_LEDS) n = STRIP_MAX_LEDS;

    if (s_txLock) xSemaphoreTake(s_txLock, portMAX_DELAY);
    int len = encodeStrip(leds, n, s_spiBuf);

    spi_transaction_t t = {};
    t.length = (size_t)len * 8;         // 单位是 bit
    t.tx_buffer = s_spiBuf;
    esp_err_t e = spi_device_polling_transmit(s_spi, &t);
    if (e != ESP_OK) {
        g_txFail++;
        g_txLastErr = (int)e;
        printf("[LED] spi tx err %s (0x%x) bytes=%d\n", esp_err_to_name(e), e, len);
    } else {
        g_txCount++;
    }
    if (s_txLock) xSemaphoreGive(s_txLock);
}
