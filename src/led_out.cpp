#include "led_out.h"
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_attr.h"     // v32.38：WORD_ALIGNED_ATTR（DMA 源地址要按字对齐）
#include "esp_rom_sys.h"  // v32.39：esp_rom_delay_us（两遍发送之间的复位间隔）
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
// v32.39【复位从 40 字节(133µs) 提到 100 字节(333µs)】
//   旧值按「WS2812B 老手册 >50µs / 常见阈值 80µs」取的，133µs 已经够。但现场这批
//   灯珠（以及多数新批次，包括 WS2812B-V5 / 各类克隆）手册要求的是 **>280µs**。
//   复位不够的后果不是「全乱」，而是「一部分灯珠锁存了、另一部分还挂上一帧的数据」
//   —— 表现正好是「随机几颗灯颜色/亮度跟别的灯不一样」。而只要画面是静止的，
//   上一帧和本帧内容相同，这个错就完全看不出来（「冻结自检不闪」正是这个道理：
//   它证明不了输出干净，只能证明没有时序抖动）。
//   333µs 对所有已知 WS2812 变种都安全，多花的时间只有 0.13ms/帧。
#define SPI_RESET_BYTES  100                   // 复位：100 × 3.3µs ≈ 333µs（> 280µs）
#define SPI_ALL_BYTES    (SPI_LED_BYTES + SPI_RESET_BYTES + 8)
// v32.39【同一帧连发两遍】
//   第二遍紧跟在 400µs 之后重发同样的字节。作用是给「锁存没锁干净 / 第一遍被线上
//   干扰啃掉几个 bit」的灯珠一次补偿机会 —— 对正常灯带毫无副作用（同样颜色再锁
//   一次还是同样颜色），成本 1.7ms/帧。若现场「随机几颗偏暗」就此消失，那问题
//   就已经定位在灯珠锁存/信号完整性这一侧，与解码无关。
#define LED_TX_REPEAT    2
#define LED_TX_GAP_US    400                   // 两遍之间 >280µs 复位

static spi_device_handle_t s_spi = NULL;
// v32.32：主循环与 HTTP 任务都会调 showStrip（网页改参数要立即上屏），
// SPI 一次事务不能被两个任务同时用 —— 用互斥量排队。
// 注意不能用 portENTER_CRITICAL：一次传输 1.35ms，关中断那么久会把一切都搞乱。
static SemaphoreHandle_t s_txLock = NULL;
// DMA 源缓冲：必须在内部 RAM（静态数组满足），传输期间不能被改写。
// v32.38【必须 4 字节对齐】SPI 的 DMA 是按 32 位字搬运的，源地址不按字对齐时，
// 头部那几个字节可能被多读/错位，线上波形就整体偏一段 —— 表现是首颗灯乱色，
// 严重时整条错位。加 WORD_ALIGNED_ATTR 让编译器保证起始地址对齐。
// 另一半在 showStrip 里：传输长度也要凑成 4 的整数倍（补 0 = 低电平，正好是复位段）。
static WORD_ALIGNED_ATTR uint8_t s_spiBuf[SPI_ALL_BYTES + 4];
static esp_err_t s_ledErr = ESP_OK; // 初始化失败错误码（延迟到 USB 就绪后打印）

volatile uint32_t g_txCount = 0;   // 成功发送帧计数（心跳诊断）
volatile uint32_t g_txFail = 0;    // 发送失败次数
volatile int      g_txLastErr = 0; // 最近一次失败的错误码
// v32.39【输出侧自证：编码结果校验和】
//   「解出来的帧一模一样」这件事，前面已经有 g_rxBitDiff / g_outDiff 两层证明；
//   但那是「颜色数组」层面的。真正上线的是编码后的 SPI 字节流，中间还有编码、
//   补字节、DMA 对齐三处可能出错。这里对**实际交给 DMA 的那段字节**算一次
//   FNV-1a，并统计「与上一帧校验和不同」的次数。
//   判读：画面静止时 txchg 必须恒为 0。若 txchg 恒 0 而灯还在变 —— 我们可以
//   保证线上发的是同一串字节，那问题只可能在灯珠侧（供电/电平/锁存），
//   固件这边再改也是白改。
volatile uint32_t g_txSum = 0;
volatile uint32_t g_txSumChg = 0;
static uint32_t   s_prevSum = 0;
static bool       s_prevSumOk = false;

static inline uint32_t fnv1a(const uint8_t* p, int n) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}
int ledTxRepeat() { return LED_TX_REPEAT; }
int ledTxResetUs() { return SPI_RESET_BYTES * 1000 / 300; }  // 近似：3.33µs/字节

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
    // v32.38：把长度凑成 4 的整数倍。SPI 走 DMA 时按 32 位字搬运，
    // 尾部的非整字部分曾出现过被多带/丢字节的情况；补的这几个字节是 0x00，
    // 对 WS2812 来说就是继续拉低（复位段），对时序只有好处没有坏处。
    while ((len & 3) != 0) s_spiBuf[len++] = 0x00;

    spi_transaction_t t = {};
    t.length = (size_t)len * 8;         // 单位是 bit
    t.tx_buffer = s_spiBuf;
    // v32.39：校验和算的是「真正交给 DMA 的那段字节」，编码/补齐之后的最终结果
    uint32_t sum = fnv1a(s_spiBuf, len);
    g_txSum = sum;
    if (s_prevSumOk && sum != s_prevSum) g_txSumChg++;
    s_prevSum = sum;
    s_prevSumOk = true;
    esp_err_t e = ESP_OK;
    for (int r = 0; r < LED_TX_REPEAT; r++) {
        if (r) esp_rom_delay_us(LED_TX_GAP_US);   // 两遍之间留足复位，别硬贴在一起
        e = spi_device_polling_transmit(s_spi, &t);
        if (e != ESP_OK) break;
    }
    if (e != ESP_OK) {
        g_txFail++;
        g_txLastErr = (int)e;
        printf("[LED] spi tx err %s (0x%x) bytes=%d\n", esp_err_to_name(e), e, len);
    } else {
        g_txCount++;
    }
    if (s_txLock) xSemaphoreGive(s_txLock);
}
