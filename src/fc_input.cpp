#include "fc_input.h"
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"

// 诊断开关：0 = 完全禁用 RX（不开中断、不接收）
// 用于二分定位"中断风暴饿死 IDLE"的风暴源是否来自飞控输入 RX
#ifndef FC_INPUT_ENABLE
#define FC_INPUT_ENABLE 1
#endif

// ============================================================
// 软件解码（替代 RMT RX —— ESP32-C3 的 RMT 每通道内存上限 64 符号，
// 无 DMA、无 partial 流式，28 颗长帧(672 符号)续接必然丢段，只解出前 2 颗）
//
// 原理：GPIO4 上升沿中断 → ISR 内忙等轮询电平，连续测量每个 bit 的
// 高电平时长（WS2812 800kHz：高 800ns=1 / 高 400ns=0，阈值 600ns）。
// 一帧在一次中断里连续收完，无续接丢失。
// 帧尾：低电平持续 > 50µs（reset）→ 忙等变高等超时 → 帧完成。
// 忙等开销：672bit × 1.25µs ≈ 0.84ms/帧，30 帧/s ≈ 2.5% CPU。
// ============================================================

#define FC_GPIO_BIT    (1u << 4)      // GPIO4
// 160MHz 主频：1 cycle = 6.25ns
#define FC_HI_1_CYC    128            // bit1 高 800ns
#define FC_HI_0_CYC    64             // bit0 高 400ns
#define FC_BIT_THRESHOLD_CYC 96       // 判别阈值 600ns
// 帧尾判定超时 20µs（3200 cycle @160MHz）：
// WS2812 bit 间电平最长 850ns，而帧尾 reset 低电平 ≥ 50µs、帧间空闲为 10~20ms，
// 20µs 可明确区分。实测 Betaflight reset 约 50µs，56µs 阈值会漏判导致帧永不结束。
#define FC_RESET_TIMEOUT_CYC 3200
#define FC_BIT_MIN_CYC  32            // 首个 bit 高时长下限（200ns）：低于此视为帧头不可靠，弃帧

// ---------- ISR 与主循环共享 ----------
static portMUX_TYPE s_rxLock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_swBits = 0;     // 累积 bit（GRB 高位在前）
static volatile uint8_t  s_swBitCnt = 0;
static volatile RGB      s_swLeds[FC_MAX_LEDS];
static volatile int      s_swLedCnt = 0;
static volatile bool     s_swFrameDone = false;

// 诊断统计
volatile uint32_t g_rxFrames = 0;
volatile uint32_t g_rxLastLeds = 0;
volatile uint32_t g_rxDone = 0;      // 软件解码：帧头中断次数
volatile uint32_t g_rxSyms = 0;      // 软件解码：累计 bit 数
volatile uint32_t g_rxDrop = 0;      // 软件解码：帧头不可靠丢弃帧数
volatile uint32_t g_rxFrameEnd = 0;  // 软件解码：ISR 置"帧完成"次数
volatile uint32_t g_rxLedAtEnd = 0;  // 软件解码：最近一次帧完成时的灯数
static esp_err_t s_rxErr = ESP_OK;
static bool s_rxSwInit = false;

bool fcRxOk() { return s_rxSwInit; }
int  fcRxErr() { return (int)s_rxErr; }

// ---------- bit 累积 ----------
static inline void IRAM_ATTR swPushBit(bool one) {
    s_swBits = (s_swBits << 1) | (one ? 1u : 0u);
    s_swBitCnt = (uint8_t)(s_swBitCnt + 1);
    if (s_swBitCnt == 24) {
        uint32_t v = s_swBits;
        if (s_swLedCnt < FC_MAX_LEDS) {
            s_swLeds[s_swLedCnt].g = (uint8_t)(v >> 16);
            s_swLeds[s_swLedCnt].r = (uint8_t)(v >> 8);
            s_swLeds[s_swLedCnt].b = (uint8_t)v;
            s_swLedCnt = (int)(s_swLedCnt + 1);
        }
        s_swBits = 0;
        s_swBitCnt = 0;
    }
    g_rxSyms = (uint32_t)(g_rxSyms + 1);
}

// ---------- GPIO 上升沿中断：一次中断收完整帧 ----------
static void IRAM_ATTR fcRiseIsr(void* arg) {
    (void)arg;

    // 测量第一个 bit 的剩余高电平（从 ISR 代码开始到变低）。
    // 带超时：20µs 内电平不变低 = 帧间空闲高 / 忙等期间锁存的边沿误触发
    // → 直接返回【不清任何状态】，保护上一帧刚置的 frameDone。
    // 进入时电平已低（锁存/帧间低）→ firstHi≈0 → 弃帧（同样不重置状态）。
    uint32_t a0 = esp_cpu_get_cycle_count();
    while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
        if ((esp_cpu_get_cycle_count() - a0) > FC_RESET_TIMEOUT_CYC) {
            return;
        }
    }
    uint32_t firstHi = esp_cpu_get_cycle_count() - a0;
    if (firstHi < FC_BIT_MIN_CYC) {
        g_rxDrop = (uint32_t)(g_rxDrop + 1); // 帧头不可靠：弃帧，保留上一帧完成状态
        return;
    }

    // 确认是真新帧：清累积
    s_swBits = 0;
    s_swBitCnt = 0;
    s_swLedCnt = 0;
    s_swFrameDone = false;
    g_rxDone = (uint32_t)(g_rxDone + 1);

    swPushBit(firstHi > FC_BIT_THRESHOLD_CYC);

    // 连续测量后续 bit：忙等"变高→变低"交替；
    // 低电平超时（帧尾 reset）或高电平超时（帧间空闲）都判定帧结束
    while (1) {
        uint32_t w0 = esp_cpu_get_cycle_count();
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) == 0) {
            if ((esp_cpu_get_cycle_count() - w0) > FC_RESET_TIMEOUT_CYC) {
                s_swFrameDone = true;
                g_rxFrameEnd = (uint32_t)(g_rxFrameEnd + 1);
                g_rxLedAtEnd = (uint32_t)s_swLedCnt;
                return;   // 帧尾 reset（低电平超时）
            }
        }
        uint32_t h0 = esp_cpu_get_cycle_count();
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
            if ((esp_cpu_get_cycle_count() - h0) > FC_RESET_TIMEOUT_CYC) {
                s_swFrameDone = true;
                g_rxFrameEnd = (uint32_t)(g_rxFrameEnd + 1);
                g_rxLedAtEnd = (uint32_t)s_swLedCnt;
                return;   // 帧间空闲高电平超时（防卡死在等待变低）
            }
        }
        uint32_t h1 = esp_cpu_get_cycle_count();
        swPushBit((h1 - h0) > FC_BIT_THRESHOLD_CYC);
    }
}

// ---------- 初始化 ----------
void fcInputInit() {
#if FC_INPUT_ENABLE == 0
    printf("[RX] DISABLED (FC_INPUT_ENABLE=0)\n");
    return;
#endif
    s_rxErr = ESP_OK;
    // 上拉：未接飞控时 GPIO4 为高电平 → 无上升沿 → 中断安静，无风暴
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << 4);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&cfg);

    esp_err_t e = gpio_install_isr_service(0); // 全工程唯一 ISR 服务
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        s_rxErr = e;
        return;
    }
    e = gpio_isr_handler_add(GPIO_NUM_4, fcRiseIsr, NULL);
    if (e != ESP_OK) {
        s_rxErr = e;
        return;
    }
    s_rxSwInit = true;
}

// ---------- 轮询：取完整帧 ----------
// 帧完成标志由 ISR 在帧尾（低电平 > 50µs）置位
bool pollFcInput(FcFrame* out) {
#if FC_INPUT_ENABLE == 0
    (void)out;
    return false;
#endif
    if (!s_swFrameDone) return false;
    portENTER_CRITICAL(&s_rxLock);
    int n = (int)s_swLedCnt;
    if (n > FC_MAX_LEDS) n = FC_MAX_LEDS;
    for (int i = 0; i < n; i++) {
        // volatile 结构体不能整体赋值，逐字段拷贝
        out->leds[i].g = s_swLeds[i].g;
        out->leds[i].r = s_swLeds[i].r;
        out->leds[i].b = s_swLeds[i].b;
    }
    s_swFrameDone = false;
    portEXIT_CRITICAL(&s_rxLock);

    out->count = n;
    if (n > 0) {
        g_rxFrames = (uint32_t)(g_rxFrames + 1);
        g_rxLastLeds = (uint32_t)n;
    }
    return n > 0;
}
