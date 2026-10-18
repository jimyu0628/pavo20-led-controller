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
// 软件解码 v18：GPIO 任意边沿中断 + 交替状态机（纯 cycle，无电平读、无定时器）
// （替代 RMT RX —— ESP32-C3 的 RMT 每通道内存上限 64 符号，
//  无 DMA、无 partial 流式，28 颗长帧(672 符号)续接必然丢段，只解出前 2 颗）
//
// 原理：WS2812 信号严格"高→低→高→低"交替（每 bit = 1 个高 + 1 个低）。
// ANYEDGE 中断下，每 2 个边沿 = 1 bit：第 1 个边沿记录时刻（高开始），
// 第 2 个边沿用「时刻差 - 高起点」得到该 bit 的高电平时长
// （高 800ns=1 / 高 400ns=0，阈值 600ns）。
//
// 帧头对齐：帧间空闲低电平（reset ≥50µs / 帧间 10~20ms）后，WS2812 帧
// 必然以"低→高"上升沿开始（bit1）。因此：距最后边沿 >2ms 时，下一个边沿
// 强制视为帧头上升沿并重置累积 → 每帧重新同步，不依赖读电平判方向。
//
// 中断丢失容错：偶发丢边沿会导致本帧错位（颜色偶乱），但每帧帧头重新同步，
// 下一帧即恢复。ISR 前缀仅 cycle 读取（~200ns），远小于 400ns 最短 bit。
// 帧完成：最后边沿后 >2ms 无活动（主循环判定并取帧）。
// ============================================================

#define FC_GPIO_BIT    (1u << 4)      // GPIO4
// 160MHz 主频：1 cycle = 6.25ns
#define FC_BIT_THRESHOLD_CYC 96       // 高电平 ≥600ns 判 1（bit1 高 800ns / bit0 高 400ns）
#define FC_FRAME_IDLE_CYC   320000    // 2ms @160MHz：帧内边沿间隔 ≤850ns，2ms 远大于帧尾 reset
#define FC_ABNORMAL_HI_CYC  256       // 高时长 >1.6µs 视为噪声/异常，忽略该 bit

// ---------- ISR 与主循环共享 ----------
static portMUX_TYPE s_rxLock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_riseCyc = 0;    // 当前 bit 高起点时刻（cycle）
static volatile bool     s_expectRise = true; // 状态机：期望上升沿（帧头对齐后交替）
static volatile uint32_t s_swBits = 0;     // 累积 bit（GRB 高位在前）
static volatile uint8_t  s_swBitCnt = 0;
static volatile RGB      s_swLeds[FC_MAX_LEDS];
static volatile int      s_swLedCnt = 0;
static volatile uint32_t s_swLastCyc = 0;  // 最后边沿时刻（cycle，帧判定用）

// 诊断统计
volatile uint32_t g_rxFrames = 0;
volatile uint32_t g_rxLastLeds = 0;
volatile uint32_t g_rxDone = 0;      // 软件解码：帧头（新帧）次数
volatile uint32_t g_rxSyms = 0;      // 软件解码：累计 bit 数
volatile uint32_t g_rxDrop = 0;      // 软件解码：异常/无配对边沿丢弃计数
volatile uint32_t g_rxFrameEnd = 0;  // 软件解码：主循环取帧次数
volatile uint32_t g_rxLedAtEnd = 0;  // 软件解码：最近取帧的灯数
static esp_err_t s_rxErr = ESP_OK;
static bool s_rxSwInit = false;

bool fcRxOk() { return s_rxSwInit; }
int  fcRxErr() { return (int)s_rxErr; }

// ---------- bit 累积（GRB 高位在前） ----------
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

// ---------- GPIO 任意边沿中断（状态机，纯 cycle，极短） ----------
static void IRAM_ATTR fcEdgeIsr(void* arg) {
    (void)arg;
    uint32_t cyc = esp_cpu_get_cycle_count();

    // 帧间空闲（距最后边沿 >2ms）后的边沿：帧间低电平 → WS2812 帧必以
    // 上升沿开始 → 强制作为新帧帧头（重置累积，期望下一次是下降沿）
    if (cyc - s_swLastCyc > FC_FRAME_IDLE_CYC) {
        s_swBits = 0;
        s_swBitCnt = 0;
        s_swLedCnt = 0;
        s_riseCyc = cyc;          // 帧头上升沿：记录 bit1 高起点
        s_expectRise = false;     // 下一期望：下降沿
        g_rxDone = (uint32_t)(g_rxDone + 1);
        s_swLastCyc = cyc;
        return;
    }

    if (s_expectRise) {
        s_riseCyc = cyc;
        s_expectRise = false;
    } else {
        uint32_t hi = cyc - s_riseCyc;
        s_expectRise = true;
        if (hi > FC_ABNORMAL_HI_CYC) {
            g_rxDrop = (uint32_t)(g_rxDrop + 1); // 异常高：噪声，忽略该 bit
        } else {
            swPushBit(hi > FC_BIT_THRESHOLD_CYC);
        }
    }
    s_swLastCyc = cyc;
}

// ---------- 初始化 ----------
void fcInputInit() {
#if FC_INPUT_ENABLE == 0
    printf("[RX] DISABLED (FC_INPUT_ENABLE=0)\n");
    return;
#endif
    s_rxErr = ESP_OK;
    // 上拉：未接飞控时 GPIO4 为高电平 → 无边沿 → 中断安静，无风暴
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << 4);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&cfg);

    esp_err_t e = gpio_install_isr_service(0); // 全工程唯一 ISR 服务
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        s_rxErr = e;
        return;
    }
    e = gpio_isr_handler_add(GPIO_NUM_4, fcEdgeIsr, NULL);
    if (e != ESP_OK) {
        s_rxErr = e;
        return;
    }
    s_rxSwInit = true;
}

// ---------- 轮询：取完整帧 ----------
// 帧完成判定：已收到 ≥1 颗灯 且 距最后边沿 >2ms（帧尾 reset/帧间空闲）
bool pollFcInput(FcFrame* out) {
#if FC_INPUT_ENABLE == 0
    (void)out;
    return false;
#endif
    if (s_swLedCnt == 0) return false;
    uint32_t nowCyc = esp_cpu_get_cycle_count();
    if (nowCyc - s_swLastCyc < FC_FRAME_IDLE_CYC) return false; // 帧仍在进行

    portENTER_CRITICAL(&s_rxLock);
    int n = (int)s_swLedCnt;
    if (n > FC_MAX_LEDS) n = FC_MAX_LEDS;
    for (int i = 0; i < n; i++) {
        // volatile 结构体不能整体赋值，逐字段拷贝
        out->leds[i].g = s_swLeds[i].g;
        out->leds[i].r = s_swLeds[i].r;
        out->leds[i].b = s_swLeds[i].b;
    }
    s_swLedCnt = 0;
    s_swBits = 0;
    s_swBitCnt = 0;
    s_expectRise = true;
    portEXIT_CRITICAL(&s_rxLock);

    out->count = n;
    if (n > 0) {
        g_rxFrames = (uint32_t)(g_rxFrames + 1);
        g_rxLastLeds = (uint32_t)n;
        g_rxFrameEnd = (uint32_t)(g_rxFrameEnd + 1);
        g_rxLedAtEnd = (uint32_t)n;
    }
    return n > 0;
}
