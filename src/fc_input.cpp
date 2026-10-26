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
#ifndef FC_INPUT_ENABLE
#define FC_INPUT_ENABLE 1
#endif

// ============================================================
// 飞控 LED 信号解码（GPIO 中断 + ISR 内忙等测宽）
//
// 为什么不用 RMT RX：ESP32-C3 的 RMT 每通道内存 48 符号、无 DMA，
// 28 颗一帧 672 符号根本放不下，只能解出前 2 颗。
//
// 原理：GPIO4 上升沿中断 → ISR 内忙等轮询电平，逐 bit 测「高电平时长」：
//   WS2812 800kHz：高 800ns=1 / 高 400ns=0，位周期恒 1250ns。
//   一帧在一次中断里连续收完，不需要续接。
//   帧尾 = 低电平持续 > 20µs（比特间隔最长 850ns，帧间 reset ≥ 50µs）。
//
// v32.24 入门槛：位周期校验 + 整帧校验 + 自适应阈值
// v32.25 重要修正（实测 bad:fps = 10230:5，几乎全是误判）
//   原因：「(高+低) 超周期就整帧作废」把【中断延迟】当成【数据损坏】了。
//   低电平段被 WiFi/BLE 中断插进来时，低循环退出得晚 → lo 变长，
//   但 hi 的测量起点在低循环之后，值判定仍然准确。
//   于是合法的好帧被成批判死，安全阀又反复重置参数 → 阈值永远停在 600ns。
//   改为：
//   a) 帧尾超时 20µs → 100µs（WiFi/BLE 中断延迟常达数十 µs，20µs 会把一帧切碎）
//   b) 只有「高电平短到物理上不可能（<125ns）」才判真丢位 → 整帧作废
//   c) 高电平被拉长（> 阈值 2.2 倍，说明测量期间被中断占住）→ 该 bit 保守判 0，
//      只错 1 个 LSB，不整帧丢弃
//   d) 残片（总 bit < 72）静默丢弃，不计入坏帧、不触发安全阀
// ============================================================

#define FC_GPIO_BIT    (1u << 4)      // GPIO4

// 160MHz 主频：1 cycle = 6.25ns
#define CYC_TO_NS(c)   ((uint32_t)(c) * 25u / 4u)

// 帧尾判定超时 100µs（16000 cycle @160MHz）：
//   WS2812 位间低电平最长 850ns，帧间 reset ≥ 50µs（Betaflight 实际 10ms 级），
//   100µs 既能区分帧尾，又能容忍 WiFi/BLE 中断造成的数十 µs 延迟。
//   v32.24 用的 20µs 太短：中断一插就把一帧切成几段，每段 bit 数都不是 24 倍数
#define FC_RESET_TIMEOUT_CYC 16000
// 单个 bit 的高电平下限（20 cycle ≈ 125ns）：低于此说明把一段低电平当成了
// 「上升沿→下降沿」，即漏检了上升沿（真丢位）—— 这是唯一会让 24bit 分组错位、
// 必须整帧作废的情况
#define FC_HI_MIN_CYC   20
// 高电平上限 = 阈值的 2.2 倍。超过说明这个 bit 的高电平段被中断拉长了，
// 测出来的「宽」不代表真实值（0 码可能被读成 1，就是老的「暗光」根因），
// 所以该 bit 按 0 处理（保守：宁可暗一个 LSB，也不要亮点/整帧丢）
#define FC_HI_MAX_X10   22u
// 残片阈值：总 bit 少于 3 颗灯就当作被打断的碎片，静默丢弃
#define FC_FRAG_BITS    72u

// ---------- ISR 与主循环共享 ----------
static volatile uint8_t s_bad = 0;         // 本帧是否出现异常
static volatile uint32_t s_swBits = 0;
static volatile uint8_t  s_swBitCnt = 0;
// v32.23：原始三字节（不在这里还原通道顺序）
// v32.24：双缓冲。ISR 写 s_raw[s_wr]，帧通过校验后才把 s_pub 指过来，
//         主循环只读 s_raw[s_pub]，避免「一边写一边读」读出错色。
static volatile uint8_t  s_raw[2][FC_MAX_LEDS][3];
static volatile uint8_t  s_wr = 0, s_pub = 0;
static volatile int      s_swLedCnt = 0;
static volatile bool     s_swFrameDone = false;

// 自适应参数：判别阈值 & 标称位周期（cycle），随实测缓慢跟随
static volatile uint32_t s_thrCyc = 96;      // 600ns（初值，对应 800kHz WS2812）
static volatile uint32_t s_periodCyc = 200;  // 1250ns
static volatile uint8_t  s_calibrated = 0;   // 0=还没学到周期，先不做周期校验

// 本帧统计
static volatile uint32_t s_fHiMin, s_fHiMax, s_fLoMax;
static volatile uint32_t s_sumPeriod, s_nPeriod;
static volatile uint32_t s_badRun = 0;   // 连续坏帧数（安全阀：连续坏太多就重新学参数）

// ---- 通道顺序（v32.23）----
// 每个条目是「输出 r/g/b 分别取自原始第几字节」。
// 0=GRB 原始 (G,R,B) → r=第1字节 g=第0字节 b=第2字节
// 3=RBG 原始 (R,B,G) → r=第0字节 g=第2字节 b=第1字节
static const uint8_t s_orderMap[6][3] = {
    {1, 0, 2}, // 0 GRB（WS2812 标准）
    {0, 1, 2}, // 1 RGB
    {2, 0, 1}, // 2 BRG
    {0, 2, 1}, // 3 RBG
    {1, 2, 0}, // 4 GBR
    {2, 1, 0}, // 5 BGR
};
static const char* s_orderName[6] = {"GRB", "RGB", "BRG", "RBG", "GBR", "BGR"};
// 默认 3(RBG)：实测这版飞控的流里「红 → 显示绿、蓝 → 显示红」，
// 正好等于原始字节序是 (R,B,G)。
static volatile uint8_t s_rxOrder = 3;

void fcSetRxOrder(int v) {
    if (v >= 0 && v < 6) s_rxOrder = (uint8_t)v;
}
int fcRxOrder() { return (int)s_rxOrder; }
const char* fcRxOrderName() { return s_orderName[(s_rxOrder < 6) ? s_rxOrder : 0]; }

// 诊断统计
volatile uint32_t g_rxFrames = 0;    // 通过校验、已上屏的帧数
volatile uint32_t g_rxLastLeds = 0;
volatile uint32_t g_rxDone = 0;      // 中断进入且帧头可信的总次数
volatile uint32_t g_rxSyms = 0;      // 累计 bit 数
volatile uint32_t g_rxDrop = 0;      // 帧头不可靠直接丢弃
volatile uint32_t g_rxFrameEnd = 0;  // 通过校验的帧数（= g_rxFrames）
volatile uint32_t g_rxLedAtEnd = 0;  // 最近一帧解出的灯数
volatile uint32_t g_rxBad = 0;       // v32.24：数据异常被整帧丢弃的次数
volatile uint32_t g_rxSeen = 0;      // v32.24：ISR 判定「一帧结束」的总次数（含坏帧）
volatile uint32_t g_rxSkip = 0;      // v32.24：空触发（进中断时已是低电平）
volatile uint32_t g_rxClamp = 0;     // v32.25：高电平被中断拉长、该 bit 被强制判 0 的次数
volatile uint32_t g_rxFrag = 0;      // v32.25：被打断的残片（<3 颗灯）静默丢弃次数
volatile uint16_t g_rxHiMinNs = 0;   // v32.24：最近一帧最短高位（ns）
volatile uint16_t g_rxHiMaxNs = 0;   // v32.24：最近一帧最长高位（ns）
volatile uint16_t g_rxLoMaxNs = 0;   // v32.24：最近一帧最长低位（ns）
volatile uint16_t g_rxThrNs = 0;     // v32.24：当前自适应判别阈值（ns）

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
            // 原始三字节按接收顺序存入，通道顺序在 pollFcInput 里还原
            volatile uint8_t* d = s_raw[s_wr][s_swLedCnt];
            d[0] = (uint8_t)(v >> 16);
            d[1] = (uint8_t)(v >> 8);
            d[2] = (uint8_t)v;
            s_swLedCnt = (int)(s_swLedCnt + 1);
        }
        s_swBits = 0;
        s_swBitCnt = 0;
    }
    g_rxSyms = (uint32_t)(g_rxSyms + 1);
}

// ---------- 帧结束：校验 → 通过才发布，否则整帧丢弃 ----------
static void IRAM_ATTR swEndFrame(void) {
    uint32_t n = (uint32_t)s_swLedCnt;
    g_rxSeen = (uint32_t)(g_rxSeen + 1);
    g_rxLedAtEnd = n;
    // 帧内一个 bit 都没测到时 s_fHiMin 还是哨兵值，别拿去换算（会溢出）
    g_rxHiMinNs = (s_fHiMin > 2000) ? 0 : (uint16_t)CYC_TO_NS(s_fHiMin);
    g_rxHiMaxNs = (uint16_t)CYC_TO_NS(s_fHiMax);
    g_rxLoMaxNs = (uint16_t)CYC_TO_NS(s_fLoMax);

    // 校验：出现异常 / bit 数不是 24 的整数倍（丢位或多位）/ 没有灯
    if (s_bad || s_swBitCnt != 0 || n == 0) {
        // v32.25：帧被打断成小段（比如中断延迟把一帧切成两截）
        // 只是一块残片，静默丢掉：不计坏帧、也不喂安全阀，
        // 否则碎片会堆积成「连续坏 30 帧」把好不容易学到的参数重置掉
        if ((uint32_t)n * 24u + s_swBitCnt < FC_FRAG_BITS) {
            g_rxFrag = (uint32_t)(g_rxFrag + 1);
            return;
        }
        g_rxBad = (uint32_t)(g_rxBad + 1);
        // 安全阀：连续坏 200 帧（10 秒级）说明学到的阈值本身不对，
        // 回退到出厂值重新学，避免「一直判坏 → 画面永远定格」。
        // v32.25：30 → 200，原来太敏感，正常的偶发打断就会把参数打回出厂
        s_badRun++;
        if (s_badRun >= 200) {
            s_badRun = 0;
            s_calibrated = 0;
            s_thrCyc = 96;
            s_periodCyc = 200;
            g_rxThrNs = (uint16_t)CYC_TO_NS(s_thrCyc);
        }
        return;   // 不发布：屏幕上继续显示上一帧好画面，躲掉一次闪烁
    }
    s_badRun = 0;

    // 学到参数：最短/最长高位取中点当阈值（要求两者分得开，即本帧 0/1 都有）
    // 第一次直接采用，之后按 1/8 平滑跟随 —— 刷机后第二帧就能用上正确阈值
    uint32_t hmin = s_fHiMin, hmax = s_fHiMax;
    if (hmin != 0xFFFFFFFFu && hmax > 0) {
        // v32.25：hmin 可能是被中断截短的假值（实测见过 212ns，而真实 0 码 400ns）。
        // 拿它算中点会把阈值拉低，导致 1 码被误判成 0（整条灯偏暗），
        // 所以限制 hmin 不低于 hmax 的 2/5（0 码/1 码比值理论是 0.5）
        if (hmin < hmax * 2u / 5u) hmin = hmax * 2u / 5u;
        if (hmax > hmin + hmin / 2u) {
            uint32_t t = (hmin + hmax) / 2u;
            if (t < 24) t = 24;
            if (t > 300) t = 300;
            s_thrCyc = s_calibrated ? ((s_thrCyc * 7 + t) / 8) : t;
        }
    }
    // 周期只作诊断参考（v32.25 起不再拿它判坏帧，理由见文件头）
    if (s_nPeriod > 8) {
        uint32_t p = s_sumPeriod / s_nPeriod;
        if (p > 40 && p < 800) {
            s_periodCyc = s_calibrated ? ((s_periodCyc * 7 + p) / 8) : p;
        }
    }
    s_calibrated = 1;
    g_rxThrNs = (uint16_t)CYC_TO_NS(s_thrCyc);

    // 发布这一帧（双缓冲交换）
    s_pub = s_wr;
    s_wr = (uint8_t)(s_wr ^ 1);
    s_swFrameDone = true;
    g_rxFrameEnd = (uint32_t)(g_rxFrameEnd + 1);
}

// ---------- GPIO 上升沿中断：一次中断收完整帧 ----------
static void IRAM_ATTR fcRiseIsr(void* arg) {
    (void)arg;

    // 第一个 bit 只测到「剩余的」高电平（中断入口有延迟），只用来判定帧头可信度
    uint32_t a0 = esp_cpu_get_cycle_count();
    while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
        if ((esp_cpu_get_cycle_count() - a0) > FC_RESET_TIMEOUT_CYC) {
            return;   // 帧间空闲高 / 延迟边沿：直接返回，不清任何状态
        }
    }
    uint32_t firstHi = esp_cpu_get_cycle_count() - a0;
    if (firstHi < FC_HI_MIN_CYC) {
        // 进入中断时电平已经是低（多半是帧内被锁存的边沿又触发了一次）
        g_rxSkip = (uint32_t)(g_rxSkip + 1);
        return;
    }

    // 确认是新帧：清状态（此时才动 s_raw，避免打乱已发布的那一帧）
    s_swBits = 0;
    s_swBitCnt = 0;
    s_swLedCnt = 0;
    s_swFrameDone = false;   // 上一帧若还没被取走，就作废，不让它被重复上屏
    s_bad = 0;
    s_fHiMin = 0xFFFFFFFFu;
    s_fHiMax = 0;
    s_fLoMax = 0;
    s_sumPeriod = 0;
    s_nPeriod = 0;
    g_rxDone = (uint32_t)(g_rxDone + 1);

    // 首位阈值略降（5/6）：中断入口延迟只会让测到的高电平变短
    swPushBit(firstHi > (s_thrCyc * 5u / 6u));

    while (1) {
        // ---- 低电平段（bit 之间的间隔）----
        uint32_t w0 = esp_cpu_get_cycle_count();
        uint32_t lo;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) == 0) {
            lo = esp_cpu_get_cycle_count() - w0;
            if (lo > FC_RESET_TIMEOUT_CYC) {   // 帧尾 reset
                swEndFrame();
                return;
            }
        }
        lo = esp_cpu_get_cycle_count() - w0;

        // ---- 高电平段（决定这一 bit 是 0 还是 1）----
        uint32_t h0 = esp_cpu_get_cycle_count();
        uint32_t hi;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
            hi = esp_cpu_get_cycle_count() - h0;
            if (hi > FC_RESET_TIMEOUT_CYC) {   // 飞控掉线后线被上拉成常高
                swEndFrame();
                return;
            }
        }
        hi = esp_cpu_get_cycle_count() - h0;

        // 真丢位判定：高电平短到物理上不可能（<125ns）才算。
        // 这是唯一会让 24bit 分组整体错位、必须整帧作废的情况
        if (hi < FC_HI_MIN_CYC) s_bad = 1;
        if (hi > s_fHiMax) s_fHiMax = hi;
        if (hi < s_fHiMin) s_fHiMin = hi;
        if (lo > s_fLoMax) s_fLoMax = lo;
        s_sumPeriod += hi + lo;
        s_nPeriod++;
        // 值判定：高电平被中断拉长时（> 阈值 2.2 倍，正常 1 码上限约 1.2µs），
        // 测到的宽度不代表真实电平宽度，0 码可能被读成 1（老的「暗光」根因），
        // 所以这一 bit 保守判 0：只错 1 个 LSB，不整帧丢弃、不闪整条灯带
        if (hi * 10u > s_thrCyc * FC_HI_MAX_X10) {
            g_rxClamp = (uint32_t)(g_rxClamp + 1);
            swPushBit(false);
        } else {
            swPushBit(hi > s_thrCyc);
        }
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
    g_rxThrNs = (uint16_t)CYC_TO_NS(s_thrCyc);
    s_rxSwInit = true;
}

// ---------- 轮询：取完整帧 ----------
bool pollFcInput(FcFrame* out) {
#if FC_INPUT_ENABLE == 0
    (void)out;
    return false;
#endif
    if (!s_swFrameDone) return false;
    const uint8_t* m = s_orderMap[(s_rxOrder < 6) ? s_rxOrder : 0];
    uint8_t p = s_pub;                 // 锁定缓冲：ISR 之后只会写另一个缓冲
    int n = (int)s_swLedCnt;
    if (n > FC_MAX_LEDS) n = FC_MAX_LEDS;
    for (int i = 0; i < n; i++) {
        const volatile uint8_t* s = s_raw[p][i];
        out->leds[i].r = s[m[0]];
        out->leds[i].g = s[m[1]];
        out->leds[i].b = s[m[2]];
    }
    s_swFrameDone = false;

    out->count = n;
    if (n > 0) {
        g_rxFrames = (uint32_t)(g_rxFrames + 1);
        g_rxLastLeds = (uint32_t)n;
    }
    return n > 0;
}

// ---------- 诊断：把最近一帧前几颗灯的原始字节打成十六进制 ----------
// 用于对照飞控设置（比如飞控全设红 → 这里应该出现一串代表红的字节）
void fcRxHeadHex(char* buf, int buflen) {
    if (buf == NULL || buflen < 4) return;
    buf[0] = '\0';
    if (!s_swFrameDone && g_rxFrameEnd == 0) {
        snprintf(buf, (size_t)buflen, "-");
        return;
    }
    const uint8_t* m = s_orderMap[(s_rxOrder < 6) ? s_rxOrder : 0];
    int n = (int)g_rxLastLeds;
    if (n > 4) n = 4;
    uint8_t p = s_pub;
    int pos = 0;
    for (int i = 0; i < n && pos < buflen - 8; i++) {
        const volatile uint8_t* s = s_raw[p][i];
        pos += snprintf(buf + pos, (size_t)(buflen - pos), "%02X%02X%02X ",
                        s[m[0]], s[m[1]], s[m[2]]);
    }
    if (pos > 0 && buf[pos - 1] == ' ') buf[pos - 1] = '\0';
}
