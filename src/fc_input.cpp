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
// 45 颗一帧 1080 符号根本放不下。
//
// 原理：GPIO4 上升沿中断 → 屏蔽中断后忙等轮询电平，逐 bit 测高电平时长：
//   WS2812 800kHz：高 800ns=1 / 高 400ns=0，位周期恒 1250ns。
//
// ---- v32.26：三次失败后的结论 —— 采集本身不可靠，别再修判据 ----
//   v32.22 能出图但颜色不对、偶尔闪；v32.24 加校验后 bad:frames=10230:5；
//   v32.25 放宽校验后 bad:frames=9243:0。三次都是同一件事：
//   **忙等采集期间被 WiFi/BLE/系统 tick 打断，测出来的位宽是错的**，
//   于是①位宽判错（灭灯区发暗光）②丢掉/并掉 bit 让 24bit 分组错位（闪乱色）
//   ③整帧 bit 数不是 24 的倍数（v32.24/25 的校验把它全判坏 → 画面完全不动）。
//
//   本次三条改法：
//   a) 【关键】进入 ISR 后立刻把 INTENABLE 清零，整个帧在「无中断」状态下采完
//      （45 颗 ≈ 1.35ms）。这是唯一能让测量准确的办法；代价是 WiFi/BLE 中断
//      被推迟 1.35ms，30Hz 下占空比 4%，实测无影响。
//   b) 位数据改成【原始 bit 数组】，帧尾再按 24 对齐分组：丢掉帧头 cnt%24 个 bit，
//      使分组终点正好落在帧尾 —— 不管帧头丢了几位，后面的 LED 一定对齐，
//      不会像原来那样「从帧头开始每 24 位切一刀」把颜色整体转位。
//   c) 取消「整帧作废」：只要还剩 ≥1 颗灯就上屏。采集既然可靠了，就不需要
//      用丢帧来遮丑；丢帧只会让画面冻住。
// ============================================================

#define FC_GPIO_BIT       (1u << 4)   // GPIO4

// 160MHz 主频：1 cycle = 6.25ns
#define CYC_TO_NS(c)      ((uint32_t)(c) * 25u / 4u)

// 帧尾判定：低电平超过 50µs 认为一帧结束（WS2812 位间低最长 850ns，
// Betaflight 帧间 reset 远大于 50µs）。v32.25 的 100µs 会把下一帧的开头
// 一起吞进来，50µs 足够区分又不会跨帧。
#define FC_GAP_CYC        8000u
// 进中断后等线变低的超时：200µs。超过说明线是常高（飞控没在发 / 掉线）
#define FC_ENTRY_MAX_CYC  32000u
// 单帧捕获硬上限：5ms（45 颗约 1.35ms，留 3 倍余量）。防止线被拉成常低时
// 在「无中断」状态下卡死，把 WiFi 一起拖垮
#define FC_CAP_MAX_CYC    800000u
// 高电平短到物理上不可能的最小值（20 cycle ≈ 125ns）→ 漏检上升沿
#define FC_HI_MIN_CYC     20u
// 高电平上限 = 阈值的 2.2 倍。超过说明测量被拉长，该 bit 按 0 处理
#define FC_HI_MAX_X10     22u

#define FC_MAX_BITS       (FC_MAX_LEDS * 24)
#define FC_BITS_BYTES     (FC_MAX_BITS / 8)

// ---------- 中断全屏蔽 ----------
// 直接读写 INTENABLE：比 portDISABLE_INTERRUPTS()（只压到 XCHAL_EXCM_LEVEL）
// 更彻底，能把 WiFi/BLE 的 3~5 级中断一起挡在外面。ISR 退出汇编只恢复 PS，
// 不碰 INTENABLE，所以这里改了不会被「还原」，必须自己恢复。
static inline uint32_t IRAM_ATTR fcIntrOffAll(void) {
    uint32_t old;
    __asm__ __volatile__("rsr %0, INTENABLE" : "=a"(old));
    __asm__ __volatile__("wsr %0, INTENABLE\n\trsync" :: "a"(0u));
    return old;
}
static inline void IRAM_ATTR fcIntrRestore(uint32_t old) {
    __asm__ __volatile__("wsr %0, INTENABLE\n\trsync" :: "a"(old));
}

// ---------- ISR 与主循环共享 ----------
// 采集开关：只有 Betaflight 模式或标定中才需要飞控帧。关掉时 ISR 立刻返回，
// 既不屏蔽中断也不占 CPU（本地灯效运行时完全不受飞控信号影响）
static volatile bool     s_active = true;
// bit 数组双缓冲：ISR 写 s_wr，收完一帧把 s_pub 指过来；主循环只读 s_pub。
static uint8_t           s_bits[2][FC_BITS_BYTES];
static volatile uint16_t s_nbits[2];
static volatile uint8_t  s_wr = 0, s_pub = 0;
static volatile bool     s_frameReady = false;

// 自适应参数：判别阈值（cycle），随实测缓慢跟随
static volatile uint32_t s_thrCyc = 96;      // 600ns（初值，对应 800kHz WS2812）
static volatile uint8_t  s_cal = 0;          // 0=还没学到，第一帧直接采用

// 本帧统计
static volatile uint32_t s_fHiMin, s_fHiMax, s_fLoMax;
static volatile uint32_t s_sumPeriod, s_nPeriod;

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
// 默认 3(RBG)：v32.23 按实测「红→绿、蓝→红」反推得到，保留不动。
// 若某天换了飞控/改了 ledstrip_grb_rgb，在网页「连接」页点一下就能换。
static volatile uint8_t s_rxOrder = 3;

void fcSetRxOrder(int v) {
    if (v >= 0 && v < 6) s_rxOrder = (uint8_t)v;
}
int fcRxOrder() { return (int)s_rxOrder; }
const char* fcRxOrderName() { return s_orderName[(s_rxOrder < 6) ? s_rxOrder : 0]; }

// 诊断统计
volatile uint32_t g_rxFrames = 0;    // 已上屏的帧数
volatile uint32_t g_rxLastLeds = 0;
volatile uint32_t g_rxDone = 0;      // 进入 ISR 并开始采集的次数
volatile uint32_t g_rxSyms = 0;      // 累计 bit 数
volatile uint32_t g_rxDrop = 0;      // 线常高直接放弃的次数
volatile uint32_t g_rxFrameEnd = 0;  // 收完整帧的总次数（= 上屏帧数）
volatile uint32_t g_rxLedAtEnd = 0;  // 最近一帧解出的灯数
volatile uint32_t g_rxSeen = 0;      // ISR 判定「一帧结束」的总次数
volatile uint32_t g_rxSkip = 0;      // 保留字段（旧日志用）
volatile uint32_t g_rxBad = 0;       // 出现过「高电平短到不可能」的帧数（干扰指标）
volatile uint32_t g_rxClamp = 0;     // 高电平被拉长、该 bit 强制判 0 的次数
volatile uint32_t g_rxFrag = 0;      // 不足 1 颗灯的残片，丢弃次数
volatile uint32_t g_rxDropBits = 0;  // 最近一帧帧头丢掉的 bit 数（= cnt%24）
volatile uint16_t g_rxHiMinNs = 0;   // 最近一帧最短高位（ns）
volatile uint16_t g_rxHiMaxNs = 0;   // 最近一帧最长高位（ns）
volatile uint16_t g_rxLoMaxNs = 0;   // 最近一帧最长低位（ns）
volatile uint16_t g_rxThrNs = 0;     // 当前自适应判别阈值（ns）
volatile uint16_t g_rxCapUs = 0;     // 最近一帧的捕获耗时（µs），反映中断屏蔽时长

static esp_err_t s_rxErr = ESP_OK;
static bool s_rxSwInit = false;

bool fcRxOk() { return s_rxSwInit; }
int  fcRxErr() { return (int)s_rxErr; }

void fcRxSetActive(bool on) { s_active = on; }
bool fcRxActive() { return s_active; }

// ---------- 帧结束：对齐 → 发布 ----------
static inline void IRAM_ATTR fcEndFrame(uint16_t cnt, uint32_t t0) {
    g_rxSeen = (uint32_t)(g_rxSeen + 1);
    g_rxLedAtEnd = (uint32_t)(cnt / 24);
    g_rxDropBits = (uint32_t)(cnt % 24);
    g_rxCapUs = (uint16_t)((esp_cpu_get_cycle_count() - t0) * 25u / 4u / 1000u);
    // 帧内一个 bit 都没测到时 s_fHiMin 还是哨兵值，别拿去换算（会溢出）
    g_rxHiMinNs = (s_fHiMin > 2000) ? 0 : (uint16_t)CYC_TO_NS(s_fHiMin);
    g_rxHiMaxNs = (uint16_t)CYC_TO_NS(s_fHiMax);
    g_rxLoMaxNs = (uint16_t)CYC_TO_NS(s_fLoMax);

    if (cnt < 24) {
        g_rxFrag = (uint32_t)(g_rxFrag + 1);   // 残片：连一颗灯都凑不齐
        return;
    }

    // 学到阈值：最短/最长高位取中点（要求两者分得开，即本帧 0/1 都有）。
    // 采集已经无中断干扰，hmin 会稳定落在 400ns 附近；仍保留「不低于 hmax*2/5」
    // 的保护，防止某帧只有单色（全是 0 码或全是 1 码）时把阈值带偏。
    uint32_t hmin = s_fHiMin, hmax = s_fHiMax;
    if (hmin != 0xFFFFFFFFu && hmax > 0) {
        if (hmin < hmax * 2u / 5u) hmin = hmax * 2u / 5u;
        if (hmax > hmin + hmin / 2u) {
            uint32_t t = (hmin + hmax) / 2u;
            if (t < 24) t = 24;
            if (t > 300) t = 300;
            // 第一次直接采用（刷机后第一帧就准），之后 1/8 平滑跟随
            s_thrCyc = s_cal ? ((s_thrCyc * 7 + t) / 8) : t;
            s_cal = 1;
        }
    }
    g_rxThrNs = (uint16_t)CYC_TO_NS(s_thrCyc);

    // 发布（双缓冲交换）
    s_nbits[s_wr] = cnt;
    s_pub = s_wr;
    s_wr = (uint8_t)(s_wr ^ 1);
    s_frameReady = true;
    g_rxFrameEnd = (uint32_t)(g_rxFrameEnd + 1);
}

// ---------- GPIO 上升沿中断：一次中断收完整帧 ----------
static void IRAM_ATTR fcRiseIsr(void* arg) {
    (void)arg;
    if (!s_active) return;      // 不需要飞控灯的场合（本地灯效）直接走开

    // 【核心】进入就关掉所有可屏蔽中断。忙等期间哪怕只被打断一次，
    // 这个 bit 的宽度就是错的（WiFi/BLE 中断延迟可达数十 µs）
    uint32_t intSaved = fcIntrOffAll();
    uint32_t t0 = esp_cpu_get_cycle_count();

    // 1) 等当前这个高电平结束。我们是上升沿之后几百 ns~几 µs 才进来的，
    //    位宽已经被截短，所以这个 bit 不要（丢几位由帧尾 24 对齐补回来）
    while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
        if ((esp_cpu_get_cycle_count() - t0) > FC_ENTRY_MAX_CYC) {
            g_rxDrop = (uint32_t)(g_rxDrop + 1);   // 线常高：没在发数据
            fcIntrRestore(intSaved);
            return;
        }
    }

    // 2) 复位本帧统计
    uint16_t cnt = 0;
    uint32_t acc = 0, accN = 0;
    uint8_t* wp = s_bits[s_wr];
    memset(wp, 0, FC_BITS_BYTES);
    s_fHiMin = 0xFFFFFFFFu;
    s_fHiMax = 0;
    s_fLoMax = 0;
    s_sumPeriod = 0;
    s_nPeriod = 0;
    uint8_t shortHi = 0;
    g_rxDone = (uint32_t)(g_rxDone + 1);

    uint32_t w0 = 0, lo = 0, h0 = 0, hi = 0;

    while (1) {
        // ---- 低电平段（bit 之间的间隔）----
        w0 = esp_cpu_get_cycle_count();
        lo = 0;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) == 0) {
            lo = esp_cpu_get_cycle_count() - w0;
            if (lo > FC_GAP_CYC) goto cap_done;                  // 帧尾 reset
            if ((w0 - t0) > FC_CAP_MAX_CYC) goto cap_done;       // 整体超时
        }

        // ---- 高电平段（决定这一 bit 是 0 还是 1）----
        h0 = esp_cpu_get_cycle_count();
        hi = 0;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
            hi = esp_cpu_get_cycle_count() - h0;
            if (hi > FC_GAP_CYC) goto cap_done;                  // 线被拉高 / 帧尾
            if ((h0 - t0) > FC_CAP_MAX_CYC) goto cap_done;       // 整体超时
        }

        // ---- 统计（只做诊断，不再拿这些判据整帧作废）----
        if (hi < FC_HI_MIN_CYC) shortHi = 1;
        if (hi > s_fHiMax) s_fHiMax = hi;
        if (hi < s_fHiMin) s_fHiMin = hi;
        if (lo > s_fLoMax) s_fLoMax = lo;
        s_sumPeriod += hi + lo;
        s_nPeriod++;

        // ---- 值判定 ----
        bool one;
        if (hi * 10u > s_thrCyc * FC_HI_MAX_X10) {
            g_rxClamp = (uint32_t)(g_rxClamp + 1);   // 测宽被拉长，保守判 0
            one = false;
        } else {
            one = (hi > s_thrCyc);
        }

        // ---- 压入 bit（按字节攒，省得每个 bit 都读改写）----
        if (cnt < FC_MAX_BITS) {
            acc = (acc << 1) | (one ? 1u : 0u);
            if (++accN == 8) {
                *wp++ = (uint8_t)acc;
                acc = 0;
                accN = 0;
            }
            cnt++;
        }
    }

cap_done:
    if (accN) *wp = (uint8_t)(acc << (8 - accN));   // 末尾不满一字节，低位补 0
    if (shortHi) g_rxBad = (uint32_t)(g_rxBad + 1);
    fcEndFrame(cnt, t0);
    fcIntrRestore(intSaved);
}

// ---------- 从 bit 数组取第 i 颗灯 ----------
static inline void fcGetLed(const uint8_t* base, uint16_t start, int i, uint8_t raw[3]) {
    uint32_t v = 0;
    int b0 = (int)start + i * 24;
    for (int b = 0; b < 24; b++) {
        int idx = b0 + b;
        v = (v << 1) | (uint32_t)((base[idx >> 3] >> (7 - (idx & 7))) & 1u);
    }
    raw[0] = (uint8_t)(v >> 16);
    raw[1] = (uint8_t)(v >> 8);
    raw[2] = (uint8_t)v;
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
    if (!s_frameReady) return false;
    uint8_t p = s_pub;                       // 锁定缓冲：ISR 之后只会写另一个
    uint16_t cnt = s_nbits[p];
    s_frameReady = false;

    // 帧头丢掉的 bit 数 = cnt % 24，丢掉它们让分组终点正好落在帧尾。
    // 这样即使中断进晚了、头几位没采到，后面的 LED 仍然一颗不差地对齐。
    uint16_t start = (uint16_t)(cnt % 24);
    const uint8_t* base = s_bits[p];
    const uint8_t* m = s_orderMap[(s_rxOrder < 6) ? s_rxOrder : 0];
    int n = (int)((cnt - start) / 24);
    if (n > FC_MAX_LEDS) n = FC_MAX_LEDS;

    for (int i = 0; i < n; i++) {
        uint8_t raw[3];
        fcGetLed(base, start, i, raw);
        out->leds[i].r = raw[m[0]];
        out->leds[i].g = raw[m[1]];
        out->leds[i].b = raw[m[2]];
    }

    // 帧头那一位必然采不到（中断进不来那么快），对齐后正好少一颗灯。
    // 把它补回来：向前复制一颗，这样灯带末端不会缺一段，飞控灯数也和实际一致。
    // 代价是被补的那颗显示的是下一颗的颜色 —— 纯色/扫描类效果看不出来
    if (start > 0 && n >= 1 && n < FC_MAX_LEDS) {
        for (int i = n; i > 0; i--) out->leds[i] = out->leds[i - 1];
        out->leds[0] = out->leds[1];
        n++;
    }

    out->count = n;
    if (n > 0) {
        g_rxFrames = (uint32_t)(g_rxFrames + 1);
        g_rxLastLeds = (uint32_t)n;
    }
    return n > 0;
}

// ---------- 诊断：把最近一帧前几颗灯的原始字节打成十六进制 ----------
// 用于对照飞控设置（飞控全设红 → 这里应出现代表红的字节）
void fcRxHeadHex(char* buf, int buflen) {
    if (buf == NULL || buflen < 4) return;
    buf[0] = '\0';
    if (g_rxFrameEnd == 0) {
        snprintf(buf, (size_t)buflen, "-");
        return;
    }
    uint8_t p = s_pub;
    uint16_t cnt = s_nbits[p];
    uint16_t start = (uint16_t)(cnt % 24);
    int n = (int)((cnt - start) / 24);
    if (n > 4) n = 4;
    int pos = 0;
    for (int i = 0; i < n && pos < buflen - 10; i++) {
        uint8_t raw[3];
        fcGetLed(s_bits[p], start, i, raw);
        pos += snprintf(buf + pos, (size_t)(buflen - pos), "%02X%02X%02X ",
                        raw[0], raw[1], raw[2]);
    }
    if (pos > 0 && buf[pos - 1] == ' ') buf[pos - 1] = '\0';
}
