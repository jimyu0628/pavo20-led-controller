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
//   a) 【关键】进入 ISR 后立刻进临界区（RISC-V 上等于关全局中断使能），
//      整个帧在「无中断」状态下采完（45 颗 ≈ 1.35ms）。这是唯一能让测量
//      准确的办法；代价是 WiFi/BLE 中断被推迟 1.35ms，30Hz 下占空比 4%。
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
// v32.27：低于这个宽度的高位当「毛刺」处理（40 cycle ≈ 250ns，只有 T0H=400ns 的
// 六成长度）。这种样本一旦混进「最短高位 hmin」，学出来的阈值就会被拖偏 ——
// v32.26 现场日志 `高位 0~0ns · 阈值 493ns` 就是这么来的（(186+800)/2=493），
// 而 0 码高电平 400ns 距 493ns 只剩 93ns 余量，稍有抖动就把 0 判成 1，
// 于是「本该灭的灯发弱光」。现在毛刺不参与学阈值，阈值自然回到 600ns（0/1 各留 200ns）。
#define FC_HI_GLITCH_CYC  40u
// 高电平上限 = 阈值的 2.2 倍。超过说明测量被拉长，该 bit 按 0 处理
#define FC_HI_MAX_X10     22u

#define FC_MAX_BITS       (FC_MAX_LEDS * 24)
#define FC_BITS_BYTES     (FC_MAX_BITS / 8)

// ---------- 采集期间的中断屏蔽 ----------
// ESP32-C3 是 RISC-V 内核，所有外设中断共用 mstatus.MIE 一个总开关，
// portENTER_CRITICAL_ISR 会把它清零 —— WiFi / BLE / 系统 tick 全部推迟到
// 采完之后，测量不会再被打断。比 portDISABLE_INTERRUPTS() 语义更明确，
// 也比手写 CSR 汇编可移植（那边 rsr/wsr 是 Xtensa 指令，C3 上编不过）。
static portMUX_TYPE s_capMux = portMUX_INITIALIZER_UNLOCKED;

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
// v32.27：本帧「按当前阈值分成两簇」的高位总和/个数。阈值就取两簇均值的中点 ——
// 这是本次修「灭灯区发弱光」的关键：
//   忙等测宽本身有系统性偏差（进循环取 h0 晚几拍、退出判定的那次循环又少算一拍，
//   实测比真值短 100~150ns），所以 0 码测出来约 270ns、1 码约 670ns。
//   旧代码用「本帧最短/最长高位」的中点定阈值 —— 这两个值恰恰是离群点，
//   某一帧里混进一个 186ns 的毛刺，阈值就被拽到 493ns，
//   距 0 码只有 93ns（约 15 个 CPU 周期）的余量，抖一下 0 就判成 1，
//   那些本该灭掉的灯于是发出很弱的杂色。
//   改成两簇均值中点后：偏差对两簇是同等的，中点自动落在真正该在的位置；
//   而且上千个 bit 做平均，个别毛刺根本撼动不了它。
static volatile uint32_t s_h0Sum, s_h0N, s_h1Sum, s_h1N;

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
// 默认 5(BGR)：v32.23 按「红→绿、蓝→红」反推成 RBG，v32.26 实测只有 BGR 颜色才对
// （红显示红、蓝显示蓝）。BGR/RBG 互换说明两端字节序正好差了一次 r↔b 交换，
// 属于正常现象 —— 飞控的 ledstrip_grb_rgb 设置、灯珠型号、走线都有影响，
// 所以做成可切换并断电记忆；这里只是把「这台硬件实测对了」的那个设成出厂默认。
static volatile uint8_t s_rxOrder = 5;

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
    uint32_t capUs = (uint32_t)((esp_cpu_get_cycle_count() - t0) * 25u / 4u / 1000u);

    // v32.27：残片（连一颗灯都没凑齐）不再覆盖「本帧诊断」。
    // 原来这里先写 g_rxLedAtEnd / g_rxCapUs 再判断残片，而采完一帧之后必然还有
    // 一次「幽灵中断」（见 fcRiseIsr 末尾的说明）会采到 0 bit，
    // 于是网页日志上永远显示 末帧 0 颗 · 高位 0~0ns · 捕获 51µs ——
    // 全是那个空帧的读数，把真帧的质量数据盖掉了（v32.26 现场日志正是如此）。
    if (cnt < 24) {
        g_rxFrag = (uint32_t)(g_rxFrag + 1);
        return;
    }

    g_rxLedAtEnd = (uint32_t)(cnt / 24);
    g_rxDropBits = (uint32_t)(cnt % 24);
    g_rxCapUs = (uint16_t)((capUs > 0xFFFFu) ? 0xFFFFu : capUs);
    // 帧内一个 bit 都没测到时 s_fHiMin 还是哨兵值，别拿去换算（会溢出）
    g_rxHiMinNs = (s_fHiMin > 2000) ? 0 : (uint16_t)CYC_TO_NS(s_fHiMin);
    g_rxHiMaxNs = (uint16_t)CYC_TO_NS(s_fHiMax);
    g_rxLoMaxNs = (uint16_t)CYC_TO_NS(s_fLoMax);

    // 学到阈值。主路：两簇均值取中点（上千 bit 平均，抗毛刺，且自动抵消测宽偏差）。
    // 备用路：两簇分不开时（整帧全黑或全亮、或阈值一开始就偏到离谱）退回「最短/最长
    // 高位的中点」，并对明显是毛刺的 hmin 先做闸门过滤。
    uint32_t hmin = s_fHiMin, hmax = s_fHiMax;
    if (hmin < FC_HI_GLITCH_CYC) hmin = 0;         // 毛刺：本帧不参与
    bool learned = false;
    if (s_h0N >= 8 && s_h1N >= 8) {
        uint32_t a0 = s_h0Sum / s_h0N;             // 0 码实测均值
        uint32_t a1 = s_h1Sum / s_h1N;             // 1 码实测均值
        if (a1 > a0 + a0 / 3u) {                   // 两簇确实分得开（>1.33 倍）
            uint32_t t = (a0 + a1) / 2u;
            // 两簇是平均过的，几乎不含抖动，可以直接跟得紧一些
            s_thrCyc = s_cal ? ((s_thrCyc * 3 + t) / 4) : t;
            s_cal = 1;
            learned = true;
        }
    }
    if (!learned && hmin > 0 && hmax > hmin + hmin / 2u) {
        uint32_t t = (hmin + hmax) / 2u;
        if (t < 24) t = 24;
        if (t > 300) t = 300;
        s_thrCyc = s_cal ? ((s_thrCyc * 7 + t) / 8) : t;
        s_cal = 1;
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
// 退出临界区前必须清掉 GPIO 中断状态锁存。整帧的 1080 个上升沿都会把这个锁存位
// 重新置起来（它只看 GPIO_PINn_INT_ENA，不受 CPU 全局中断使能影响），所以
// portEXIT_CRITICAL_ISR 一开中断，CPU 立刻又进一次本 ISR —— 那时线已经是低的，
// 只会在里面空等 50µs 然后记一个「残片」再退出。
// v32.26 现场日志 残片 5358 ≈ LED 帧 5356（每帧正好多一次）就是这次幽灵中断，
// 它白烧 50µs CPU，还把网页日志的「本帧诊断」全刷成 0。
#define FC_IRQ_CLEAR()   REG_WRITE(GPIO_STATUS_W1TC_REG, FC_GPIO_BIT)

static void IRAM_ATTR fcRiseIsr(void* arg) {
    (void)arg;
    if (!s_active) return;      // 不需要飞控灯的场合（本地灯效）直接走开

    // 【核心】整个帧在临界区里采完：忙等期间哪怕只被打断一次，
    // 这个 bit 的宽度就是错的（WiFi/BLE 中断延迟可达数十 µs）
    portENTER_CRITICAL_ISR(&s_capMux);
    uint32_t t0 = esp_cpu_get_cycle_count();

    // 1) 等当前这个高电平结束。我们是上升沿之后几百 ns~几 µs 才进来的，
    //    位宽已经被截短，所以这个 bit 不要（丢几位由帧尾 24 对齐补回来）
    while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
        if ((esp_cpu_get_cycle_count() - t0) > FC_ENTRY_MAX_CYC) {
            g_rxDrop = (uint32_t)(g_rxDrop + 1);   // 线常高：没在发数据
            FC_IRQ_CLEAR();
            portEXIT_CRITICAL_ISR(&s_capMux);
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
    s_h0Sum = 0; s_h0N = 0; s_h1Sum = 0; s_h1N = 0;
    uint8_t shortHi = 0;
    g_rxDone = (uint32_t)(g_rxDone + 1);

    uint32_t w0 = 0, lo = 0, hi = 0;

    while (1) {
        // ---- 低电平段（bit 之间的间隔）----
        // v32.27：边沿时刻取「最后一次仍为低」与「第一次读到高」的中点，
        // 把轮询粒度（一拍≈12 周期≈75ns）带来的误差从「单向少算」变成「对称 ±半拍」。
        // 原来单向少算 100~150ns，0/1 两簇整体左移且不对齐，阈值中点跟着偏。
        w0 = esp_cpu_get_cycle_count();
        uint32_t tLowLast = w0;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) == 0) {
            tLowLast = esp_cpu_get_cycle_count();
            if (tLowLast - w0 > FC_GAP_CYC) goto cap_done;   // 帧尾 reset（同时也是死循环保险）
        }
        uint32_t tRise = (tLowLast + esp_cpu_get_cycle_count()) / 2u;

        // ---- 高电平段（决定这一 bit 是 0 还是 1）----
        uint32_t tHiLast = tRise;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
            tHiLast = esp_cpu_get_cycle_count();
            if (tHiLast - tRise > FC_GAP_CYC) goto cap_done; // 线被拉长 / 帧尾（同上是保险）
        }
        uint32_t tFall = (tHiLast + esp_cpu_get_cycle_count()) / 2u;

        hi = tFall - tRise;
        lo = tRise - w0;
        // 整帧硬上限（每 bit 判一次，不在内层轮询里加开销）：
        // 线一直被拉成有跳变但不成帧时，别在「无中断」状态里卡死
        if (tFall - t0 > FC_CAP_MAX_CYC) goto cap_done;

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
        // 按判定结果分簇累计（供帧尾算两簇均值中点当新阈值）
        if (one) { s_h1Sum += hi; s_h1N++; }
        else     { s_h0Sum += hi; s_h0N++; }

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
    FC_IRQ_CLEAR();          // 清掉整帧积累的上升沿锁存，避免退出后再空转一次
    portEXIT_CRITICAL_ISR(&s_capMux);
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
