#include "fc_input.h"
#include <string.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"   // v32.31：pollFcInput 里用 portENTER_CRITICAL（任务上下文）
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
// 原理：GPIO4 上升沿中断 → 屏蔽中断后忙等轮询电平，逐 bit 测高/低电平时长。
//   WS2812 800kHz：位周期 1250ns；0 码 高400/低850（高 < 低），1 码 高800/低450（高 > 低）
//
// ---- v32.29：从「绝对阈值」改成「同一位内 高 vs 低」+ 三类异常分类 ----
// v32.28 现场日志：`异常帧 0 · 残片 0 · 帧头丢 13bit` / `高位 262~1025ns` /
// `阈值 600ns` / `最长低位 1987ns`，用户仍报「灭灯区有弱光」「亮部光条里有杂色」。
// 三条结论：
//   ① 绝对阈值天生不靠谱：采集的测宽有系统性偏差（进/出轮询各少算几拍），
//      FC 那侧的高电平也未必是标准的 400/800。只要 0 码被抬高一点就越过阈值，
//      0 码立刻变成 1 —— 灭灯区发弱光。
//      → 改成判「这一位自己的高 与 低 谁长」：0 码恒 400<850、1 码恒 800>450。
//        两边共用同一个下降沿，测宽的**系统性偏差在比较中被抵消**；
//        FC 时钟整体偏快偏慢也不影响（占空比不变）。这才是 WS2812 解码的
//        标准判据，绝对阈值只留给「帧尾那一位」（它的低电平是帧间复位，不能用）。
//   ② 位周期是硬约束：一位的 (高+低) 一定约等于一个位周期。比半个周期还短
//      的跳变肯定是噪声振铃，直接丢，不再当成一位 —— 这就是「多出 13 bit
//      导致后面所有灯 24bit 分组整体错位 → 亮部杂色」的来源。
//   ③ 毛刺要分两种处理（旧版一律「累加到下一位」，那是错的）：
//      · 前一段低 <250ns → 一位被切开了，高电平要**并到上一位**（回退重判）
//      · 高 <200ns 且前后低都正常 → 边沿振铃，**整段丢弃**，不计位、不累加
// ============================================================

#define FC_GPIO_BIT       (1u << 4)   // GPIO4

// 160MHz 主频：1 cycle = 6.25ns
#define CYC_TO_NS(c)      ((uint32_t)(c) * 25u / 4u)

// v32.35【帧尾判定的时间常数，这条改小了 8 倍，是治「彩虹跑几分钟突然卡住又全灭」的关键】
//   老值 50µs（8000 cycle）。问题：中断只要判断不出「帧尾」，ISR 就得在那儿死等
//   50µs 才收工。现场日志 `残片 732220` —— 这七十多万次残片，每次都是 ISR
//   进来、抓到 1~23 个「位」、然后干等 50µs。732220 × 50µs ≈ 36 秒纯忙等，
//   而且这 36 秒里**全局中断是关着的**（主循环、WiFi、蓝牙全被按停）。
//   WS2812 位与位之间的低电平最长 850ns（这台飞控 925ns 周期时只有 700ns 左右），
//   所以「低电平超过 6.25µs」已经绝不可能是正常位间间隔 —— 要么帧尾，要么噪声。
//   6.25µs 足够把两种情况都识别出来，代价是忙等少 8 倍。
#define FC_TAIL_CYC       1000u
// 线被拉高（一直为高）的上限：25µs。真 1 码的高也只有 800ns 量级。
#define FC_HI_MAX_CYC     4000u
// 进中断时线还是高（上升沿之后 1~2µs 内进来的）：等它变低的上限 25µs → 7.5µs。
// 噪声尖峰拉高的时间极短，7.5µs 完全够；老值 25µs 等于每来一个噪声尖峰
// 就白等 25µs（WiFi 开着时 GPIO4 这根长线上一秒能有两三千个尖峰）。
#define FC_ENTRY_LOW_MAX_CYC  1200u
// 等「下一个上升沿」的上限：3µs。真数据位周期 1250ns，位间低最长 850ns ——
// 3µs 内一定会有下一个上升沿。等不到说明这次中断不是一帧的起点。
#define FC_ENTRY_RISE_CYC     480u
// 低电平短到物理上不可能（<150ns）。真 1 码的低通常 450ns，但有的飞控把
// T1H 拉到 1µs 时 T1L 只剩 250ns —— 阈值必须比它更小，否则每个 1 码都会被
// 当成「一位被切开」而合并掉，位数就少了、分组全错。150ns 只抓真正的毛刺。
#define FC_MIN_LOW_CYC        24u
// 高电平短到物理上不可能（<150ns，真 0 码短则 220ns）→ 边沿振铃，整段丢弃
#define FC_SPIKE_HI_CYC       24u
// 连续多少次「幽灵边沿、且一个真帧都没收到」就静默一下（防死机的最后一道闸）
// v32.35：300 次 / 静默 2 秒 → 150 次 / 静默 0.3 秒。
//   现场日志 `幽灵边沿 2812 · 静默 9` —— 9 × 300 = 2700，说明这道闸被噪声
//   触发了 9 次；每次静默 2 秒 = **灯带瞎 2 秒**（而且这期间画面是冻着的）。
//   0.3 秒足够让噪声风暴过去，肉眼几乎看不出来。
#define FC_GHOST_RUN_MAX      150u
#define FC_MUTE_CYC           48000000u   // 0.3s @160MHz
// 单帧捕获硬上限：5ms
#define FC_CAP_MAX_CYC    800000u
// 默认位周期：200 cycle = 1250ns（WS2812 标准）。之后按实测自适应
#define FC_PERIOD_NOM     200u

#define FC_MAX_BITS       (FC_MAX_LEDS * 24)

// ---------- 采集期间的中断屏蔽 ----------
static portMUX_TYPE s_capMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- ISR 与主循环共享 ----------
// 采集开关。v32.29 起它不只是「省性能」，更是**输出与输入的互斥锁**：
// ISR 最长要屏蔽全部中断 1.3ms，而本板 WS2812 输出（RMT）没有 DMA，
// 靠中断每 80µs 续填一次 FIFO。两者撞上就是：RMT 断流 → 灯珠收到残帧 →
// 前面几颗是新的、后面还是旧的 → 亮部光条里出现杂色；撞狠了通道直接卡死
// → 灯带定格/全灭，只能重启。所以主循环必须「发完一帧才开采集，收到一帧就关采集」。
static volatile bool     s_active = false;
// bit 数组：1 字节存 1 位（1536 位 = 3KB，双缓冲 6KB）。
// 不用位压缩：毛刺合并要「回退上一位重判」，按位打包回退很别扭。
static uint8_t           s_bits[2][FC_MAX_BITS];
static volatile uint16_t s_nbits[2];
static volatile uint8_t  s_wr = 0, s_pub = 0;
static volatile bool     s_frameReady = false;

// 判别阈值（cycle）：v32.29 起只用于「帧尾那一位」（它的低电平是帧间复位，
// 不能拿高低比）。仍然按两簇均值的中点自适应，日志里当健康指标看。
static volatile uint32_t s_thrCyc = 96;      // 600ns
static volatile uint8_t  s_cal = 0;
// 位周期估计（cycle），按接受位的 (高+低) 均值平滑跟随
static volatile uint32_t s_periodCyc = FC_PERIOD_NOM;

// 本帧统计
static volatile uint32_t s_fHiMin, s_fHiMax, s_fLoMax;
static volatile uint32_t s_h0Sum, s_h0N, s_h1Sum, s_h1N;
static volatile uint32_t s_pSum, s_pN;

// ---- 通道顺序（v32.23）----
// 每个条目是「输出 r/g/b 分别取自原始第几字节」。
static const uint8_t s_orderMap[6][3] = {
    {1, 0, 2}, // 0 GRB（WS2812 标准）
    {0, 1, 2}, // 1 RGB
    {2, 0, 1}, // 2 BRG
    {0, 2, 1}, // 3 RBG
    {1, 2, 0}, // 4 GBR
    {2, 1, 0}, // 5 BGR
};
static const char* s_orderName[6] = {"GRB", "RGB", "BRG", "RBG", "GBR", "BGR"};
// 默认 5(BGR)：这台飞控 + 这条灯带实测只有 BGR 颜色才对。
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
volatile uint32_t g_rxSyms = 0;      // 累计 bit 数（保留）
volatile uint32_t g_rxDrop = 0;      // 线常高直接放弃的次数
volatile uint32_t g_rxFrameEnd = 0;  // 收完整帧的总次数
volatile uint32_t g_rxLedAtEnd = 0;  // 最近一帧解出的灯数（未对齐补偿）
volatile uint32_t g_rxSeen = 0;      // ISR 判定「一帧结束」的总次数
volatile uint32_t g_rxSkip = 0;      // 保留字段
volatile uint32_t g_rxBad = 0;       // 掺了毛刺/振铃的帧数（信号质量指标）
volatile uint32_t g_rxClamp = 0;     // 保留（旧日志用）
volatile uint32_t g_rxFrag = 0;      // 不足 1 颗灯 / 超长的残片，丢弃次数
volatile uint32_t g_rxShort = 0;     // v32.35：残片细分 —— 抓到的位不足 24（噪声碎片）
volatile uint32_t g_rxOver = 0;      // v32.35：残片细分 —— 位数超过缓冲（跨帧粘在一起）
volatile uint32_t g_rxDropBits = 0;  // 最近一帧 cnt%24（正常恒为 23，见文件头）
volatile uint32_t g_rxGhost = 0;     // 「不是帧起点」的中断次数
volatile uint32_t g_rxMute = 0;      // 因幽灵边沿太多而静默采集的次数
volatile uint32_t g_rxSpike = 0;     // v32.29：被丢弃的振铃毛刺（累计）
volatile uint32_t g_rxMerge = 0;     // v32.29：被合并的「切开位」（累计）
volatile uint32_t g_rxNear = 0;      // v32.29：判为 1 但离阈值很近的位（弱光嫌疑）
volatile uint32_t g_rxMisalign = 0;  // v32.33：帧内有断流/毛刺/合并的帧（颜色可能错位）
volatile uint32_t g_rxBits = 0;      // v32.29：最近一帧的原始 bit 数（cnt）
volatile uint32_t g_rxGap = 0;       // v32.33：最近一帧的断流位数
volatile uint16_t g_rxGapMaxNs = 0;  // v32.33：最近一帧最长的一位时长
volatile uint16_t g_rxHiMinNs = 0;
volatile uint16_t g_rxHiMaxNs = 0;
volatile uint16_t g_rxLoMaxNs = 0;
volatile uint16_t g_rxThrNs = 0;
volatile uint16_t g_rxCapUs = 0;
volatile uint16_t g_rxPeriodNs = 0;  // v32.29：实测位周期（ns）

static esp_err_t s_rxErr = ESP_OK;
static bool s_rxSwInit = false;

bool fcRxOk() { return s_rxSwInit; }
int  fcRxErr() { return (int)s_rxErr; }

void fcRxSetActive(bool on) { s_active = on; }
bool fcRxActive() { return s_active; }

// ---------- 帧结束：对齐 → 发布 ----------
static inline void IRAM_ATTR fcEndFrame(uint16_t cnt, uint32_t t0,
                                        uint16_t nGap, uint32_t perMax, bool abnormal) {
    g_rxSeen = (uint32_t)(g_rxSeen + 1);
    uint32_t capUs = (uint32_t)((esp_cpu_get_cycle_count() - t0) * 25u / 4u / 1000u);

    // 残片（连一颗灯都没凑齐）不覆盖「本帧诊断」—— 帧间那些不是帧起点的中断
    // 会走到这里，把好帧的质量数据刷成 0。
    if (cnt < 24) {
        g_rxFrag = (uint32_t)(g_rxFrag + 1);
        g_rxShort = (uint32_t)(g_rxShort + 1);
        return;
    }
    if (cnt > FC_MAX_BITS) {          // 超长（>64 颗）：缓冲装不下，整帧丢
        g_rxFrag = (uint32_t)(g_rxFrag + 1);
        g_rxOver = (uint32_t)(g_rxOver + 1);
        return;
    }

    g_rxBits = (uint32_t)cnt;
    g_rxLedAtEnd = (uint32_t)(cnt / 24);
    g_rxDropBits = (uint32_t)(cnt % 24);
    // v32.33：帧头丢多少位（cnt%24）是**正常现象**，不是错误 —— 中断永远赶不上
    // 帧头那一位，尾部对齐后前面少几颗由 pollFcInput 复制补回。v32.32 之前
    // 把它当「错位帧」，现场 3407 帧报了 3393 帧，纯属吓自己。
    // v32.33：只有「帧中间出问题」的帧才可能真的分组错位（前半段整体偏移）。
    // 断流 / 丢弃振铃 / 合并切开位，三者任一发生就计一帧 —— 这个数才驱动判断。
    if (abnormal) g_rxMisalign = (uint32_t)(g_rxMisalign + 1);
    g_rxGap = (uint32_t)nGap;
    g_rxGapMaxNs = (perMax * 25u / 4u > 0xFFFFu) ? 0xFFFFu : (uint16_t)CYC_TO_NS(perMax);
    g_rxCapUs = (uint16_t)((capUs > 0xFFFFu) ? 0xFFFFu : capUs);
    g_rxHiMinNs = (s_fHiMin > 2000) ? 0 : (uint16_t)CYC_TO_NS(s_fHiMin);
    g_rxHiMaxNs = (uint16_t)CYC_TO_NS(s_fHiMax);
    g_rxLoMaxNs = (uint16_t)CYC_TO_NS(s_fLoMax);

    // 位周期：本帧所有「接受位」的 (高+低) 平均 → 缓慢跟随。
    // 它不是判据（判据是「高 vs 低」），只用来识别「比半个周期还短」的噪声跳变。
    if (s_pN >= 16) {
        uint32_t p = s_pSum / s_pN;
        if (p > 100 && p < 400) {                     // 625~2500ns 才算像话
            s_periodCyc = (s_periodCyc * 7 + p) / 8;
            g_rxPeriodNs = (uint16_t)CYC_TO_NS(s_periodCyc);
        }
    }

    // 判别阈值：两簇（0 码高位均值 / 1 码高位均值）的中点
    if (s_h0N >= 8 && s_h1N >= 8) {
        uint32_t a0 = s_h0Sum / s_h0N;
        uint32_t a1 = s_h1Sum / s_h1N;
        if (a1 > a0 + a0 / 3u) {
            uint32_t t = (a0 + a1) / 2u;
            if (t < 24) t = 24;
            if (t > 300) t = 300;
            s_thrCyc = s_cal ? ((s_thrCyc * 3 + t) / 4) : t;
            s_cal = 1;
        } else {
            // v32.33：两簇分不开 = 本帧判得很乱（阈值已经不可信）。
            // 直接作废校准，下一帧退回「高 vs 低」重新学一遍 —— 闭环自纠，
            // 免得一个偏掉的阈值把后续所有帧都带歪（全亮 / 全暗那种）。
            s_cal = 0;
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

// 进/出中断都必须清 GPIO 中断状态锁存：整帧的上千个上升沿都会把锁存位重新置起来
// （它只看 GPIO_PINn_INT_ENA，不受 CPU 全局中断使能影响）。任何一条返回路径留下
// 「已锁存未清除」，中断线就一直悬着 → CPU 反复重入 ISR → 主循环/WiFi 全饿死。
#define FC_IRQ_CLEAR()   REG_WRITE(GPIO_STATUS_W1TC_REG, FC_GPIO_BIT)

static volatile uint16_t s_ghostRun = 0;
static volatile uint32_t s_muteUntilCyc = 0;
// v32.34【现场真凶 —— 必须用独立标志，不能用 0 当哨兵】
//   旧写法：return (int32_t)(cycle - s_muteUntilCyc) < 0;
//   而 s_muteUntilCyc 平时是 0（上电初值 / fcRxReset 里也清 0）。
//   cycle 是 32 位、160MHz，2^31 拍 = **13.42 秒**：
//     开机 0~13.4s   cycle-0 正数 → 不静默（正常收帧）
//     13.4~26.8s     cycle-0 最高位为 1 → (int32_t) 变负数 → **一直静默**
//     之后每次计数器越过 2^31 都静默 13.4 秒
//   现场表现完全对上：跑十几秒灯全灭，过十几秒自己又亮；
//   日志里「飞控静默 13s」「窗口超时 36」（13.4s ÷ 400ms 开窗周期 ≈ 33 次）、
//   「自愈 5」（13s ÷ 2s ≈ 6 次）三个数全对得上。
//   现在只在真的置了静默时才比对时间，且到点自动清零。
static volatile bool s_muteOn = false;
static inline bool IRAM_ATTR fcMuted(void) {
    if (!s_muteOn) return false;
    if ((int32_t)(esp_cpu_get_cycle_count() - s_muteUntilCyc) < 0) return true;
    s_muteOn = false;          // 静默到期：清掉，别再算
    return false;
}

// ---------- GPIO 上升沿中断：一次中断收完整帧 ----------
static void IRAM_ATTR fcRiseIsr(void* arg) {
    (void)arg;
    FC_IRQ_CLEAR();             // ① 先清锁存
    if (!s_active) return;      // 输出优先：没开采集时立刻走开（不动中断、不占 CPU）
    if (fcMuted()) return;      // 风暴静默期

    portENTER_CRITICAL_ISR(&s_capMux);
    uint32_t t0 = esp_cpu_get_cycle_count();

    // ②-a：进中断时线一般还是高的（上升沿之后 1~2µs），等它变低
    uint32_t t = t0;
    while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
        if (esp_cpu_get_cycle_count() - t > FC_ENTRY_LOW_MAX_CYC) {
            g_rxDrop = (uint32_t)(g_rxDrop + 1);   // 线常高：飞控没在发
            portEXIT_CRITICAL_ISR(&s_capMux);
            return;
        }
    }
    // ②-b：等下一个上升沿 = 本帧第一个「能测准」的位起点。
    // 中断延迟有 1~2µs（不止一位），进中断那一位的宽度已经被截短，不要它。
    uint32_t tLowLast = esp_cpu_get_cycle_count();
    while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) == 0) {
        tLowLast = esp_cpu_get_cycle_count();
        if (tLowLast - t > FC_ENTRY_RISE_CYC) {
            g_rxGhost = (uint32_t)(g_rxGhost + 1);
            if (++s_ghostRun >= FC_GHOST_RUN_MAX) {
                s_ghostRun = 0;
                g_rxMute = (uint32_t)(g_rxMute + 1);
                s_muteUntilCyc = esp_cpu_get_cycle_count() + FC_MUTE_CYC;
                s_muteOn = true;   // v32.34：先定截止时间，再置标志
            }
            portEXIT_CRITICAL_ISR(&s_capMux);
            return;
        }
    }
    uint32_t tRise = (tLowLast + esp_cpu_get_cycle_count()) / 2u;

    // 复位本帧统计
    uint8_t* bp = s_bits[s_wr];
    uint16_t cnt = 0;
    s_fHiMin = 0xFFFFFFFFu;
    s_fHiMax = 0;
    s_fLoMax = 0;
    s_h0Sum = 0; s_h0N = 0; s_h1Sum = 0; s_h1N = 0;
    s_pSum = 0; s_pN = 0;
    uint16_t glitch = 0;
    uint16_t nSpike = 0, nMerge = 0, nNear = 0;
    uint16_t nGap = 0;          // v32.33：本帧断流位数
    uint32_t perMax = 0;        // v32.33：本帧最长的一位（暴露断流）
    uint32_t pendHi = 0;        // 跨「被切开」段累计的高电平（还没成形的一位）
    uint32_t pLocal = s_periodCyc;  // 本帧的位周期跟踪（只用最近几个位，跟得上变化）
    g_rxDone = (uint32_t)(g_rxDone + 1);

    for (;;) {
        // ---- 高电平段：边沿时刻取「最后一次仍为高」与「第一次读到低」的中点 ----
        uint32_t tHiLast = tRise;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) != 0) {
            tHiLast = esp_cpu_get_cycle_count();
            if (tHiLast - tRise > FC_HI_MAX_CYC) goto cap_done;  // 线被拉高：异常收尾
        }
        uint32_t tFall = (tHiLast + esp_cpu_get_cycle_count()) / 2u;
        uint32_t hi = tFall - tRise;
        if (tFall - t0 > FC_CAP_MAX_CYC) goto cap_done;       // 整帧硬上限

        // v32.35【噪声碎片就地掐掉】一帧的第一位，高电平必然 ≥ 220ns（0 码）。
        // 进来时如果第一位的高短于 150ns，这只可能是射频噪声尖峰（WiFi 开到
        // AP 模式后 GPIO4 这根长线上尖峰很多）。老代码会接着往下等「帧尾」，
        // 每个尖峰白等 50µs；现在立刻走人，代价不到 1µs。
        if (cnt == 0 && pendHi == 0 && hi < FC_SPIKE_HI_CYC) {
            // 不计入「毛刺/异常帧」—— 那是帧内质量指标，被噪声灌满就没意义了。
            // 它会计进「残片（短）」，日志里一眼能看出噪声有多吵。
            goto cap_done;
        }

        // ---- 低电平段：等到下一个上升沿；超过 FC_TAIL_CYC(6.25µs) 就是帧尾 ----
        bool tail = false;
        uint32_t tLowEnd = tFall;
        while ((REG_READ(GPIO_IN_REG) & FC_GPIO_BIT) == 0) {
            tLowEnd = esp_cpu_get_cycle_count();
            if (tLowEnd - tFall > FC_TAIL_CYC) { tail = true; break; }
        }
        uint32_t lo = tLowEnd - tFall;

        // ---- 低位短到物理上不可能（<250ns，真 1 码的低也有 450ns）----
        // 说明一位被毛刺切成了两段：先攒着高电平，等它真正结束再判。
        // 这里绝不能拿这个假的「低」去判高低 —— 那样每一位都会被判成 1。
        if (!tail && lo < FC_MIN_LOW_CYC) {
            pendHi += hi;
            nMerge++; glitch = 1;
            tRise = (tLowEnd + esp_cpu_get_cycle_count()) / 2u;
            continue;
        }

        uint32_t h = pendHi + hi;
        pendHi = 0;

        // ---- 这一位是噪声吗？----
        // 高 <150ns：边沿振铃，整段丢弃（真 0 码也在 220ns 以上）
        // (高+低) < 1/3 位周期：不是数据位
        bool noise = (h < FC_SPIKE_HI_CYC) ||
                     (!tail && (h + lo) * 3u < pLocal);
        if (noise) {
            nSpike++; glitch = 1;
        } else {
            // ---- 正常一位：判 0 / 1 ----
            // v32.33【判据改良 —— 治「紫色光条前后出现蓝红分离」】
            //   现场日志：位周期只有 925ns（不是 WS2812 标准的 1250），
            //   0 码高/低 = 225 / 700，1 码 = 643 / 282。
            //   「高 > 低」在位周期接近 1250 时余量很大，但到 925ns 时 0 码只差
            //   125ns —— 一有干扰就翻，翻一个位就是一颗灯的某个通道错值：
            //   紫色（R+B）会掉一个通道变成纯蓝 / 纯红，而且相邻灯珠成片出现，
            //   看起来就是「亮部前后蓝红分离」。日志里「临界位 219560」就是它。
            //
            //   改成两判据并用：
            //     · byThr ：高位 > 自适应阈值（本帧 0 簇均值 与 1 簇均值 的中点）。
            //               它是**统计值**，几百个位一起算出来的，单点抖动翻不动它，
            //               而且两簇均值都出自同一套测量，系统偏差自动抵消。
            //     · byDuty：高 > 低。标准 WS2812 判法，不需要任何先验，冷启动用它。
            //   两者一致直接采信；不一致时信 byThr，并记一次真正的「临界位」。
            bool one;
            if (tail) {
                // 帧尾这一位的低电平是帧间复位，不能用占空比，只能看阈值
                one = (h > s_thrCyc);
            } else {
                bool byDuty = (h > lo);
                bool byThr  = (h > s_thrCyc);
                if (!s_cal) {          // 还没校准：没有可信阈值，用占空比
                    one = byDuty;
                } else {
                    one = byThr;
                    if (byDuty != byThr) nNear++;   // 两判据打架 = 真临界位
                }
            }
            if (cnt < FC_MAX_BITS) bp[cnt] = (uint8_t)one;
            cnt++;
            if (h > s_fHiMax) s_fHiMax = h;
            if (h < s_fHiMin) s_fHiMin = h;
            if (lo > s_fLoMax) s_fLoMax = lo;
            if (one) { s_h1Sum += h; s_h1N++; }
            else     { s_h0Sum += h; s_h0N++; }
            if (!tail) {
                uint32_t per = h + lo;
                s_pSum += per; s_pN++;
                pLocal = (pLocal * 3u + per) / 4u;
                if (per > perMax) perMax = per;
                // v32.33：一位的时长超过两个位周期 = 帧中间断流（线被干扰瞬间断过），
                // 这之后的分组就整体偏了 —— 断流是「颜色错位」唯一可靠的信号。
                if (per > s_periodCyc * 2u) nGap++;
            }
        }

        if (tail) break;
        tRise = (tLowEnd + esp_cpu_get_cycle_count()) / 2u;
    }

cap_done:
    if (glitch) g_rxBad = (uint32_t)(g_rxBad + 1);
    g_rxSpike = (uint32_t)(g_rxSpike + nSpike);
    g_rxMerge = (uint32_t)(g_rxMerge + nMerge);
    g_rxNear  = (uint32_t)(g_rxNear + nNear);
    fcEndFrame(cnt, t0, nGap, perMax, (glitch != 0) || (nGap != 0));
    if (cnt >= 24) s_ghostRun = 0;
    FC_IRQ_CLEAR();
    portEXIT_CRITICAL_ISR(&s_capMux);
}

// ---------- 从 bit 数组取第 i 颗灯 ----------
static inline void fcGetLed(const uint8_t* base, uint16_t start, int i, uint8_t raw[3]) {
    uint32_t v = 0;
    int b0 = (int)start + i * 24;
    for (int b = 0; b < 24; b++) v = (v << 1) | (uint32_t)base[b0 + b];
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
    g_rxPeriodNs = (uint16_t)CYC_TO_NS(s_periodCyc);
    s_rxSwInit = true;
}

// ---------- 自愈：把采集状态清干净（收到帧卡死时调用） ----------
// v32.31【重要】这里原来是 gpio_isr_handler_remove() + gpio_isr_handler_add()，
// 而 gpio_isr_handler_add 在 handler 已存在时返回 ESP_ERR_INVALID_STATE ——
// 返回值根本没检查。一旦 remove 成功、add 失败，GPIO 中断就彻底没了，
// **只有重启 ESP32 才能恢复**，而且飞控那边重启完全没用 —— 现场
// 「灯全灭、BF 重启无效、ESP32 重启就好、有时几分钟自己又活」就是它。
// 另一个问题：remove/add 每次都要重新分配中断，ISR 延迟抖动 → 采到错帧。
//
// 现在不碰中断了：采集开关 s_active 只是一个标志，ISR 每进来先看它，
// GPIO 中断从头到尾一直是开着的，所以「重挂中断」这件事根本不需要做。
void fcRxReset() {
    FC_IRQ_CLEAR();              // 清掉可能悬着的锁存，保证下次上升沿能进来
    // v32.32：幂等补挂 —— handler 已经在时 gpio_isr_handler_add 返回
    // ESP_ERR_INVALID_STATE（无害，不破坏任何东西）；只有真的丢了才会装上。
    // v32.31 的问题是先 remove 再 add，add 失败就永久没中断；这里不 remove。
    gpio_isr_handler_add(GPIO_NUM_4, fcRiseIsr, NULL);
    // v32.33【关键兜底】gpio_isr_handler_add 在 handler 已存在时是**直接返回**
    // （ESP_ERR_INVALID_STATE），它**不会**重新打开 GPIO 的边沿中断使能位。
    // 也就是说：只要 INT_ENA 被谁清过一次（重配引脚、驱动内部操作、异常现场），
    // 上面这行 add 就是空转 —— 中断永远不来，飞控信号一帧也收不到，
    // 表现就是「灯全灭、BF 重启无效、只有 ESP32 重启才好」。
    // gpio_intr_disable/enable 直接写使能位，幂等、不碰 handler 注册表，
    // 每 2 秒兜一次，代价只有两条寄存器写。
    gpio_intr_disable(GPIO_NUM_4);
    gpio_intr_enable(GPIO_NUM_4);
    // v32.33【竞态】这里原来还要把 s_wr / s_pub / s_nbits 全清 0，可它每 2 秒
    // 就被主循环调用一次，而 ISR 可能正在写 s_bits[s_wr] —— 清了反而让
    // 「边采边写」更乱（表现为偶发整条灯带颜色跳一下）。缓冲交给双缓冲自己
    // 翻转就行，这里只清「静默 / 校准」这类纯状态量。
    s_ghostRun = 0;
    s_muteOn = false;            // v32.34：清静默要用标志，光清时间戳没用
    s_muteUntilCyc = 0;
    s_cal = 0;
    s_periodCyc = FC_PERIOD_NOM;
}

// ---------- 轮询：取完整帧 ----------
bool pollFcInput(FcFrame* out) {
#if FC_INPUT_ENABLE == 0
    (void)out;
    return false;
#endif
    if (!s_frameReady) return false;
    uint8_t p = s_pub;                       // 锁定缓冲：ISR 只会写另一个
    uint16_t cnt = s_nbits[p];
    s_frameReady = false;

    // 帧头丢掉的 bit 数 = cnt % 24，丢掉它们让分组终点正好落在帧尾。
    uint16_t start = (uint16_t)(cnt % 24);
    if (cnt < 24 || start >= cnt) return false;
    const uint8_t* base = s_bits[p];
    const uint8_t* m = s_orderMap[(s_rxOrder < 6) ? s_rxOrder : 0];
    int n = (int)((cnt - start) / 24);
    if (n > FC_MAX_LEDS) n = FC_MAX_LEDS;

    // v32.31【竞态】双缓冲只有 2 块：发布后 s_wr 会翻到我们正在读的这一块，
    // 下一次采集（1.3ms 内）就会边采边写，我们读到的就是半新半旧的乱码 ——
    // 表现为「颜色乱跳 / 偶尔全黑」。取帧期间短关中断（约 5~10µs，
    // 远小于 RMT 80µs 的续填周期），读完再放开。
    portENTER_CRITICAL(&s_capMux);
    for (int i = 0; i < n; i++) {
        uint8_t raw[3];
        fcGetLed(base, start, i, raw);
        out->leds[i].r = raw[m[0]];
        out->leds[i].g = raw[m[1]];
        out->leds[i].b = raw[m[2]];
    }
    portEXIT_CRITICAL(&s_capMux);

    // 帧头那一位必然采不到（中断进不来那么快），对齐后正好少一颗灯。
    // 把它补回来：向前复制一颗，这样灯带末端不会缺一段，飞控灯数也和实际一致。
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
