#pragma once
// ============================================================
// 飞控 LED 信号输入（GPIO 软件解码）
//   - 接收 Betaflight LED strip 输出的 3.3V WS2812 时序
//   - 每颗灯 24 bit，按下面的「通道顺序」还原成 RGB
//   - 提供轮询接口：主循环每次调用 pollFcInput() 获取最新帧
//   - 说明：ESP32-C3 RMT RX 每通道 48 符号上限、无 DMA，长帧（45 颗 = 1080
//     符号）放不下，故改用 GPIO 上升沿中断 + ISR 忙等逐 bit 测宽。
//     v32.26：采集期间屏蔽全部中断（否则位宽全是错的），帧尾按 24 对齐分组，
//     取消整帧作废（详见 .cpp 头部注释）
// ============================================================

#include <stdint.h>
#include "config.h"
#include "effects.h"

// 一帧飞控数据（解码后）
struct FcFrame {
    RGB leds[FC_MAX_LEDS];   // 解码出的灯色（已按通道顺序还原成 R/G/B）
    int count;               // 本帧灯数
};

// 初始化软件解码（在 setup 中调用一次）
void fcInputInit();

// 轮询：若有新帧则返回 true 并写入 out。
// v32.26 起不再整帧丢弃（采集已可靠），只有「不足 1 颗灯」的残片会被丢掉
bool pollFcInput(FcFrame* out);

// v32.26：采集开关。只有 Betaflight 模式(0)或标定中才需要飞控帧 ——
// 因为采集期间要屏蔽全部中断约 1.4ms，本地灯效运行时没必要付这个代价。
// v32.29【重要】它同时是「输出/输入互斥」：本板 WS2812 输出（RMT）无 DMA，
// 靠中断每 80µs 续填 FIFO，与采集的 1.4ms 中断屏蔽撞上就会断流 →
// 灯珠收到残帧 → 亮部出现杂色，狠了通道卡死 → 灯带动不了。
// 主循环必须：发完一帧才 fcRxSetActive(true)，收到一帧立刻 fcRxSetActive(false)。
void fcRxSetActive(bool on);
bool fcRxActive();

// v32.29：把采集状态清干净（卡住不恢复时主循环会调它做自愈）
void fcRxReset();

// ---- v32.23：飞控一侧的通道顺序 ----
// 板上 WS2812 常见的是 GRB，但飞控（Betaflight 的 ledstrip_grb_rgb 设置、
// 或外接的 LED 芯片批次）可能不同，顺序错了就会出现「红显示成绿、蓝显示成红」。
// 可选 0..5：0=GRB 1=RGB 2=BRG 3=RBG 4=GBR 5=BGR；断电记忆，网页 /api/set?ford=N 可改。
void fcSetRxOrder(int v);
int  fcRxOrder();
const char* fcRxOrderName();   // "GRB"/"RGB"/... 心跳日志用

// v32.24：把最近一帧前 4 颗灯的字节打成十六进制（如 "FF0000 FF0000"），
// 用来对照飞控设置排查颜色问题。buf 至少 40 字节。
void fcRxHeadHex(char* buf, int buflen);

// 诊断统计（主循环心跳打印用）
extern volatile uint32_t g_rxFrames;   // 已上屏的帧数
extern volatile uint32_t g_rxLastLeds; // 最近一帧解出的灯珠数
extern volatile uint32_t g_rxDone;     // 进入 ISR 并开始采集的次数
extern volatile uint32_t g_rxSyms;     // 累计解码 bit 数
extern volatile uint32_t g_rxDrop;     // 线常高直接放弃的次数
extern volatile uint32_t g_rxFrameEnd; // 收完整帧的总次数
extern volatile uint32_t g_rxLedAtEnd; // 最近一次帧完成时的灯数
extern volatile uint32_t g_rxBad;      // 帧内出现「高电平短到不可能」的帧数（干扰指标）
extern volatile uint32_t g_rxSeen;     // 判帧结束总次数
extern volatile uint32_t g_rxSkip;     // 保留字段
extern volatile uint32_t g_rxClamp;    // 高电平被拉长、强制判 0 的 bit 数
extern volatile uint32_t g_rxFrag;     // 不足 1 颗灯的残片，丢弃次数
extern volatile uint32_t g_rxDropBits; // v32.26：最近一帧帧头丢掉的 bit 数（= cnt%24）
extern volatile uint32_t g_rxGhost;    // v32.28：不是帧起点的中断次数（幽灵边沿 / 线上噪声）
extern volatile uint32_t g_rxMute;     // v32.28：因幽灵边沿刷屏而静默采集的次数
extern volatile uint32_t g_rxSpike;    // v32.29：被丢弃的振铃毛刺（累计）
extern volatile uint32_t g_rxMerge;    // v32.29：被合并的「切开位」（累计）
extern volatile uint32_t g_rxNear;     // v32.29：高/低比值接近 1 的「判得勉强」的位
extern volatile uint32_t g_rxMisalign; // v32.29：bit 数不是 24N-1 的帧数（分组错位，杂色来源）
extern volatile uint32_t g_rxBits;     // v32.29：最近一帧的原始 bit 数
extern volatile uint16_t g_rxPeriodNs; // v32.29：实测位周期（ns）
extern volatile uint16_t g_rxHiMinNs;  // 最短高位（ns）
extern volatile uint16_t g_rxHiMaxNs;  // 最长高位（ns）
extern volatile uint16_t g_rxLoMaxNs;  // 最长低位（ns）
extern volatile uint16_t g_rxThrNs;    // 自适应判别阈值（ns）
extern volatile uint16_t g_rxCapUs;    // v32.26：最近一帧捕获耗时（= 中断屏蔽时长 µs）
bool fcRxOk();    // 解码是否初始化成功
int  fcRxErr();   // 初始化失败错误码（0 = 无错误）
