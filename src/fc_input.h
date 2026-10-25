#pragma once
// ============================================================
// 飞控 LED 信号输入（GPIO 软件解码）
//   - 接收 Betaflight LED strip 输出的 3.3V WS2812 时序
//   - 每颗灯 24 bit，按下面的「通道顺序」还原成 RGB
//   - 提供轮询接口：主循环每次调用 pollFcInput() 获取最新帧
//   - 说明：ESP32-C3 RMT RX 每通道 64 符号上限，长帧续接不可靠，
//     故改用 GPIO 上升沿中断 + ISR 忙等逐 bit 测量（详见 fc_input.cpp）
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

// 轮询：若有完整新帧则解码并返回 true（out 写入解码结果）
bool pollFcInput(FcFrame* out);

// ---- v32.23：飞控一侧的通道顺序 ----
// 板上 WS2812 常见的是 GRB，但飞控（Betaflight 的 ledstrip_grb_rgb 设置、
// 或外接的 LED 芯片批次）可能不同，顺序错了就会出现「红显示成绿、蓝显示成红」。
// 可选 0..5：0=GRB 1=RGB 2=BRG 3=RBG 4=GBR 5=BGR；断电记忆，网页 /api/set?ford=N 可改。
void fcSetRxOrder(int v);
int  fcRxOrder();
const char* fcRxOrderName();   // "GRB"/"RGB"/... 心跳日志用

// 诊断统计（主循环心跳打印用）
extern volatile uint32_t g_rxFrames;   // 累计成功解码的飞控帧数
extern volatile uint32_t g_rxLastLeds; // 最近一帧解出的灯珠数
extern volatile uint32_t g_rxDone;     // 帧头中断次数
extern volatile uint32_t g_rxSyms;     // 累计解码 bit 数
extern volatile uint32_t g_rxDrop;     // 帧头不可靠丢弃帧数
extern volatile uint32_t g_rxFrameEnd; // ISR 置"帧完成"次数
extern volatile uint32_t g_rxLedAtEnd; // 最近一次帧完成时的灯数
bool fcRxOk();    // 软件解码是否初始化成功
int  fcRxErr();   // 初始化失败错误码（0 = 无错误）
