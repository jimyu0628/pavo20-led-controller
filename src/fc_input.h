#pragma once
// ============================================================
// 飞控 LED 信号输入（GPIO 软件解码）
//   - 接收 Betaflight LED strip 输出的 3.3V WS2812 时序
//   - 解码为 GRB 每颗灯的颜色
//   - 提供轮询接口：主循环每次调用 pollFcInput() 获取最新帧
//   - 说明：ESP32-C3 RMT RX 每通道 64 符号上限，长帧续接不可靠，
//     故改用 GPIO 上升沿中断 + ISR 忙等逐 bit 测量（详见 fc_input.cpp）
// ============================================================

#include <stdint.h>
#include "config.h"
#include "effects.h"

// 一帧飞控数据（解码后）
struct FcFrame {
    RGB leds[FC_MAX_LEDS];   // 解码出的灯色（GRB）
    int count;               // 本帧灯数
};

// 初始化软件解码（在 setup 中调用一次）
void fcInputInit();

// 轮询：若有完整新帧则解码并返回 true（out 写入解码结果）
bool pollFcInput(FcFrame* out);

// 诊断统计（主循环心跳打印用）
extern volatile uint32_t g_rxFrames;   // 累计成功解码的飞控帧数
extern volatile uint32_t g_rxLastLeds; // 最近一帧解出的灯珠数
extern volatile uint32_t g_rxDone;     // 帧头（新帧）次数
extern volatile uint32_t g_rxSyms;     // 累计解码 bit 数
extern volatile uint32_t g_rxBitLike;  // 边沿间隔 ∈[40,140]cycle（真 bit 候选）
extern volatile uint32_t g_rxMid;      // 边沿间隔 ∈(140,256]cycle（边缘）
extern volatile uint32_t g_rxDrop;     // 边沿间隔 >256cycle（异常/噪声）
extern volatile uint32_t g_rxFrameEnd; // 主循环取帧次数
extern volatile uint32_t g_rxLedAtEnd; // 最近取帧的灯数
bool fcRxOk();    // 软件解码是否初始化成功
int  fcRxErr();   // 初始化失败错误码（0 = 无错误）
