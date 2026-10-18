#pragma once
// ============================================================
// 飞控 LED 信号输入（RMT RX）
//   - 接收 Betaflight LED strip 输出的 3.3V WS2812 时序
//   - 解码为 GRB 每颗灯的颜色
//   - 提供轮询接口：主循环每次调用 pollFcInput() 获取最新帧
// ============================================================

#include <stdint.h>
#include "driver/rmt_types.h"
#include "config.h"
#include "effects.h"

// 一帧飞控数据（解码后）
struct FcFrame {
    RGB leds[FC_MAX_LEDS];   // 解码出的灯色（GRB）
    int count;               // 本帧灯数
};

// 初始化 RMT RX（在 setup 中调用一次）
void fcInputInit();

// 轮询：若有完整新帧则解码并返回 true（out 写入解码结果）
bool pollFcInput(FcFrame* out);

// 诊断统计（主循环心跳打印用）
extern volatile uint32_t g_rxFrames;   // 累计成功解码的飞控帧数
extern volatile uint32_t g_rxLastLeds; // 最近一帧解出的灯珠数
