#pragma once
// ============================================================
// 灯带输出（RMT TX）：把 RGB 缓冲编码为 WS2812 时序并发送
// ============================================================

#include <stdint.h>
#include "config.h"
#include "effects.h"

// 初始化 RMT TX（setup 中调用一次）
void ledOutInit();

// 把 leds 前 n 颗编码为 WS2812 符号并发送（GRB 顺序）
void showStrip(const RGB* leds, int n);
