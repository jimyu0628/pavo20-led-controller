#pragma once
// ============================================================
// 灯效引擎：RGB 结构 + HSV 转换 + 9 种灯效渲染
// ============================================================

#include <stdint.h>
#include "config.h"

// ---------- 颜色结构 ----------
struct RGB {
    uint8_t r, g, b;
    RGB() : r(0), g(0), b(0) {}
    RGB(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {}
    RGB& operator*=(float f) { r = (uint8_t)(r * f); g = (uint8_t)(g * f); b = (uint8_t)(b * f); return *this; }
};

// 8 色（说明书顺序：白红黄蓝绿青品红、关闭）
extern const RGB COLORS[COLOR_COUNT];

// 亮度档 100/75/50/25%
extern const uint8_t BRIGHT_SCALE[BRIGHT_COUNT];

// HSV → RGB（h: 0~359, s/v: 0~255）
void hsv2rgb(uint16_t h, uint8_t s, uint8_t v, RGB& out);

// 渲染灯效到 out（mode: 1~9，phase 为帧序号）
void renderEffect(RGB* out, int n, int mode, const RGB& color, int phase);

// 给整条灯带应用亮度系数
void applyBrightness(RGB* leds, int n, uint8_t scale);
