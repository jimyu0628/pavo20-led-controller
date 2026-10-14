#include "effects.h"
#include <math.h>

const RGB COLORS[COLOR_COUNT] = {
    {255, 255, 255}, // 0 白
    {255,   0,   0}, // 1 红
    {255, 255,   0}, // 2 黄
    {  0,   0, 255}, // 3 蓝
    {  0, 255,   0}, // 4 绿
    {  0, 255, 255}, // 5 青
    {255,   0, 255}, // 6 品红
    {  0,   0,   0}, // 7 关闭
};

const uint8_t BRIGHT_SCALE[BRIGHT_COUNT] = {255, 191, 128, 64}; // 100/75/50/25%

// ---------- HSV → RGB ----------
void hsv2rgb(uint16_t h, uint8_t s, uint8_t v, RGB& out) {
    h %= 360;
    uint8_t region = (uint8_t)(h / 60);
    uint8_t f = (uint8_t)(h % 60);
    uint8_t p = (uint8_t)(v * (255 - s) / 255);
    uint8_t q = (uint8_t)(v * (255 - s * f / 60) / 255);
    uint8_t t = (uint8_t)(v * (255 - s * (60 - f) / 60) / 255);
    switch (region) {
        case 0: out = {v, t, p}; break;
        case 1: out = {q, v, p}; break;
        case 2: out = {p, v, t}; break;
        case 3: out = {p, q, v}; break;
        case 4: out = {t, p, v}; break;
        default: out = {v, p, q}; break;
    }
}

// ---------- 亮度 ----------
void applyBrightness(RGB* leds, int n, uint8_t scale) {
    for (int i = 0; i < n; i++) {
        leds[i].r = (uint8_t)(leds[i].r * scale / 255);
        leds[i].g = (uint8_t)(leds[i].g * scale / 255);
        leds[i].b = (uint8_t)(leds[i].b * scale / 255);
    }
}

// ---------- 工具 ----------
static RGB scaled(const RGB& c, float f) {
    RGB r = c;
    r *= f;
    return r;
}

// ---------- 9 种灯效 ----------
// mode 1: 单色常亮
static void fxSolid(RGB* out, int n, const RGB& c, int phase) {
    for (int i = 0; i < n; i++) out[i] = c;
}

// mode 2: 单色呼吸
static void fxBreath(RGB* out, int n, const RGB& c, int phase) {
    float v = 0.5f + 0.5f * sinf(phase * 2.0f * 3.14159f / 100.0f);
    for (int i = 0; i < n; i++) out[i] = scaled(c, v);
}

// mode 3: 单色跑马（一条亮带单向跑）
static void fxMarquee(RGB* out, int n, const RGB& c, int phase) {
    int span = n / 5; if (span < 1) span = 1;
    int pos = phase % n;
    for (int i = 0; i < n; i++) {
        int d = (i - pos + n) % n;
        out[i] = (d < span) ? c : RGB();
    }
}

// mode 4: 双边跑马（两条亮带同向跑，间隔半圈）
static void fxMarquee2(RGB* out, int n, const RGB& c, int phase) {
    int span = n / 8; if (span < 1) span = 1;
    int pos = phase % n;
    int pos2 = (pos + n / 2) % n;
    for (int i = 0; i < n; i++) {
        int d = (i - pos + n) % n;
        int d2 = (i - pos2 + n) % n;
        out[i] = (d < span || d2 < span) ? c : RGB();
    }
}

// mode 5: 单色导向（带渐变尾巴的光点单向移动）
static void fxGuide(RGB* out, int n, const RGB& c, int phase) {
    int head = phase % n;
    for (int i = 0; i < n; i++) {
        int d = (head - i + n) % n;          // 距头部的距离
        if (d < n / 3) {
            float f = 1.0f - (float)d * 3.0f / (float)n;
            out[i] = scaled(c, f);
        } else {
            out[i] = RGB();
        }
    }
}

// mode 6: 单色增长（从起点逐颗点亮，循环）
static void fxGrow(RGB* out, int n, const RGB& c, int phase) {
    int k = phase % (n + 1);
    for (int i = 0; i < n; i++) out[i] = (i < k) ? c : RGB();
}

// mode 7: 彩虹呼吸（整条同色，色相流动 + 亮度呼吸）
static void fxRainbowBreath(RGB* out, int n, const RGB& c, int phase) {
    float v = 0.5f + 0.5f * sinf(phase * 2.0f * 3.14159f / 100.0f);
    uint16_t hue = (uint16_t)((phase * 3) % 360);
    RGB col;
    hsv2rgb(hue, 200, 255, col);
    for (int i = 0; i < n; i++) out[i] = scaled(col, 0.4f + 0.6f * v);
}

// mode 8: 彩虹渐变（沿灯带位置彩虹 + 整体流动）
static void fxRainbowFlow(RGB* out, int n, const RGB& c, int phase) {
    for (int i = 0; i < n; i++) {
        uint16_t hue = (uint16_t)(((long)i * 360 / n + phase * 2) % 360);
        hsv2rgb(hue, 220, 255, out[i]);
    }
}

// mode 9: 双边彩虹（从中间向两侧色相展开 + 流动）
static void fxRainbow2(RGB* out, int n, const RGB& c, int phase) {
    int mid = n / 2;
    for (int i = 0; i < n; i++) {
        int dist = (i < mid) ? (mid - i) : (i - mid);
        uint16_t hue = (uint16_t)(((long)dist * 720 / n + phase * 2) % 360);
        hsv2rgb(hue, 220, 255, out[i]);
    }
}

// ---------- 总入口 ----------
void renderEffect(RGB* out, int n, int mode, const RGB& color, int phase) {
    switch (mode) {
        case 1: fxSolid(out, n, color, phase); break;
        case 2: fxBreath(out, n, color, phase); break;
        case 3: fxMarquee(out, n, color, phase); break;
        case 4: fxMarquee2(out, n, color, phase); break;
        case 5: fxGuide(out, n, color, phase); break;
        case 6: fxGrow(out, n, color, phase); break;
        case 7: fxRainbowBreath(out, n, color, phase); break;
        case 8: fxRainbowFlow(out, n, color, phase); break;
        case 9: fxRainbow2(out, n, color, phase); break;
        default: for (int i = 0; i < n; i++) out[i] = RGB(); break;
    }
}
