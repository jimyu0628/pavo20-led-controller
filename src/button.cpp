#include "button.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "config.h"

static uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

static bool s_keyWasDown = false;
static uint32_t s_keyDownMs = 0;
static bool s_brightTriggered = false;
static bool s_calTriggered = false;
static uint32_t s_lastRepeatMs = 0;

static int s_clickCount = 0;        // 待定组合点击数
static uint32_t s_lastClickMs = 0;  // 最后一次点击时间

void buttonInit() {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << PIN_KEY);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

ButtonEvent pollButton() {
    bool down = (gpio_get_level((gpio_num_t)PIN_KEY) == 0);
    uint32_t now = nowMs();

    // ---- 按下边沿 ----
    if (down && !s_keyWasDown) {
        s_keyWasDown = true;
        s_keyDownMs = now;
        s_brightTriggered = false;
        s_calTriggered = false;
    }
    // ---- 按住期间：长按判定 ----
    else if (down && s_keyWasDown) {
        // 长按 3s → 标定（不打断亮度：3s 前已先触发一次亮度）
        if (!s_calTriggered && (now - s_keyDownMs) >= KEY_CAL_MS) {
            s_calTriggered = true;
            return BTN_CAL;
        }
        // 长按 0.8s → 亮度，之后每 0.5s 重复
        if (!s_brightTriggered && (now - s_keyDownMs) >= KEY_LONG_MS) {
            s_brightTriggered = true;
            s_lastRepeatMs = now;
            return BTN_BRIGHT;
        }
        if (s_brightTriggered && (now - s_lastRepeatMs) >= KEY_REPEAT_MS) {
            s_lastRepeatMs = now;
            return BTN_BRIGHT;
        }
    }
    // ---- 松开边沿：短按记一次点击 ----
    else if (!down && s_keyWasDown) {
        s_keyWasDown = false;
        uint32_t held = now - s_keyDownMs;
        if (held < KEY_PRESS_MS) {
            s_clickCount++;
            s_lastClickMs = now;
        }
    }

    // ---- 组合点击判定：距最后点击超过间隔且未按住 ----
    if (!s_keyWasDown && s_clickCount > 0 &&
        (now - s_lastClickMs) > KEY_DOUBLE_GAP_MS) {
        int cc = s_clickCount;
        s_clickCount = 0;
        if (cc == 1) return BTN_CLICK;
        if (cc == 2) return BTN_DOUBLE;
        return BTN_TRIPLE; // >=3
    }

    return BTN_NONE;
}
