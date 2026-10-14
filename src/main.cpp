// ============================================================
// 幻彩灯带控制板 · 主程序（ESP32-C3 SuperMini，ESP-IDF 框架）
// 复刻说明书控制板全部功能：
//   - 10 种模式：0=Betaflight(1:1/1:2/1:4) + 9 种灯效
//   - 单键手势：单击=模式 / 双击=颜色 / 三击=速度 / 长按=亮度 / 长按3s=标定
//   - 灯珠数量标定：解码飞控红(×2)/蓝(×4)灯珠计算实际灯数，存 NVS flash
// 引脚：GPIO4=飞控信号输入  GPIO3=灯带数据  GPIO9=BOOT按键
// ============================================================

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_timer.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"
#include "effects.h"
#include "fc_input.h"
#include "led_out.h"
#include "button.h"

// ---------- 时间工具 ----------
static uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }
static void delayMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// ---------- 非阻塞日志 ----------
// 用 usb_serial_jtag_write_bytes 直接写 USB 串口，timeout=0：
// 主机没在读 / 发送缓冲满时立即返回丢弃数据，绝不阻塞 main 任务
// （默认 printf 走 vfs 会阻塞在 USB-Serial-JTAG 上，曾导致任务看门狗超时）
static void logMsg(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    usb_serial_jtag_write_bytes(buf, strlen(buf), 0);
}

// ---------- NVS 持久化（替代 Arduino Preferences） ----------
static nvs_handle_t s_nvs;

static void nvsInit() {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    nvs_open("pavo_led", NVS_READWRITE, &s_nvs);
}

static uint8_t nvsGetU8(const char* k, uint8_t def) {
    uint8_t v = 0;
    return nvs_get_u8(s_nvs, k, &v) == ESP_OK ? v : def;
}
static uint16_t nvsGetU16(const char* k, uint16_t def) {
    uint16_t v = 0;
    return nvs_get_u16(s_nvs, k, &v) == ESP_OK ? v : def;
}
static void nvsPutU8(const char* k, uint8_t v) {
    nvs_set_u8(s_nvs, k, v);
    nvs_commit(s_nvs);
}
static void nvsPutU16(const char* k, uint16_t v) {
    nvs_set_u16(s_nvs, k, v);
    nvs_commit(s_nvs);
}

// ---------- 运行状态（断电记忆） ----------
static int s_mode = 0;        // 0=Betaflight, 1~9=灯效
static int s_colorIdx = 0;    // 0~7
static int s_speedIdx = 1;    // 0快 1中 2慢
static int s_brightIdx = 0;   // 0~3
static int s_calibCount = STRIP_DEFAULT_LEDS; // 标定灯数（实际灯带长度）

static bool s_calibrating = false;
static int s_calCount = STRIP_DEFAULT_LEDS;   // 标定过程中的计数

// 灯带显示缓冲 + 实际显示长度
static RGB s_strip[STRIP_MAX_LEDS];
static int s_stripLen = STRIP_DEFAULT_LEDS;

// Betaflight 模式状态
static bool s_fcActive = false;
static uint32_t s_lastFcMs = 0;

// 效果帧节奏
static uint32_t s_lastFxMs = 0;
static int s_fxPhase = 0;
static const uint32_t FX_INTERVAL_MS[SPEED_COUNT] = {20, 45, 90};

// ---------- 持久化 ----------
static void loadPrefs() {
    s_mode       = nvsGetU8("mode", 0);
    s_colorIdx   = nvsGetU8("color", 0);
    s_speedIdx   = nvsGetU8("speed", 1);
    s_brightIdx  = nvsGetU8("bright", 0);
    s_calibCount = nvsGetU16("calib", STRIP_DEFAULT_LEDS);
    if (s_mode < 0 || s_mode >= MODE_COUNT) s_mode = 0;
    if (s_colorIdx < 0 || s_colorIdx >= COLOR_COUNT) s_colorIdx = 0;
    if (s_speedIdx < 0 || s_speedIdx >= SPEED_COUNT) s_speedIdx = 1;
    if (s_brightIdx < 0 || s_brightIdx >= BRIGHT_COUNT) s_brightIdx = 0;
    if (s_calibCount < 1 || s_calibCount > CALIB_MAX) s_calibCount = STRIP_DEFAULT_LEDS;
    s_stripLen = (s_calibCount > STRIP_MAX_LEDS) ? STRIP_MAX_LEDS : s_calibCount;
}

static void savePrefs() {
    nvsPutU8("mode", (uint8_t)s_mode);
    nvsPutU8("color", (uint8_t)s_colorIdx);
    nvsPutU8("speed", (uint8_t)s_speedIdx);
    nvsPutU8("bright", (uint8_t)s_brightIdx);
    nvsPutU16("calib", (uint16_t)s_calibCount);
}

// ---------- 显示 ----------
static void refreshStrip() {
    showStrip(s_strip, s_stripLen);
}

// ---------- Betaflight 模式：比例映射 ----------
static int currentRatio() {
    if (s_calibCount <= 32) return 1;
    if (s_calibCount <= 64) return 2;
    return 4;
}

static void mapFcToStrip(const FcFrame& frame) {
    int ratio = currentRatio();
    int fc = frame.count;
    for (int i = 0; i < s_stripLen; i++) {
        int src = i / ratio;
        s_strip[i] = (src < fc) ? frame.leds[src] : RGB();
    }
    refreshStrip();
}

// ---------- 标定模式 ----------
// 每颗灯：红色=实际 2 颗，蓝色=实际 4 颗，其他=1 颗
static int computeCalCount(const RGB* leds, int n) {
    int c = 0;
    for (int i = 0; i < n; i++) {
        bool isRed  = (leds[i].r > 100 && leds[i].g < 80 && leds[i].b < 80);
        bool isBlue = (leds[i].b > 100 && leds[i].r < 80 && leds[i].g < 80);
        if (isRed)       c += 2;
        else if (isBlue) c += 4;
        else             c += 1;
    }
    return c;
}

static void updateCalibDisplay() {
    int c = (s_calCount > CALIB_MAX) ? CALIB_MAX : s_calCount;
    for (int i = 0; i < s_stripLen; i++) {
        s_strip[i] = (i < c) ? RGB(255, 255, 255) : RGB(); // 前 c 颗白
    }
    refreshStrip();
}

static void enterCalibration() {
    s_calibrating = true;
    s_calCount = s_calibCount;
    updateCalibDisplay();
    logMsg("[标定] 已进入。在 Betaflight LED 页设置红/蓝灯珠（红=2颗 蓝=4颗），长按 3s 退出保存。\n");
}

static void exitCalibration() {
    s_calibrating = false;
    s_calibCount = (s_calCount < 1) ? 1 : (s_calCount > CALIB_MAX ? CALIB_MAX : s_calCount);
    s_stripLen = (s_calibCount > STRIP_MAX_LEDS) ? STRIP_MAX_LEDS : s_calibCount;
    savePrefs();
    logMsg("[标定] 退出。灯数=%d，比例=1:%d，Betaflight 侧应设 %d 颗。\n",
           s_calibCount, currentRatio(),
           (s_calibCount + currentRatio() - 1) / currentRatio());
    for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
    refreshStrip();
}

// ---------- 按键事件处理 ----------
static void handleButton(ButtonEvent ev) {
    if (s_calibrating) {
        if (ev == BTN_CAL) exitCalibration();
        return; // 标定中忽略其他手势
    }
    switch (ev) {
        case BTN_CLICK:
            s_mode = (s_mode + 1) % MODE_COUNT;
            s_fxPhase = 0;
            savePrefs();
            logMsg("[按键] 模式=%d\n", s_mode);
            break;
        case BTN_DOUBLE:
            s_colorIdx = (s_colorIdx + 1) % COLOR_COUNT;
            savePrefs();
            logMsg("[按键] 颜色=%d\n", s_colorIdx);
            break;
        case BTN_TRIPLE:
            s_speedIdx = (s_speedIdx + 1) % SPEED_COUNT;
            savePrefs();
            logMsg("[按键] 速度=%d\n", s_speedIdx);
            break;
        case BTN_BRIGHT:
            s_brightIdx = (s_brightIdx + 1) % BRIGHT_COUNT;
            savePrefs();
            logMsg("[按键] 亮度=%d%%\n",
                   (int)(BRIGHT_SCALE[s_brightIdx] * 100 / 255));
            break;
        case BTN_CAL:
            enterCalibration();
            break;
        default:
            break;
    }
}

// ---------- 主程序入口 ----------
extern "C" void app_main(void) {
    nvsInit();
    loadPrefs();

    buttonInit();
    ledOutInit();
    fcInputInit();

    // 开机自检：白灯闪 250ms
    for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB(255, 255, 255);
    refreshStrip();
    delayMs(250);
    for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
    refreshStrip();

    logMsg("=== 幻彩灯带控制板 (ESP32-C3 SuperMini) ===\n");
    logMsg("build: %s %s\n", __DATE__, __TIME__);
    logMsg("灯数=%d 比例=1:%d 模式=%d 颜色=%d 速度=%d 亮度档=%d\n",
           s_calibCount, currentRatio(), s_mode, s_colorIdx, s_speedIdx, s_brightIdx);

    uint32_t lastBtnPoll = 0;
    FcFrame frame;

    while (1) {
        uint32_t now = nowMs();

        // 1) 按键（每 5ms）
        if (now - lastBtnPoll >= 5) {
            lastBtnPoll = now;
            ButtonEvent ev = pollButton();
            if (ev != BTN_NONE) handleButton(ev);
        }

        // 2) 飞控信号轮询
        if (pollFcInput(&frame)) {
            if (s_calibrating) {
                s_calCount = computeCalCount(frame.leds, frame.count);
                updateCalibDisplay();
            } else if (s_mode == 0) {
                mapFcToStrip(frame);
                s_fcActive = true;
                s_lastFcMs = now;
            }
        }

        // 3) Betaflight 模式：信号超时熄灭
        if (!s_calibrating && s_mode == 0 && s_fcActive &&
            (now - s_lastFcMs) > FC_TIMEOUT_MS) {
            s_fcActive = false;
            for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
            refreshStrip();
        }

        // 4) 灯效模式渲染（按速度定帧间隔）
        if (!s_calibrating && s_mode != 0 &&
            (now - s_lastFxMs) >= FX_INTERVAL_MS[s_speedIdx]) {
            s_lastFxMs = now;
            s_fxPhase++;
            if (s_fxPhase >= 1000000) s_fxPhase = 0; // 防 int 溢出
            renderEffect(s_strip, s_stripLen, s_mode, COLORS[s_colorIdx], s_fxPhase);
            applyBrightness(s_strip, s_stripLen, BRIGHT_SCALE[s_brightIdx]);
            refreshStrip();
        }

        delayMs(1);
    }
}
