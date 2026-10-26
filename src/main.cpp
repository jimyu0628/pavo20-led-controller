// ============================================================
// 幻彩灯带控制板 · 主程序（ESP32-C3 SuperMini，ESP-IDF 框架）
// 复刻说明书控制板全部功能：
//   - 10 种模式：0=Betaflight(1:1/1:2/1:4) + 9 种灯效
//   - 无飞控独立使用：开机默认进入本地灯效，裸板调试上电即亮
//   - 飞控自动接管：GPIO4 检测到飞控 T6(LED_STRIP) 信号即自动显示飞控灯，
//     信号丢失 0.5s 后自动回到当前灯效（Betaflight 模式无信号则灭灯）
//   - 单键手势：单击=模式 / 双击=颜色 / 三击=速度 / 长按=亮度 / 长按3s=标定
//   - 灯珠数量标定：解码飞控红(×2)/蓝(×4)灯珠计算实际灯数，存 NVS flash
// 引脚：GPIO4=飞控信号输入(预留)  GPIO3=灯带数据  GPIO9=BOOT按键
// ============================================================
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"
#include "effects.h"
#include "fc_input.h"
#include "led_out.h"
#include "button.h"
#include "wifi_cfg.h"
#include "ble_uart.h"
#include "msp_tcp.h"      // v32.10：MSP over TCP（5761），走 WiFi 给 BF Configurator 用
#include "ota.h"
#include "device_name.h"  // v32.19：设备名（PavoLED + MAC 后缀），WiFi/BLE 共用
#include "esp_ota_ops.h"
#include "esp_system.h"   // esp_reset_reason / esp_restart
#include "esp_wifi.h"     // v32.13：AP 发射功率限制（弱供电下保热点）
// ---------- 时间工具 ----------
static uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }
static void delayMs(uint32_t ms) {
    // 关键：FreeRTOS tick 默认 100Hz（10ms/tick），pdMS_TO_TICKS(1) 会算出 0，
    // vTaskDelay(0) = yield（不睡眠）→ 主循环忙转占满 CPU → IDLE 饿死 → 任务看门狗超时！
    // 因此不足 1 tick 的延迟必须至少让出 1 tick（10ms），主循环才能真正睡眠。
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if (ticks == 0) ticks = 1;
    vTaskDelay(ticks);
}
// ---------- 日志 ----------
// 用 printf 走系统 console（USB-Serial-JTAG vfs）：
//  - 开机即可正常输出（裸 usb_serial_jtag_write_bytes 在驱动未初始化时输出会丢失，
//    导致 build 行与 [LED]/[RX] FAILED 错误码全看不到）
//  - 只用于启动/按键/标定等低频消息；经实测 printf 不会阻塞主任务
//    （此前任务看门狗超时真凶是 GPIO4 悬空导致的 RMT RX 中断风暴，已修复）
static void logMsg(const char* fmt, ...) {
    // v32.5：160 字节太小——心跳整行 180+ 字符会被 vsnprintf 截断，
    // 导致 [MSP]/[RX] 后半段永远打不出来（排查时看不到关键字段）
    char buf[448];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("%s", buf);
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
// ---------- 运行状态 ----------
static int s_mode = DEFAULT_MODE; // 0=Betaflight, 1~9=灯效；开机固定本地灯效
static int s_colorIdx = 0;    // 0~7（断电记忆）
// v32.21：速度、亮度从「档位索引」改成「无级百分比」——网页要能拖动到任意值。
// 按键手势仍按固定档循环（下表），只是它改的是同一个百分比变量，两套入口不打架。
static int s_speedPct  = 55;  // 1~100，越大越快（断电记忆）
static int s_brightPct = 50;  // 0~100，0=关灯（断电记忆）
static const uint8_t SPEED_STEPS[SPEED_COUNT]   = {85, 55, 15};       // 快/中/慢（三击循环）
static const uint8_t BRIGHT_STEPS[BRIGHT_COUNT] = {100, 75, 50, 25};  // 长按循环
static int s_calibCount = STRIP_DEFAULT_LEDS; // 标定灯数（实际灯带长度，断电记忆）
static bool s_calibrating = false;
static int s_calCount = STRIP_DEFAULT_LEDS;   // 标定过程中的计数
// v32.23：飞控侧 LED 字节顺序（0=GRB 1=RGB 2=BRG 3=RBG 4=GBR 5=BGR），断电记忆
static int s_fcOrder = 3;
// 灯带显示缓冲 + 实际显示长度
static RGB s_strip[STRIP_MAX_LEDS];
static int s_stripLen = STRIP_DEFAULT_LEDS;
// 飞控信号接管状态
// v32.22：s_fcActive 只可能在「飞控模式(s_mode==0)」且真的收到帧时为 true。
// 一旦切到灯效模式必须立刻清掉，否则灯效渲染（主循环第 4 步）会被永久跳过。
static bool s_fcActive = false;   // true=飞控正在接管显示（仅飞控模式）
static uint32_t s_lastFcMs = 0;   // 最后一次有效飞控帧时间
// 效果帧节奏
static uint32_t s_lastFxMs = 0;
static int s_fxPhase = 0;
// v32.21：速度 1~100 线性映射到帧间隔 200ms~20ms（原来只有 20/45/90 三档，
// 网页拖到 47 这种中间值就没处安放）。100=20ms（50 帧/秒，丝滑），1=200ms（肉眼可数的慢）。
static uint32_t fxIntervalMs() { return 20u + (uint32_t)(100 - s_speedPct) * 180u / 99u; }
// 亮度百分比 → 亮度系数（0~255）。0% 直接给 0，等于关灯。
static uint8_t brightScale() { return (uint8_t)(s_brightPct * 255 / 100); }
// ---------- 持久化 ----------
static void loadPrefs() {
    // 开机恢复上次保存的设置（v30 起模式也记忆）：
    // 颜色/速度/亮度/模式/标定灯数均断电记忆；
    // 若上次是飞控模式(0)且未接飞控，上电灭灯属正常，单击按键切到本地灯效即可
    uint8_t tmp = 0;
    uint8_t oi  = 0;
    s_mode       = nvsGetU8("mode", DEFAULT_MODE);
    s_colorIdx   = nvsGetU8("color", 0);
    // v32.21：新键 spct/bpct 优先；老固件只留下 speed/bright1 档位索引，做一次兼容换算，
    // 免得刷完新固件亮度和速度跳回默认值。
    if (nvs_get_u8(s_nvs, "spct", &tmp) == ESP_OK) {
        s_speedPct = tmp;
    } else {
        oi = nvsGetU8("speed", 1);
        s_speedPct = SPEED_STEPS[(oi < SPEED_COUNT) ? oi : 1];
    }
    if (nvs_get_u8(s_nvs, "bpct", &tmp) == ESP_OK) {
        s_brightPct = tmp;
    } else {
        oi = nvsGetU8("bright1", 2);
        s_brightPct = BRIGHT_STEPS[(oi < BRIGHT_COUNT) ? oi : 2];
    }
    s_calibCount = nvsGetU16("calib", STRIP_DEFAULT_LEDS);
    s_fcOrder    = nvsGetU8("ford", 3);   // v32.23：飞控字节顺序，默认 RBG
    if (s_fcOrder < 0 || s_fcOrder > 5) s_fcOrder = 3;
    fcSetRxOrder(s_fcOrder);
    if (s_mode < 0 || s_mode >= MODE_COUNT) s_mode = DEFAULT_MODE;
    if (s_colorIdx < 0 || s_colorIdx >= COLOR_COUNT) s_colorIdx = 0;
    if (s_speedPct  < 1)   s_speedPct  = 1;
    if (s_speedPct  > 100) s_speedPct  = 100;
    if (s_brightPct < 0)   s_brightPct = 0;
    if (s_brightPct > 100) s_brightPct = 100;
    if (s_calibCount < 1 || s_calibCount > CALIB_MAX) s_calibCount = STRIP_DEFAULT_LEDS;
    s_stripLen = (s_calibCount > STRIP_MAX_LEDS) ? STRIP_MAX_LEDS : s_calibCount;
}
static void savePrefs() {
    nvsPutU8("mode", (uint8_t)s_mode);
    nvsPutU8("color", (uint8_t)s_colorIdx);
    nvsPutU8("spct", (uint8_t)s_speedPct);
    nvsPutU8("bpct", (uint8_t)s_brightPct);
    nvsPutU16("calib", (uint16_t)s_calibCount);
    nvsPutU8("ford", (uint8_t)s_fcOrder);
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
// 飞控模式下收到帧即调用：按比例把飞控 LED 帧映射到灯带
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
// 在档位表里找「比当前值小的最大档」，没有（已是最低档）就绕回第一档 —— 循环切换
static int nextStep(const uint8_t* tab, int n, int cur) {
    for (int i = 0; i < n; i++) if (tab[i] < cur) return tab[i];
    return tab[0];
}
static void handleButton(ButtonEvent ev) {
    if (s_calibrating) {
        if (ev == BTN_CAL) exitCalibration();
        return; // 标定中忽略其他手势
    }
    switch (ev) {
        case BTN_CLICK:
            s_mode = (s_mode + 1) % MODE_COUNT;
            s_fxPhase = 0;
            savePrefs(); // v30：模式切换也断电记忆
            logMsg("[按键] 模式=%d\n", s_mode);
            break;
        case BTN_DOUBLE:
            s_colorIdx = (s_colorIdx + 1) % COLOR_COUNT;
            savePrefs();
            logMsg("[按键] 颜色=%d\n", s_colorIdx);
            break;
        case BTN_TRIPLE:
            s_speedPct = nextStep(SPEED_STEPS, SPEED_COUNT, s_speedPct);
            savePrefs();
            logMsg("[按键] 速度=%d\n", s_speedPct);
            break;
        case BTN_BRIGHT:
            s_brightPct = nextStep(BRIGHT_STEPS, BRIGHT_COUNT, s_brightPct);
            savePrefs();
            logMsg("[按键] 亮度=%d%%\n", s_brightPct);
            break;
        case BTN_CAL:
            enterCalibration();
            break;
        default:
            break;
    }
}
// ---------- Web 设置接口（wifi_cfg 模块调用） ----------
// 状态读取
int webGetMode()   { return s_mode; }
int webGetColor()  { return s_colorIdx; }
int webGetSpeed()  { return s_speedPct; }   // v32.21：1~100（原来是 0/1/2 档位）
int webGetBright() { return s_brightPct; }  // v32.21：0~100（原来是 0~3 档位）
int webGetCalib()  { return s_calibCount; }
int webGetFcOrder(){ return s_fcOrder; }      // v32.23：飞控字节顺序索引 0~5
bool webIsCalibrating() { return s_calibrating; }
// 状态写入（HTTP handler task 调用；int 赋值原子，无需锁）
void webSetMode(int v)   { if (v >= 0 && v < MODE_COUNT)  { s_mode = v; s_fxPhase = 0; savePrefs(); } }
void webSetColor(int v)  { if (v >= 0 && v < COLOR_COUNT) { s_colorIdx = v; savePrefs(); } }
void webSetSpeed(int v)  { if (v >= 1 && v <= 100) { s_speedPct = v; savePrefs(); } }
void webSetBright(int v) { if (v >= 0 && v <= 100) { s_brightPct = v; savePrefs(); } }
// v32.23：飞控字节顺序。改完立刻生效，下一帧飞控数据就按新顺序还原。
void webSetFcOrder(int v) {
    if (v < 0 || v > 5) return;
    s_fcOrder = v;
    fcSetRxOrder(v);
    savePrefs();
    logMsg("[RX] 飞控字节顺序 = %s\n", fcRxOrderName());
}

// 网页设置 LED 数量（等价于按键标定：只记总数，红蓝按比例自动分配）
void webSetCount(int v) {
    if (s_calibrating) return; // 标定中不干预
    if (v < 1) v = 1;
    if (v > CALIB_MAX) v = CALIB_MAX;
    if (v == s_calibCount) return;
    s_calibCount = v;
    s_stripLen = (v > STRIP_MAX_LEDS) ? STRIP_MAX_LEDS : v;
    savePrefs();
}
// 立即按当前设置刷新灯带（模式切换即时生效；标定中不干预）
void webApplyNow() {
    if (s_calibrating) return;
    if (s_mode == 0) {
        if (!s_fcActive) { // Betaflight 无飞控：灭灯
            for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
            refreshStrip();
        }
    } else {
        renderEffect(s_strip, s_stripLen, s_mode, COLORS[s_colorIdx], s_fxPhase);
        applyBrightness(s_strip, s_stripLen, brightScale());
        refreshStrip();
    }
}
// ---------- 主程序入口 ----------
extern "C" void app_main(void) {
    nvsInit();
    loadPrefs();
    // OTA：从备用分区启动后标记有效，防止下次重启自动回滚
    const esp_partition_t* rp = esp_ota_get_running_partition();
    if (rp) {
        esp_ota_img_states_t ost;
        if (esp_ota_get_state_partition(rp, &ost) == ESP_OK &&
            ost == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_ota_mark_app_valid_cancel_rollback();
            logMsg("[OTA] 新固件校验通过，已锁定\n");
        }
    }
    buttonInit();
    ledOutInit();
    fcInputInit();
    wifiCfgInit(); // AP 热点 RGBled + Web 设置（v32.14：有人在线不自动关，断开后 30 分钟无访问才关）
    // v32.13：把 WiFi 发射功率从默认 ~20dBm 降到 10dBm。
    // 现象「插上飞控串口线/LED 线热点就没了」最常见的原因是 5V 轨带不动 WiFi 发射的
    // 峰值电流（300mA+）——电压被拉塌后要么 brownout 复位、要么射频校准失败起不了 AP。
    // 10dBm 对 3~5 米手机调参完全够用，峰值电流大约减半。
    {
        esp_err_t tp = esp_wifi_set_max_tx_power(40); // 单位 0.25dBm → 40 = 10dBm
        int8_t got = 0;
        esp_wifi_get_max_tx_power(&got);
        printf("[WiFi] TX 功率：%d/4 = %d dBm（设置 %s）\n",
               (int)got, (int)got / 4, tp == ESP_OK ? "ok" : "fail");
    }
    bleUartInit(); // BLE NUS 透传（SpeedyBee App 蓝牙调参）
    mspTcpInit();  // v32.10：MSP over TCP 5761，PC 端 BF Configurator 走 WiFi 直连
    // 开机自检：v32.14 从「全白 255 亮 1s」改为「低亮度绿色 300ms」。
    // 20 颗 WS2812 全白 255 峰值约 1.2A，BEC 上会瞬时压降；绿色单通道 + 约 16% 占空
    // 峰值只剩 60mA 左右（约 1/20），既能肉眼看清灯带通电，又不会拉扯 5V 轨。
    for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB(0, 40, 20);
    refreshStrip();
    delayMs(300);
    for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
    refreshStrip();
    delayMs(120); // 再闪一下做个"心跳"，确认刷新正常（总耗电仍可忽略）
    for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB(0, 40, 20);
    refreshStrip();
    delayMs(120);
    for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
    refreshStrip();
    // 合并成单行输出：USB 串口在连接后才就绪，开机早期的多条 printf 容易丢失，
    // 单行 + \n 一次性输出，版本/灯数/模式全部可见
    // v32.19：设备名带 MAC 后缀（如 PavoLED-A3），WiFi 热点和 BLE 广播都用它，
    // 飞场里多台同款板子能分清。开机就打印出来，方便确认这台到底是哪个。
    logMsg("=== 幻彩灯带控制板 %s | 设备名 %s | 灯数=%d 比例=1:%d 模式=%d | 复位原因=%d ===\n",
           otaVersion(), DEV_NAME, s_calibCount, currentRatio(), s_mode, (int)esp_reset_reason());
    logMsg("build: %s %s | 颜色=%d 速度=%d 亮度=%d%% | LED init=%s err=0x%x\n",
           __DATE__, __TIME__, s_colorIdx, s_speedPct, s_brightPct,
           ledOutOk() ? "OK" : "FAIL", (unsigned)ledOutErr());
    // 开机信息存入 Web 调试区（USB 串口连上较慢错过开机日志时，网页可查看）
    char bi[512];
    snprintf(bi, sizeof(bi),
             "版本 %s | 设备名 %s | 灯数 %d | 比例 1:%d | 模式 %d | build %s %s | 颜色 %d 速度 %d 亮度 %d%% | LED %s | 复位原因 %d",
             otaVersion(), DEV_NAME, s_calibCount, currentRatio(), s_mode,
             __DATE__, __TIME__, s_colorIdx, s_speedPct, s_brightPct,
             ledOutOk() ? "OK" : "FAIL", (int)esp_reset_reason());
    wifiCfgSetBootInfo(bi);
    uint32_t lastBtnPoll = 0;
    uint32_t lastHbMs = 0;
    uint32_t lastRxLogMs = 0;
    uint32_t lastSeen = 0;
    uint32_t lastTxTotal = 0;
    uint32_t lastUrx = 0, lastUtx = 0; // MSP 透传字节（每秒差值）
    FcFrame frame;
    while (1) {
        uint32_t now = nowMs();
        // 0) 发送心跳：每秒打印 TX/RX 状态与统计（错误码每秒重复，防截断丢失）
        if (now - lastHbMs >= 1000) {
            lastHbMs = now;
            wifiCfgPoll(now); // 每秒：无线闲置满 60 秒就关 WiFi + 蓝牙（有人在用/有请求/蓝牙连着则续期）
            uint32_t cur = (uint32_t)g_txCount;
            const char* wst = wifiCfgState() == 0 ? "off" : (wifiCfgState() == 2 ? "keep" : "ap");
            uint32_t urx = bleUartRxCount(), utx = bleUartTxCount();
            logMsg("[LED] tx=%u/s ok=%d err=0x%x | [WiFi] %s %s %s | [BLE] %s | %s | [MSP] rx=%u/s tx=%u/s %s | [TCP] %s\n",
                   cur - lastTxTotal, ledOutOk() ? 1 : 0, (unsigned)ledOutErr(),
                   wst, wifiCfgStr(), wifiCfgDiag(), bleUartState(), otaVersion(),
                   urx - lastUrx, utx - lastUtx, bleUartDiag(), mspTcpDiag());
            // v32.24：飞控解码质量单独一行（每 5 秒），便于判断「跳色」是不是解码错帧
            if (now - lastRxLogMs >= 5000) {
                lastRxLogMs = now;
                char head[48];
                fcRxHeadHex(head, sizeof(head));
                logMsg("[RX] ok=%d err=0x%x frames=%u bad=%u skip=%u led=%u | hi=%u..%u lo=%u thr=%u ns | ord=%s head=%s\n",
                       fcRxOk() ? 1 : 0, (unsigned)fcRxErr(),
                       (uint32_t)g_rxFrames, (uint32_t)g_rxBad, (uint32_t)g_rxSkip,
                       (uint32_t)g_rxLastLeds,
                       (unsigned)g_rxHiMinNs, (unsigned)g_rxHiMaxNs,
                       (unsigned)g_rxLoMaxNs, (unsigned)g_rxThrNs,
                       fcRxOrderName(), head);
            }
            lastTxTotal = cur;
            lastUrx = urx; lastUtx = utx;
        }
        // 1) 按键（每 5ms）
        if (now - lastBtnPoll >= 5) {
            lastBtnPoll = now;
            ButtonEvent ev = pollButton();
            if (ev != BTN_NONE) handleButton(ev);
        }
        // 2) 飞控信号轮询
        //    v32.22 修复「进过飞控模式后切不回灯效」：
        //    原来这里是「任意模式只要收到帧就接管显示」，而飞控按 20Hz 持续推帧，
        //    于是 s_fcActive 永远为真、主循环第 4 步的灯效渲染被整条跳过 ——
        //    用户在网页/按键切到灯效模式，帧是切了，但灯一颗都不变（看着像不受控）。
        //    现在只有「飞控模式(0)」才把控制权交给飞控；标定仍需帧来数灯珠。
        // v32.24：飞控只要还在发帧就续上「存活」时间戳（哪怕那一帧因数据异常被
        // 整帧丢弃），否则连续几次坏帧会把「信号丢失」误判成真、把灯带灭掉。
        if ((uint32_t)g_rxSeen != lastSeen) {
            lastSeen = (uint32_t)g_rxSeen;
            s_lastFcMs = now;          // 标定期间也要，免得退出标定误判超时
        }
        if (pollFcInput(&frame)) {
            if (s_calibrating) {
                s_calCount = computeCalCount(frame.leds, frame.count);
                updateCalibDisplay();
            } else if (s_mode == 0) {
                mapFcToStrip(frame);
                s_fcActive = true;
            }
        }
        // 2.5) 不在飞控模式（或在标定）就别占着控制权：切走的那一刻立刻交还本地渲染，
        //      不必等 500ms 超时；顺手把帧计时回拨一拍，让灯效这一轮马上出图。
        if (s_fcActive && (s_mode != 0 || s_calibrating)) {
            s_fcActive = false;
            s_lastFxMs = now - fxIntervalMs();
        }
        // 3) 飞控信号丢失：超时后释放接管，恢复当前模式显示
        if (!s_calibrating && s_fcActive &&
            (now - s_lastFcMs) > FC_TIMEOUT_MS) {
            s_fcActive = false;
            if (s_mode == 0) { // Betaflight：无信号灭灯
                for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
                refreshStrip();
            } // 灯效模式：不灭灯，灯效渲染自然接管
        }
        // 4) 灯效模式渲染（非标定 && 无飞控接管 && 模式 1~9，按速度决定帧间隔）
        if (!s_calibrating && !s_fcActive && s_mode != 0 &&
            (now - s_lastFxMs) >= fxIntervalMs()) {
            s_lastFxMs = now;
            s_fxPhase++;
            if (s_fxPhase >= 1000000) s_fxPhase = 0; // 防 int 溢出
            renderEffect(s_strip, s_stripLen, s_mode, COLORS[s_colorIdx], s_fxPhase);
            applyBrightness(s_strip, s_stripLen, brightScale());
            refreshStrip();
        }
        delayMs(1);
    }
}
