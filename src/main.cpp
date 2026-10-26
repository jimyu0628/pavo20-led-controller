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
#include "esp_attr.h"     // v32.33：RTC_NOINIT_ATTR（崩溃落点记录）
#include "esp_wifi.h"     // v32.13：AP 发射功率限制（弱供电下保热点）
#include "esp_task_wdt.h" // v32.29：主任务看门狗（卡死自动重启，不用拔电）

// ---------- v32.33：崩溃落点记录 ----------
// 现场「死机」只有两种：收不到飞控帧（灯全灭但 WiFi 还活着），或者整机重启
// （BOOT 行的复位原因会变）。前者靠 RX 统计看得见，后者一直是黑盒：
// 串口没接、网页日志随重启清空，根本不知道死在哪。ESP32 的 RTC 内存掉电才丢、
// 软重启保留，正好用来放「最后执行到哪一步 + 已经跑了多久」。
// 下次开机把上一次的值打出来，一眼就知道是 SPI 发送、飞控映射还是无线轮询。
#define STAGE_MAGIC   0x5A000000u
RTC_NOINIT_ATTR static uint32_t s_mark;    // 高 8 位 magic，低 8 位阶段号
RTC_NOINIT_ATTR static uint32_t s_markUp;  // 打标记那一刻的运行秒数
static uint32_t s_prevStage = 0;           // 上次（崩溃前）的阶段
static uint32_t s_prevUpSec = 0;           // 上次崩之前跑了多少秒
#define STAGE_UNKNOWN 0
#define STAGE_BOOT    1
#define STAGE_IDLE    2
#define STAGE_EFFECT  3
#define STAGE_FCMAP   4
#define STAGE_LEDOUT  5
#define STAGE_WIFI    6
#define STAGE_BUTTON  7
static inline void stageMark(uint32_t st) {
    s_mark = STAGE_MAGIC | (st & 0xFFu);
    s_markUp = (uint32_t)(esp_timer_get_time() / 1000000);
}
static const char* stageName(uint32_t st) {
    switch (st) {
        case STAGE_BOOT:   return "启动初始化";
        case STAGE_IDLE:   return "主循环空闲";
        case STAGE_EFFECT: return "灯效渲染";
        case STAGE_FCMAP:  return "飞控映射";
        case STAGE_LEDOUT: return "灯带输出";
        case STAGE_WIFI:   return "无线轮询";
        case STAGE_BUTTON: return "按键处理";
        default:           return "未知";
    }
}
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
// v32.27：出厂默认改成 5(BGR) —— 这台飞控 + 这条灯带实测只有 BGR 颜色才对
static int s_fcOrder = 5;
// v32.32：BLE 广播开关（0=关，默认）。C3 单射频，BLE 广播会打断灯带数据流 → 频闪；
// 只有要用手机 App 蓝牙调参时才需要开。断电记忆。
static int s_bleAdv = 0;
// 灯带显示缓冲 + 实际显示长度
static RGB s_strip[STRIP_MAX_LEDS];
static int s_stripLen = STRIP_DEFAULT_LEDS;
// 飞控信号接管状态
// v32.22：s_fcActive 只可能在「飞控模式(s_mode==0)」且真的收到帧时为 true。
// 一旦切到灯效模式必须立刻清掉，否则灯效渲染（主循环第 4 步）会被永久跳过。
static bool s_fcActive = false;   // true=飞控正在接管显示（仅飞控模式）
static bool s_fcBlanked = false;  // v32.35：已因信号丢失灭过灯（避免每秒重复清屏）
static uint32_t s_lastFcMs = 0;   // 最后一次有效飞控帧时间（只认「已解出并上屏」的帧）
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
    s_fcOrder    = nvsGetU8("ford", 5);   // v32.27：飞控字节顺序，默认 BGR
    if (s_fcOrder < 0 || s_fcOrder > 5) s_fcOrder = 5;
    s_bleAdv     = nvsGetU8("bleen", 0);  // v32.32：BLE 广播开关，默认关
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
    nvsPutU8("bleen", (uint8_t)s_bleAdv);
}
// ---------- 显示 ----------
static void refreshStrip() {
    stageMark(STAGE_LEDOUT);   // v32.33：万一崩在 SPI 发送里，重启后能看见
    showStrip(s_strip, s_stripLen);
    stageMark(STAGE_IDLE);
}
// ---------- Betaflight 模式：比例映射 ----------
static int currentRatio() {
    if (s_calibCount <= 32) return 1;
    if (s_calibCount <= 64) return 2;
    return 4;
}
// 飞控模式下收到帧即调用：按比例把飞控 LED 帧映射到灯带
// 飞控实际灯数：取历史最大值。ISR 进入时帧头那一位必然采不到（中断延迟），
// 所以单帧解出的灯数偶尔会比真实值少 1~2 颗；用峰值当基准，缺的尾部沿用
// 最后一颗的颜色，避免灯带末端出现一小段黑。
static int s_fcPeak = 0;
static uint8_t s_peakLowRun = 0;   // v32.30：连续「本帧比峰值少」的帧数
// v32.35：一帧解出的灯数最多比基准多多少，多出来就按「解错了」处理。
//   现场「彩虹跑几分钟突然卡住」有一半是这里：偶发一帧解出 64 颗（把噪声当成位），
//   峰值被抬到 64，之后每帧 off=64-42=22 → 前 22 颗全部「保持上一帧」不动，
//   只有末端两三颗在变 —— 肉眼就是「画面停住了」。
#define FC_PEAK_JUMP_MAX  8
static uint8_t s_peakJumpRun = 0;  // v32.35：连续「比基准多太多」的帧数
static void mapFcToStrip(const FcFrame& frame) {
    int ratio = currentRatio();
    int have = frame.count;
    if (have <= 0) return;
    // v32.35：离谱的帧直接丢，绝不让它污染基准（丢一帧看不出来，抬错基准会卡住画面）。
    // 但要留一条活路：用户真的在 BF 里把 LED 数量改大时，会连续很多帧都「偏大」，
    // 连续 30 帧（≈0.65 秒）都是同一个偏大值，就认账、把基准抬上去。
    if (s_fcPeak > 0 && have > s_fcPeak + FC_PEAK_JUMP_MAX) {
        if (++s_peakJumpRun < 30) return;
        s_fcPeak = have;
        s_peakJumpRun = 0;
        s_peakLowRun = 0;
    } else {
        s_peakJumpRun = 0;
    }
    if (have > FC_MAX_LEDS) return;
    // v32.30：峰值只用来补「偶尔少 1~2 颗」（中断延迟导致帧头丢位），不能只增不减。
    // 否则一次解码错乱（解出 42 颗）就把基准永久抬高，飞控侧把灯数改小之后
    // 本板永远跟不上 —— 末端该灭的灯会一直亮着。连续 20 帧（≈1 秒）都少就跟下来。
    if (have > s_fcPeak) {
        s_fcPeak = have;
        s_peakLowRun = 0;
    } else if (have < s_fcPeak) {
        if (++s_peakLowRun >= 20) { s_fcPeak = have; s_peakLowRun = 0; }
    } else {
        s_peakLowRun = 0;
    }
    int fc = s_fcPeak;
    // v32.34【帧头对齐 —— 治「红色纯色时各灯明暗跳动」和扫描跳格】
    //   中断进不来那么快，一帧开头必然少采几颗（45 颗的帧现场只解出 42~43 颗）。
    //   少掉的是**帧头**，帧尾永远对齐。所以本帧第 j 颗其实是飞控的第 (off+j) 颗，
    //   而旧代码直接把它当成第 j 颗 —— 每帧「少几颗」不一样，整条图案就左右平移
    //   1~3 颗，肉眼看着就是抖动/跳格；比例 1:2 时平移还会被放大成明暗跳变。
    int off = s_fcPeak - have;
    if (off < 0) off = 0;
    // v32.35：偏移太离谱 = 这帧不是正常帧尾对齐，宁可扔掉也不要把整条画面冻住
    if (off > FC_PEAK_JUMP_MAX * 2) return;
    for (int i = 0; i < s_stripLen; i++) {
        int src = i / ratio;
        if (src >= fc) { s_strip[i] = RGB(); continue; }  // 超出飞控灯数：灭
        int j = src - off;                 // 换算到本帧的下标
        if (j < 0) continue;               // 帧头丢掉的几颗：保持上一帧，别闪一下黑
        if (j >= have) continue;
        RGB c = frame.leds[j];
        // v32.31 暗切：飞控在「不亮」的区域并不是给 0，而是给一个极小的值 ——
        // 现场实测 BF 亮度 100 时暗区就是 0x08（8/255 = 3%），亮度 11 时是 0x01。
        // 本板再乘 42% 亮度后落在 1/255 附近，暗环境里就是那层「弱光」。
        // v32.33【保色相】原来逐通道切会把颜色渐变的尾巴切歪：紫 = R+B，扫描条纹
        // 边缘 R=15 / B=8 时 B 被切掉、R 留着 → 冒出一段纯红；另一侧正好反过来
        // → 冒出一段纯蓝。用户看到的「亮部前后蓝红分离」有一半是它造成的
        // （另一半是真临界位判错，已在 fc_input 里改判据）。
        // 改成整颗判定：三通道最大值都不超过门限才整颗归零，色相永远不动。
        uint8_t mx = c.r;
        if (c.g > mx) mx = c.g;
        if (c.b > mx) mx = c.b;
        if (mx <= FC_DARK_CUT) c = RGB();
        s_strip[i] = c;
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
// v32.30：飞控侧「应该发多少颗」。网页日志里跟「末帧 N 颗」并排显示，
// 一眼就能看出飞控配置和本板是否对得上 —— 用户长期被「末帧 42 颗」误导，
// 以为本板读到了飞控设置，其实它只是「本帧解出多少位 ÷ 24」。
int webGetRatio()  { return currentRatio(); }
int webGetFcNeed() { int r = currentRatio(); return (s_calibCount + r - 1) / r; }
int webGetFcOrder(){ return s_fcOrder; }      // v32.23：飞控字节顺序索引 0~5
int webGetBleAdv() { return s_bleAdv; }       // v32.32：BLE 广播开关 0/1
bool webIsCalibrating() { return s_calibrating; }
// ---------- v32.29：运行健康诊断（网页日志要看到，不然只能靠猜） ----------
// 这三项专治「跑一段时间灯全灭、日志也不动」这类现场说不清的问题：
//   txr 每秒钟真正推给灯带的帧数 —— 0 就是主循环没在跑或 RMT 废了
//   txf RMT 发送失败次数 —— 涨过就说明输出被打断过（杂色/定格）
//   lpm 最近一秒主循环最长一次「卡了多久」 —— 大了就是主循环被阻塞
//   sil 距上一帧飞控信号多少秒 —— 涨说明飞控那边不发了 / 我们收不到了
static volatile uint32_t s_lpMaxMs = 0;   // 主循环最大间隔（ms）
static uint32_t s_txRate = 0;             // 每秒发送帧数
static uint32_t s_fcSilentS = 0;          // 距上一帧飞控信号的秒数
static uint32_t s_rxResetN = 0;           // 采集自愈次数
static uint32_t s_autoRstN = 0;           // v32.33：本次开机已自动重启次数（上限 3）
// v32.32：采集窗管理（旧版窗口可能一直开着，ISR 反复屏蔽中断把整机拖慢）
static uint32_t s_capWinMs = 0;           // 本次开窗时刻（0=窗口没开）
static uint32_t s_capRetryMs = 0;         // 无帧退避截止时刻
static uint32_t s_capWinDrop = 0;         // 窗口超时关闭次数（涨=收不到帧）
int webGetTxRate()  { return (int)s_txRate; }
int webGetTxFail()  { return (int)g_txFail; }
int webGetLoopGap() { return (int)s_lpMaxMs; }
int webGetFcSilent(){ return (int)s_fcSilentS; }
int webGetRxReset() { return (int)s_rxResetN; }
int webGetCapDrop() { return (int)s_capWinDrop; }
// v32.33：上一次（重启前）执行到哪一步、跑了多久。0 = 没有记录（首次上电/断电）。
int webGetLastStage() { return (int)s_prevStage; }
int webGetLastUp()    { return (int)s_prevUpSec; }
const char* webStageName(int st) { return stageName((uint32_t)st); }
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
// v32.32：BLE 广播开关。开着才能用手机 App 蓝牙调参；关着干扰最小（推荐默认关）。
void webSetBleAdv(int v) {
    int on = v ? 1 : 0;
    if (on == s_bleAdv && bleUartAdvAllowed() == (on != 0)) return;
    s_bleAdv = on;
    bleUartSetAdv(on != 0);
    savePrefs();
    logMsg("[BLE] 广播开关 = %s\n", on ? "开（手机可搜到）" : "关（不影响灯带与 WiFi 调参）");
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
    // v32.33：先把上一次的落点读出来（RTC 内存，软重启保留），再覆盖它
    if ((s_mark & 0xFF000000u) == STAGE_MAGIC) {
        s_prevStage = s_mark & 0xFFu;
        s_prevUpSec = s_markUp;
    }
    stageMark(STAGE_BOOT);
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
    // v32.32：按上电记忆决定是否广播（默认关）。协议栈本身常驻，网页随时可开关。
    bleUartSetAdv(s_bleAdv != 0);
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
             "版本 %s | 设备名 %s | 灯数 %d | 比例 1:%d | 模式 %d | 飞控色序 %s | build %s %s | 颜色 %d 速度 %d 亮度 %d%% | LED %s | 复位原因 %d | 上次落点 %s · 跑了 %us",
             otaVersion(), DEV_NAME, s_calibCount, currentRatio(), s_mode, fcRxOrderName(),
             __DATE__, __TIME__, s_colorIdx, s_speedPct, s_brightPct,
             ledOutOk() ? "OK" : "FAIL", (int)esp_reset_reason(),
             s_prevStage ? stageName(s_prevStage) : "无记录", (unsigned)s_prevUpSec);
    wifiCfgSetBootInfo(bi);
    uint32_t lastBtnPoll = 0;
    uint32_t lastHbMs = 0;
    uint32_t lastRxLogMs = 0;
    uint32_t lastFrames = 0;   // v32.35：上一次已发布帧计数（判「飞控还在不在」）
    uint32_t lastFragN = 0, lastFrameN = 0;   // v32.35：碎片风暴检测
    uint32_t lastTxTotal = 0;
    uint32_t lastUrx = 0, lastUtx = 0; // MSP 透传字节（每秒差值）
    uint32_t lastLoopMs = nowMs();
    uint32_t fcLostLogMs = 0;
    FcFrame frame;
    // v32.29：主任务挂到任务看门狗上。往后任何一处卡死（RMT、printf、NVS…）
    // 8 秒后自动重启 —— 现场「灯全灭、只能拔电」从此不会再有。
    esp_task_wdt_add(NULL);
    esp_task_wdt_reset();
    while (1) {
        uint32_t now = nowMs();
        esp_task_wdt_reset();
        stageMark(STAGE_IDLE);
        // 主循环间隔：正常 = 一次 delayMs(1) ≈ 10ms（tick 100Hz）。
        // 明显大于 100ms 就是被谁阻塞了，记下来给网页看。
        {
            uint32_t d = now - lastLoopMs;
            lastLoopMs = now;
            if (d > s_lpMaxMs) s_lpMaxMs = d;
        }
        // 0) 发送心跳：每秒打印 TX/RX 状态与统计（错误码每秒重复，防截断丢失）
        if (now - lastHbMs >= 1000) {
            lastHbMs = now;
            wifiCfgPoll(now); // 每秒：无线闲置满 60 秒就关 WiFi + 蓝牙（有人在用/有请求/蓝牙连着则续期）
            stageMark(STAGE_IDLE);
            uint32_t cur = (uint32_t)g_txCount;
            const char* wst = wifiCfgState() == 0 ? "off" : (wifiCfgState() == 2 ? "keep" : "ap");
            uint32_t urx = bleUartRxCount(), utx = bleUartTxCount();
            s_txRate = (uint32_t)g_txCount - lastTxTotal;
            // v32.35：没收到过帧时不要把「开机多久」当成「飞控静默多久」
            s_fcSilentS = s_lastFcMs ? (now - s_lastFcMs) / 1000 : 0;
            logMsg("[LED] tx=%u/s fail=%u ok=%d err=0x%x | [WiFi] %s %s %s | [BLE] %s | %s | [MSP] rx=%u/s tx=%u/s %s | [TCP] %s\n",
                   s_txRate, (unsigned)g_txFail, ledOutOk() ? 1 : 0, (unsigned)ledOutErr(),
                   wst, wifiCfgStr(), wifiCfgDiag(), bleUartState(), otaVersion(),
                   urx - lastUrx, utx - lastUtx, bleUartDiag(), mspTcpDiag());
            // v32.29：主循环卡顿/采集自愈（有才打，正常时一行都不多）
            if (s_lpMaxMs > 100) {
                logMsg("[LOOP] 主循环最长间隔 %lums（正常约 10ms）｜采集自愈 %lu 次｜窗口超时 %lu｜飞控静默 %lus\n",
                       (unsigned long)s_lpMaxMs, (unsigned long)s_rxResetN,
                       (unsigned long)s_capWinDrop, (unsigned long)s_fcSilentS);
            }
            // v32.35【碎片风暴自我修复】
            //   现场日志 `残片 732220` = 七十多万次「ISR 进来了但一位有效数据都没收到」，
            //   一秒能到几千次，全是 WiFi 开到 AP 模式后 GPIO4 长线上拾取的射频尖峰。
            //   它本身不致命（v32.35 已把每次的代价从 50µs 压到 6µs），但一旦真的
            //   把采集状态弄乱（比如半帧粘连），光靠被动等待是回不来的。
            //   这里每秒算一次增量：1 秒内 >400 次无效采集、却一帧有效都没有 → 认定风暴，
            //   立刻强复位采集链路并关窗退避，不要等 3 秒/25 秒那两道慢保护。
            {
                uint32_t dfr = (uint32_t)g_rxFrag - lastFragN;
                uint32_t dfm = (uint32_t)g_rxFrames - lastFrameN;
                lastFragN = (uint32_t)g_rxFrag;
                lastFrameN = (uint32_t)g_rxFrames;
                if (s_mode == 0 && !s_calibrating && dfr > 400 && dfm == 0) {
                    fcRxReset();
                    fcRxSetActive(false);
                    s_capWinMs = 0;
                    s_capRetryMs = now + 120;
                    s_rxResetN++;
                    logMsg("[RX] 碎片风暴：1 秒 %u 次无效采集 / 0 帧有效 → 强复位采集链路\n",
                           (unsigned)dfr);
                }
            }
            s_lpMaxMs = 0;   // 每秒归零，网页看到的是「最近一秒」的最大卡顿
            // v32.24：飞控解码质量单独一行（每 5 秒），便于判断「跳色」是不是解码错帧
            if (now - lastRxLogMs >= 5000) {
                lastRxLogMs = now;
                char head[48];
                fcRxHeadHex(head, sizeof(head));
                logMsg("[RX] ok=%d err=0x%x frames=%u bits=%u mis=%u spike=%u merge=%u near=%u | hi=%u..%u lo=%u per=%u thr=%u ns | frag=%u ghost=%u cap=%uus ord=%s head=%s\n",
                       fcRxOk() ? 1 : 0, (unsigned)fcRxErr(),
                       (uint32_t)g_rxFrames, (uint32_t)g_rxBits,
                       (uint32_t)g_rxMisalign, (uint32_t)g_rxSpike,
                       (uint32_t)g_rxMerge, (uint32_t)g_rxNear,
                       (unsigned)g_rxHiMinNs, (unsigned)g_rxHiMaxNs,
                       (unsigned)g_rxLoMaxNs, (unsigned)g_rxPeriodNs,
                       (unsigned)g_rxThrNs,
                       (uint32_t)g_rxFrag, (uint32_t)g_rxGhost,
                       (unsigned)g_rxCapUs,
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
        //    v32.32：输出已经换成 SPI+DMA，DMA 搬运不依赖中断，采集那 1.3ms 的
        //    中断屏蔽不会再掐断灯带数据流（旧版 RMT 无 DMA，就是靠中断每 60µs
        //    续填 FIFO，一屏蔽就断流 → 频闪/残帧/通道卡死）。所以这里不再要求
        //    「发送彻底结束后才开窗」，只保留接口调用，时序结构不变。
        //    新增一条保护：采集窗不会无限开着 —— 开窗后 200ms 还没收到帧就关窗
        //    并退避 200ms，免得 ISR 一直占着中断把整机拖慢（这也是旧版「死机」的帮凶）。
        bool wantFc = (s_mode == 0 || s_calibrating);
        if (wantFc) {
            if (!fcRxActive() && (int32_t)(now - s_capRetryMs) >= 0) {
                ledOutWaitIdle();
                fcRxSetActive(true);
                s_capWinMs = now;
                if (s_capWinMs == 0) s_capWinMs = 1;
            }
        } else if (fcRxActive()) {
            fcRxSetActive(false);
            s_capWinMs = 0;
        }
        if (fcRxActive() && s_capWinMs && (now - s_capWinMs) > 200) {
            fcRxSetActive(false);               // 200ms 没帧：先还给系统
            s_capWinMs = 0;
            s_capRetryMs = now + 200;           // 退避 200ms 再开窗
            s_capWinDrop++;
        }
        // v32.35【关键修正】「飞控还在不在」只能看**已经解出来的帧**（g_rxFrames），
        // 不能看 ISR 进了多少次（g_rxSeen）—— 进 ISR 不代表解出帧，噪声碎片、
        // 幽灵边沿都会让它涨。现场日志 `残片 732220 · 飞控静默 0s` 就是这个错：
        // 噪声风暴里帧早就没了，可 s_lastFcMs 每秒被刷新几十上百次，于是
        // 「1 秒超时灭灯」「3 秒自愈」「25 秒自动重启」三条保护**全部失效**，
        // 画面就永远停在最后一帧上不动（用户看到的就是「彩虹突然卡住」）。
        if ((uint32_t)g_rxFrames != lastFrames) {
            lastFrames = (uint32_t)g_rxFrames;
            s_lastFcMs = now;
        }
        if (pollFcInput(&frame)) {
            fcRxSetActive(false);      // 收到就把中断还回去：射频/按键都要它
            s_capWinMs = 0;
            s_fcBlanked = false;       // v32.35：帧回来了，允许下次断线时重新灭灯
            if (s_calibrating) {
                s_calCount = computeCalCount(frame.leds, frame.count);
                updateCalibDisplay();
            } else if (s_mode == 0) {
                stageMark(STAGE_FCMAP);
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
        //    v32.29：光灭灯不够，还要**自愈**。现象「跑一段时间收不到飞控、重启就好」
        //    说明采集链路（中断/状态机）偶尔会卡住。现在连续 3 秒没帧就重挂一次
        //    GPIO 中断并把采集状态清干净，不用重启也能回来。
        //    v32.35：超时后**先保持最后一帧**，别急着灭灯 —— 现场「彩虹突然卡住
        //    然后全灭」里的「全灭」就是旧代码在 1 秒时直接清屏造成的。现在
        //    1 秒只释放接管（画面冻住），2.5 秒确认真断了才灭灯。
        if (!s_calibrating && s_fcActive &&
            (now - s_lastFcMs) > FC_TIMEOUT_MS) {
            s_fcActive = false;
        }
        if (s_mode == 0 && !s_calibrating && !s_fcActive && !s_fcBlanked &&
            (now - s_lastFcMs) > FC_BLANK_MS) {
            s_fcBlanked = true;        // 真断了：灭灯（只做一次，别每秒刷屏）
            for (int i = 0; i < s_stripLen; i++) s_strip[i] = RGB();
            refreshStrip();
        }
        if (s_mode == 0 && (now - s_lastFcMs) > 3000) {
            if (fcLostLogMs == 0) {
                fcLostLogMs = now;
                logMsg("[RX] 3 秒没收到飞控帧，重置采集链路（不用重启）sil=%lums\n",
                       (unsigned long)(now - s_lastFcMs));
            }
            if (now - fcLostLogMs > 2000) {
                fcLostLogMs = now;
                fcRxReset();
                s_rxResetN++;
            }
        } else {
            fcLostLogMs = 0;
        }
        // v32.33：最后的自救 —— 「灯全灭、BF 重启无效、只有把 ESP32 断电重启才好」
        // 这个形态说明状态坏在 ESP32 自己身上（中断/GPIO 寄存器），软件清不干净。
        // 那就让它自己重启：条件卡得比较死，必须是「本次开机已经正常收过 200 帧
        // 以上」并且「连续 25 秒一帧都没收到」—— 飞控压根没接时 g_rxFrames 是 0，
        // 永远不会触发，也不会陷入重启循环。
        if (s_mode == 0 && !s_calibrating && (uint32_t)g_rxFrames > 200 &&
            (now - s_lastFcMs) > 25000) {
            if (s_autoRstN < 3) {
                s_autoRstN++;
                logMsg("[SYS] 连续 25s 收不到飞控帧（本次开机已收 %u 帧），自动重启恢复（第 %u 次）\n",
                       (unsigned)g_rxFrames, (unsigned)s_autoRstN);
                delayMs(80);
                esp_restart();
            }
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
