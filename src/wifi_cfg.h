#pragma once
// ============================================================
// WiFi 热点 + Web 设置（ESP32-C3 AP 模式）
//   - 热点名 RGBled / 密码 12345678（WPA2）
//   - 上电开启；从未有手机连接则 30s 后自动关闭
//   - 手机连接过（曾连上）则保持；关闭网页（无 HTTP 请求）60s 后关闭
//   - 网页点击"关闭 WiFi"立即关闭（需重启设备重新开启）
//   - Web 设置：模式/颜色/速度/亮度（复用按键同一套状态）
// ============================================================
#include <stdint.h>

// 初始化 AP + HTTP 服务器（setup 中调用一次，需在 nvsInit 之后）
void wifiCfgInit();

// 主循环每秒调用：执行"关闭 WiFi"判定
void wifiCfgPoll(uint32_t nowMs);

// 热点状态：0=已关闭 1=开启(未连过) 2=开启(曾连/保持期)
int wifiCfgState();

// 热点诊断字符串（SSID/IP），供心跳显示；热点关闭时返回 ""
const char* wifiCfgStr();

// ---- Web 设置接口（由 main.cpp 实现，读写主程序状态） ----
int  webGetMode();
int  webGetColor();
int  webGetSpeed();
int  webGetBright();
int  webGetCalib();
bool webIsCalibrating();
void webSetMode(int v);
void webSetColor(int v);
void webSetSpeed(int v);
void webSetBright(int v);
void webSetCount(int v); // 设置 LED 数量（1~128，断电记忆，等价按键标定）
// 立即按当前设置刷新灯带（模式切换即时生效）
void webApplyNow();
