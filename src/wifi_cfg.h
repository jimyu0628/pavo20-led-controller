#pragma once
// ============================================================
// WiFi 热点 + Web 设置（ESP32-C3 AP 模式）
//   - 热点名 PavoLED-XX（与 BLE 广播名同源，device_name.h）/ 密码 12345678（WPA2）
//   - 上电开启；captive portal（DNS 劫持 + 302）让手机自动弹控制页
//   - v32.21 关闭策略：没有设备连热点、没有 HTTP 请求、没有蓝牙连接，
//     三者同时安静满 60 秒 → 关 WiFi + 蓝牙，只留灯带（需断电重启才恢复）
//     网页上不再提供「关闭 WiFi」按钮，全部交给闲置判定
//   - Web 设置：模式/颜色/速度/亮度/灯带长度（复用按键同一套状态）
// ============================================================
#include <stdint.h>

// 初始化 AP + HTTP 服务器（setup 中调用一次，需在 nvsInit 之后）
void wifiCfgInit();

// 主循环每秒调用：执行闲置关闭判定（到点在 pwrTask 里真正动手）
void wifiCfgPoll(uint32_t nowMs);

// 热点状态：0=已关闭 1=开启(未连过) 2=开启(曾连/保持期)
int wifiCfgState();

// 热点诊断字符串（SSID/IP），供心跳显示；热点关闭时返回 ""
const char* wifiCfgStr();

// 开机信息（版本/灯数/模式等），串口错过启动日志时 Web 调试区可看
void wifiCfgSetBootInfo(const char* s);
// v32.14：热点诊断串（在线设备数 + 距自动关闭剩余秒数，off<0 表示有人在线不会关）
const char* wifiCfgDiag();
const char* wifiCfgBootInfo();

// v32.21：距自动关闭还剩几秒（网页「连接」页倒计时）；-1 = 有人在用、不会关
int wifiCfgIdleLeft();

// DNS 劫持（captive portal）是否绑定成功：1=ok 0=fail/未启动
int wifiCfgDnsOk();

// ---- Web 设置接口（由 main.cpp 实现，读写主程序状态） ----
int  webGetMode();
int  webGetColor();
int  webGetSpeed();   // v32.21：1~100（原来是 0/1/2 档位）
int  webGetBright();  // v32.21：0~100（原来是 0~3 档位）
int  webGetCalib();
bool webIsCalibrating();
void webSetMode(int v);
void webSetColor(int v);
void webSetSpeed(int v);
void webSetBright(int v);
void webSetCount(int v); // 设置 LED 数量（1~128，断电记忆，等价按键标定）
int  webGetFcOrder();    // v32.23：飞控 LED 字节顺序索引 0~5（0=GRB…5=BGR）
void webSetFcOrder(int v); // v32.23：改飞控字节顺序（颜色不对时逐个试），断电记忆
// v32.29：运行健康诊断（网页日志显示，专治「灯全灭、日志也不动」这类现场问题）
int  webGetTxRate();     // 每秒真正推给灯带的帧数（0 = 主循环没跑或 RMT 废了）
int  webGetTxFail();     // RMT 发送失败累计（涨过 = 输出被打断过）
int  webGetLoopGap();    // 最近一秒主循环最长间隔（ms，正常约 10）
int  webGetFcSilent();   // 距上一帧飞控信号的秒数
int  webGetRxReset();    // 采集链路自愈次数
// 立即按当前设置刷新灯带（模式切换即时生效）
void webApplyNow();
