#pragma once
// ============================================================
// 设备名（v32.19）：WiFi 热点 SSID 与 BLE 广播名共用同一个运行时名字
//   - 基名 PavoLED，后缀取芯片 MAC 末字节，例：PavoLED-A3
//   - 两边都调 devName()，从同一个 MAC 算出来，天然一致
//   - 飞场里同时有多块板子时，靠后缀区分谁的
//
// 用法：把 DEV_NAME 当字符串用即可（它是函数调用，不是字面量）。
//   printf("...%s", DEV_NAME);           ✓
//   ble_svc_gap_device_name_set(DEV_NAME); ✓
//   "字面量" DEV_NAME                      ✗ 不能拼接
// ============================================================
const char* devName(void);
#define DEV_NAME devName()
