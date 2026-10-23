#pragma once
// ============================================================
// BLE NUS（Nordic UART Service）透传（v32，手机蓝牙调参 A 路线）
//   - ESP32-C3 内置 BLE：广播设备名 PavoLED，暴露标准 NUS 服务
//   - 与 SpeedyBee App 的蓝牙调参模块协议一致：MSP 帧透明透传
//   - UART0（GPIO21 TX / GPIO20 RX，115200）桥接飞控 UART3（B10/B11）
//   手机 -> BLE 写 -> NUS RX -> UART0 TX(GPIO21) -> 飞控 RX(B11)
//   飞控 TX(B10) -> UART0 RX(GPIO20) -> NUS TX 通知 -> 手机
// ============================================================
#include <stdbool.h>

// 初始化 BLE 广播 + NUS 服务 + UART0 桥接（app_main 调用一次）
void bleUartInit(void);

// 心跳状态： "ble adv"（广播中）/ "ble conn"（已连接）/ "ble off"（异常）
const char* bleUartState(void);
