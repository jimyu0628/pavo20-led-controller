#pragma once
// ============================================================
// BLE NUS（Nordic UART Service）透传（v32.5，手机蓝牙调参 A 路线）
//   - ESP32-C3 内置 BLE：广播设备名 PavoLED，暴露标准 NUS 服务
//   - 与 SpeedyBee App 的蓝牙调参模块协议一致：MSP 帧透明透传
//   - UART1（GPIO6 TX / GPIO7 RX，115200）桥接飞控 UART1（A09/A10）
//     手机 -> BLE 写 -> NUS RX -> UART1 TX(GPIO6) -> 飞控 RX(A10)
//     飞控 TX(A09) -> UART1 RX(GPIO7) -> NUS TX 通知 -> 手机
//   注意：不用 UART0(GPIO20/21)——与板载 USB 串口(CH340)共用，会双驱动互拉
// ============================================================
#include <stdbool.h>
#include <stdint.h>

// 初始化 BLE 广播 + NUS 服务 + UART1 桥接（app_main 调用一次）
void bleUartInit(void);

// 心跳状态： "ble adv"（广播中）/ "ble conn"（已连接并订阅）/ "ble err"（异常）
const char* bleUartState(void);

// 一行诊断串：sub=订阅 mtu=协商值 nf=通知成功/失败 e=最后错误码 probe=MSP探测
const char* bleUartDiag(void);

// 透传字节计数（心跳诊断：手机->飞控 tx / 飞控->手机 rx）
uint32_t bleUartTxCount(void);
uint32_t bleUartRxCount(void);

// MSP 探测结果：-1 未测 / 0 飞控无回复 / 1 飞控有回复
int bleUartProbe(void);
