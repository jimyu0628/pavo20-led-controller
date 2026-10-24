// MSP over TCP（v32.10）：把 UART1 上的飞控 MSP 暴露成 TCP 服务，端口 5761
// 背景：SpeedyBee App 与 Betaflight App 的 BLE 通道都只认厂家模块，
//       第三方 BLE 桥很难通过它们的识别。但 Betaflight Configurator 支持
//       「Manual Selection -> tcp://IP:5761」（原本给 ELRS 接收机 WiFi 用）。
//       本板本来就有 WiFi AP，直接当 ELRS 那样的 TCP-MSP 网关即可，
//       不依赖任何 App 的 UUID 白名单。
#pragma once
#include <stdint.h>

// 创建监听任务（内部会重试，WiFi 未开或稍后才开都能自己起来）
void mspTcpInit(void);

// UART->手机方向：把飞控回包转发给已连接的 TCP 客户端。
// 没有客户端时返回 0（调用方无需判断）。
int mspTcpSend(const uint8_t* p, int n);

int mspTcpConnected(void);

// 心跳/网页用：tcp=0/1 rx=<客户端发来字节> tx=<发回客户端字节>
const char* mspTcpDiag(void);
