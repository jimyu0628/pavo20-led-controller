#pragma once
// ============================================================
// 灯带输出（v32.32 起走 SPI2 + DMA）：把 RGB 缓冲编码为 WS2812 时序并发送
//
// 为什么不再用 RMT：ESP32-C3 的 RMT **没有 DMA**，长帧靠中断每 60µs 续填一次
// FIFO，WiFi/BLE 任何一次关中断（或飞控采集的 1.3ms 屏蔽）都会让它断流 →
// 灯带频闪 / 收到残帧 / 通道卡死。SPI + DMA 由硬件搬运，不依赖中断。
// ============================================================

#include <stdint.h>
#include "config.h"
#include "effects.h"

// 初始化 SPI2 MOSI 输出（setup 中调用一次）
void ledOutInit();

// 把 leds 前 n 颗编码为 WS2812 时序并发送（GRB 顺序）。
// v32.32：DMA 传输是同步的（返回时已全部发完），所以它天然是「阻塞到发完」。
void showStrip(const RGB* leds, int n);

// v32.32：保留接口，现在是空实现 —— SPI+DMA 的 showStrip 返回时就已经发完了，
// 不需要再等。留着是为了不改主循环的时序结构（也兼容以后换回异步输出）。
void ledOutWaitIdle();

// 已成功发送的帧计数（主循环每秒读一次打印心跳，用于确认信号确实在发）
extern volatile uint32_t g_txCount;
// v32.29：发送失败次数 / 最近错误码。这两个涨说明 RMT 通道被打断过，
// 是「亮部杂色」「灯带全灭」的直接证据（现在失败会自动复位通道自愈）。
extern volatile uint32_t g_txFail;
extern volatile int      g_txLastErr;
// v32.39：编码结果的校验和（FNV-1a）与「和上一帧不同」的次数。
//   画面静止时 txchg 必须恒为 0 —— 那说明交给 DMA 的字节流一帧都没变过。
extern volatile uint32_t g_txSum;
extern volatile uint32_t g_txSumChg;
int ledTxRepeat();    // 每帧连发几遍（v32.39 = 2）
int ledTxResetUs();   // 帧内复位低电平长度（µs，v32.39 ≈ 333）

// 初始化结果诊断（USB 串口就绪后由 main 打印，避免开机早期日志丢失）
bool ledOutOk();    // TX 通道与编码器是否都创建成功
int  ledOutErr();   // 最近一次初始化失败的错误码（0 = 无错误）
