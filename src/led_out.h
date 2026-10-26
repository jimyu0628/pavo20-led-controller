#pragma once
// ============================================================
// 灯带输出（RMT TX）：把 RGB 缓冲编码为 WS2812 时序并发送
// ============================================================

#include <stdint.h>
#include "config.h"
#include "effects.h"

// 初始化 RMT TX（setup 中调用一次）
void ledOutInit();

// 把 leds 前 n 颗编码为 WS2812 符号并发送（GRB 顺序）
void showStrip(const RGB* leds, int n);

// v32.29：阻塞等到硬件真的把这一帧发完（最多 30ms）。
// 开飞控采集窗之前必须调它 —— showStrip 只是把事务排进队列就返回了。
void ledOutWaitIdle();

// 已成功发送的帧计数（主循环每秒读一次打印心跳，用于确认信号确实在发）
extern volatile uint32_t g_txCount;
// v32.29：发送失败次数 / 最近错误码。这两个涨说明 RMT 通道被打断过，
// 是「亮部杂色」「灯带全灭」的直接证据（现在失败会自动复位通道自愈）。
extern volatile uint32_t g_txFail;
extern volatile int      g_txLastErr;

// 初始化结果诊断（USB 串口就绪后由 main 打印，避免开机早期日志丢失）
bool ledOutOk();    // TX 通道与编码器是否都创建成功
int  ledOutErr();   // 最近一次初始化失败的错误码（0 = 无错误）
