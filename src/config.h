#pragma once
// ============================================================
// 幻彩灯带控制板 · 全局配置（ESP32-C3 SuperMini）
// 复刻「幻彩灯带使用说明书」控制板全部功能：
//   Betaflight 模式(1:1/1:2/1:4) + 9 种灯效 + 灯珠数量标定
//   单键操作（板载 BOOT 键：单击/双击/三击/长按/长按3秒）
// ============================================================

// ---------- 引脚定义（SuperMini 丝印） ----------
// 使用 IDF 的 GPIO_NUM_x 枚举（gpio_num_t），适配 RMT/GPIO 驱动配置结构
#include "driver/gpio.h"
#define PIN_FC_IN    GPIO_NUM_4   // 飞控 LED 焊盘信号输入（RMT RX），3.3V 直连
#define PIN_LED_OUT  GPIO_NUM_3   // 灯带 DIN 数据输出（RMT TX）
#define PIN_KEY      GPIO_NUM_9   // 板载 BOOT 按键（按下为低电平），作为唯一按键

// ---------- 灯带参数 ----------
// 100 灯/米 × 55cm = 55 颗；此处为缓冲上限，实际长度以标定值为准
#define STRIP_MAX_LEDS   64    // 灯带缓冲上限（≤128，按 128 颗上限留余量）
#define STRIP_DEFAULT_LEDS 55  // 出厂默认灯数（按你实际灯带填写，可重新标定）

// Betaflight 侧：F411 飞控 LED 最多 32 颗（F405 可 64），取 32 上限
#define FC_MAX_LEDS      32

// ---------- RMT 时序（WS2812，800kHz） ----------
// ESP32-C3 的 RMT 可靠时钟源为 40MHz XTAL（不支持 80MHz 分辨率）
// 40MHz：1 tick = 25ns；以下时序按 40MHz 换算，实际时序与 80MHz 版一致
#define RMT_CLK_SRC      RMT_CLK_SRC_XTAL  // 40MHz 晶振，C3 必选可用
#define RMT_RES_HZ       40000000ULL       // 40MHz，1 tick = 25ns
#define RMT_MEM_BLOCK    64                // 每通道内存块符号数（默认 64）
#define RX_ACC_MAX       896               // 累积缓冲：32颗×24bit=768 + 余量
#define TICK_HI_THRESHOLD  24              // 高电平 ≥ 0.6µs(24 tick) 判为 bit1

// 编码参数（单位：tick @40MHz）
#define T1H_TICKS  32    // bit1 高 800ns
#define T1L_TICKS  18    // bit1 低 450ns
#define T0H_TICKS  16    // bit0 高 400ns
#define T0L_TICKS  34    // bit0 低 850ns
#define RESET_TICKS 3200 // 复位 80µs

// ---------- 时序逻辑 ----------
#define FC_TIMEOUT_MS    500   // Betaflight 模式：无信号超时后灯带熄灭
#define RX_FRAME_GAP_US  4000  // 两次接收间隔 >4ms 判定一帧结束（单帧约1ms，帧间隔约10~20ms）
#define RX_SIGNAL_MIN_NS 200   // RX 过滤 <200ns 噪声
#define RX_SIGNAL_MAX_NS 40000 // 低电平持续 >40µs 视为帧间复位，RX 停止

// ---------- 按键手势 ----------
#define KEY_PRESS_MS      300   // 单击最大按住时长
#define KEY_DOUBLE_GAP_MS 400   // 两次点击最大间隔（区分单击/双击/三击）
#define KEY_LONG_MS       800   // 长按触发亮度
#define KEY_CAL_MS        3000  // 长按 3s 进入/退出标定
#define KEY_REPEAT_MS     500   // 长按保持时亮度循环间隔

// ---------- 灯效参数 ----------
#define MODE_COUNT     10    // 模式总数：0=Betaflight，1~9=灯效
#define COLOR_COUNT    8     // 8 色 + 关闭
#define SPEED_COUNT    3     // 快/中/慢
#define BRIGHT_COUNT   4     // 100/75/50/25%
#define CALIB_MAX      128   // 标定灯数上限（同说明书）
