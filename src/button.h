#pragma once
// ============================================================
// 单键手势检测（GPIO9 / 板载 BOOT 键）
//   单击 / 双击 / 三击 / 长按(0.8s 亮度，可保持) / 长按3s(标定)
// ============================================================

#include <stdint.h>

enum ButtonEvent {
    BTN_NONE = 0,
    BTN_CLICK,      // 单击 → 切换模式
    BTN_DOUBLE,     // 双击 → 切换颜色
    BTN_TRIPLE,     // 三击 → 切换速度
    BTN_BRIGHT,     // 长按 0.8s（保持可重复）→ 切换亮度
    BTN_CAL,        // 长按 3s → 进入/退出标定
};

// 初始化（pinMode）
void buttonInit();

// 轮询（每 5ms 调用一次），返回一个事件（无事件返回 BTN_NONE）
ButtonEvent pollButton();
