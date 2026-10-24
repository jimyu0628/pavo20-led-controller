// ============================================================
// 设备名生成（v32.19）
//
// 名字 = "PavoLED-" + MAC 末字节（两位大写十六进制）
//   例：PavoLED-A3
//
// 为什么要 MAC 后缀：飞场里若同时有多块同款板子，SSID 全叫 PavoLED
// 就分不清该连哪块；加后缀后一眼对得上，蓝牙名也同样带上，两边永远一致。
//
// 为什么用 WIFI_STA 的 MAC 当基准：ESP32-C3 的 STA / AP / BLE 三个 MAC
// 是同一批 efuse 地址派生（AP = STA+1，BLE 另一段），各有各的值。
// 我们统一取 STA MAC 的末字节来算后缀，不直接用各自的 MAC —— 否则
// WiFi 叫 -A3、蓝牙叫 -A4，反而更乱。
//
// 长度预算（重要，别随便加长基名）：
//   BLE 广播主包 = flags(3) + FFE0(4) + ABF0(4) + 名字(1 长度 + 1 类型 + N)
//   上限 31 字节 → "PavoLED-A3"（10 字符）时共 22 字节，还有富余。
//   基名再长 4 个字符就会顶到 26，接近上限，不建议。
// ============================================================
#include "device_name.h"
#include <stdio.h>
#include <string.h>
#include "esp_mac.h"

#define DEV_NAME_BASE "PavoLED"

// 只初始化一次：名字字符串是常量，没必要每次调用都算。
// 注意这里不加锁 —— 首次调用发生在初始化阶段（WiFi/BLE 起来之前），
// 两边是顺序调用的；即使并发，最坏结果也只是各算一遍，值相同。
static char   s_name[24];
static bool   s_ready = false;

const char* devName(void) {
    if (s_ready) return s_name;

    uint8_t mac[6] = {0};
    esp_err_t e = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (e == ESP_OK) {
        // 只取末字节：同批次板子 MAC 连续，末字节必不相同；
        // 256 种取值，飞场几台设备足够用，名字也不会太长。
        snprintf(s_name, sizeof(s_name), "%s-%02X", DEV_NAME_BASE, mac[5]);
    } else {
        // 读不到 MAC 就退回无后缀的基名，至少不会起不来
        snprintf(s_name, sizeof(s_name), "%s", DEV_NAME_BASE);
        printf("[名字] 读 MAC 失败(%d)，退回基名 %s\n", (int)e, s_name);
    }
    s_ready = true;
    return s_name;
}
