#pragma once
// ============================================================
// WiFi OTA 无线升级（v31）
//   - /ota   ：固件升级上传页
//   - /api/ota：POST multipart 流式接收 firmware.bin → 写入备用 OTA 分区 → 重启
//   - 双 OTA 分区（ota_0/ota_1 + otadata）：失败自动回滚
// 升级文件：GitHub Actions 构件里的 firmware.bin（app 分区固件，非 merged.bin）
// ============================================================
#include "esp_http_server.h"

// 注册 /ota 与 /api/ota 路由（在 wifiCfgInit 的 httpd_start 成功后调用）
void otaHttpdRegister(httpd_handle_t server);

// 固件版本号（心跳/网页显示；需与 main.cpp 版本头同步）
const char* otaVersion();
