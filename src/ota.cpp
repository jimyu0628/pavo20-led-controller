#include "ota.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char* OTA_VER = "v32.9"; // 版本号唯一来源；小版本号区分每次修正

const char* otaVersion() { return OTA_VER; }

// ---------------- 固件升级页（HTML，手机适配） ----------------
static const char OTA_PAGE[] = R"rawliteral(<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>固件升级</title><style>
body{font-family:system-ui,-apple-system,sans-serif;background:#111;color:#eee;margin:0;padding:16px;max-width:560px;margin:0 auto}
h1{font-size:20px;color:#ffb454;margin:8px 0 4px}
.card{background:#1d1d1d;border-radius:10px;padding:12px 14px;margin:10px 0;font-size:14px;color:#ccc}
button{min-height:44px;min-width:140px;border:1px solid #36c;background:#1f3a52;color:#eee;border-radius:8px;font-size:15px;margin-top:8px}
input[type=file]{margin:8px 0;font-size:14px;color:#ccc}
#st{color:#9be;font-size:13px;margin-top:8px}
.warn{color:#888;font-size:12px;line-height:1.7}
</style></head><body>
<h1>固件升级（WiFi OTA）</h1>
<div class="card">
 <p style="margin:4px 0">选择 GitHub Actions 构件里的 <b>firmware.bin</b>，上传后自动写入并重启。</p>
 <input type="file" id="fw" accept=".bin">
 <br>
 <button onclick="up()">上传并升级</button>
 <div id="st">上传前请确认设备供电稳定。</div>
</div>
<div class="card warn">
 升级期间不要断电；失败会自动回滚旧固件（不会变砖）。<br>
 完成后设备自动重启，重新连接热点 <b>RGBled</b> 即可。
</div>
<script>
function up(){
 var f=document.getElementById("fw").files[0];
 var st=document.getElementById("st");
 if(!f){st.textContent="请先选择 firmware.bin";return;}
 if(f.size>1500000){st.textContent="文件过大（应小于 1.5MB）";return;}
 var fd=new FormData();fd.append("fw",f);
 st.textContent="上传中…约 10~30 秒，请勿断电";
 fetch("/api/ota",{method:"POST",body:fd})
  .then(function(r){return r.json();})
  .then(function(j){st.textContent=j.ok?"升级成功，设备重启中…":"失败："+j.msg;})
  .catch(function(){st.textContent="连接中断：设备正在重启（正常）";});
}
</script>
</body></html>)rawliteral";

static esp_err_t handler_ota_page(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    httpd_resp_sendstr(req, OTA_PAGE);
    return ESP_OK;
}

// ---------------- multipart 流式接收 + 写入 OTA 分区 ----------------
static char* memFindIn(char* b, size_t n, const char* s, size_t m) {
    if (m == 0 || m > n) return NULL;
    for (size_t i = 0; i + m <= n; i++)
        if (memcmp(b + i, s, m) == 0) return b + i;
    return NULL;
}

static esp_err_t sendJsonErr(httpd_req_t* req, const char* msg) {
    char b[96];
    snprintf(b, sizeof(b), "{\"ok\":0,\"msg\":\"%s\"}", msg);
    httpd_resp_sendstr(req, b);
    return ESP_OK;
}

static esp_err_t handler_ota_post(httpd_req_t* req) {
    char ctBuf[192];
    if (httpd_req_get_hdr_value_str(req, "Content-Type", ctBuf, sizeof(ctBuf)) != ESP_OK)
        return sendJsonErr(req, "no content-type");
    const char* ct = ctBuf;
    const char* bd = strstr(ct, "boundary=");
    if (!bd) return sendJsonErr(req, "no boundary");
    bd += 9;
    char boundary[80];
    int bl = 0;
    while (bd[bl] && bd[bl] != ';' && bl < 79) { boundary[bl] = bd[bl]; bl++; }
    boundary[bl] = 0;
    char sep[96]; // 数据结束分隔符：\r\n--boundary
    int sl = snprintf(sep, sizeof(sep), "\r\n--%s", boundary);
    static const char hdrEnd[4] = {'\r', '\n', '\r', '\n'};

    const esp_partition_t* up = esp_ota_get_next_update_partition(NULL);
    if (!up) return sendJsonErr(req, "no ota partition");
    esp_ota_handle_t oh;
    esp_err_t er = esp_ota_begin(up, OTA_SIZE_UNKNOWN, &oh);
    if (er != ESP_OK) return sendJsonErr(req, "ota begin fail");

    static char acc[2048 + 96]; // 累积缓冲（httpd 单 task 顺序执行，无重入；static 避免占用 handler 栈）
    static char buf[1024];
    size_t accLen = 0;
    int phase = 0;         // 0=头部(找 \r\n\r\n) 1=数据(找结束分隔符)
    size_t written = 0;
    int r;
    esp_err_t failErr = ESP_OK;

    while ((r = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        if (accLen + r > sizeof(acc)) { failErr = ESP_FAIL; break; } // 不应发生（数据阶段边收边写）
        memcpy(acc + accLen, buf, r);
        accLen += r;
        if (phase == 0) {
            char* he = memFindIn(acc, accLen, hdrEnd, 4);
            if (!he) {
                if (accLen > 4096) { failErr = ESP_FAIL; break; }
                continue;
            }
            size_t skip = (size_t)(he - acc) + 4;
            memmove(acc, acc + skip, accLen - skip);
            accLen -= skip;
            phase = 1;
        }
        if (phase == 1) {
            char* sp = memFindIn(acc, accLen, sep, (size_t)sl);
            if (sp) {
                size_t dn = (size_t)(sp - acc); // 数据到分隔符前（不含结尾 CRLF，sp 指向 \r\n）
                if (dn > 0) {
                    er = esp_ota_write(oh, acc, dn);
                    if (er != ESP_OK) { failErr = er; break; }
                    written += dn;
                }
                accLen = 0;
                break; // 文件数据完整收到，忽略尾部
            }
            size_t keep = (sl > 1) ? (size_t)(sl - 1) : 0; // 保留窗口，防分隔符跨 chunk
            size_t safe = accLen > keep ? accLen - keep : 0;
            if (safe > 0) {
                er = esp_ota_write(oh, acc, safe);
                if (er != ESP_OK) { failErr = er; break; }
                written += safe;
                memmove(acc, acc + safe, keep);
                accLen = keep;
            }
        }
    }

    if (failErr == ESP_OK && phase == 1 && accLen > 0) { // 无分隔符即结束：写残余由 esp_ota_end 校验
        er = esp_ota_write(oh, acc, accLen);
        if (er != ESP_OK) failErr = er;
        else written += accLen;
    }

    if (failErr != ESP_OK) {
        esp_ota_abort(oh);
        return sendJsonErr(req, "write fail");
    }
    er = esp_ota_end(oh);
    if (er != ESP_OK) {
        printf("[OTA] image verify fail %d (%u bytes)\n", er, (unsigned)written);
        return sendJsonErr(req, "verify fail");
    }
    er = esp_ota_set_boot_partition(up);
    if (er != ESP_OK) return sendJsonErr(req, "set boot fail");

    printf("[OTA] OK, %u bytes, rebooting...\n", (unsigned)written);
    httpd_resp_sendstr(req, "{\"ok\":1}");
    vTaskDelay(pdMS_TO_TICKS(600));
    esp_restart();
    return ESP_OK; // 不会执行到
}

// ---------------- 注册路由 ----------------
void otaHttpdRegister(httpd_handle_t server) {
    static const httpd_uri_t u_page = {.uri = "/ota", .method = HTTP_GET, .handler = handler_ota_page, .user_ctx = NULL};
    static const httpd_uri_t u_post = {.uri = "/api/ota", .method = HTTP_POST, .handler = handler_ota_post, .user_ctx = NULL};
    httpd_register_uri_handler(server, &u_page);
    httpd_register_uri_handler(server, &u_post);
    printf("[OTA] 升级页就绪：/ota（上传 firmware.bin 无线升级）\n");
}
