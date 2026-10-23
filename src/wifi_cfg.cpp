#include "wifi_cfg.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota.h"

// ============================================================
// WiFi 热点 + Web 设置（AP 模式，ESP32-C3）
//
// 关闭逻辑（wifiCfgPoll 每秒判定，与用户需求一致）：
//   1. 网页点击"关闭 WiFi" → 立即关闭
//   2. 已开启且从未有手机连接 → 上电 30s 后关闭
//   3. 曾连接过（保持期）→ 距最后 HTTP 请求 >60s（关闭网页/无操作）→ 关闭
// 备注：关闭后需重启设备才能重新开启热点
// ============================================================

#define WIFI_SSID       "RGBled"
#define WIFI_PASS       "12345678"
#define WIFI_CHANNEL    1
#define WIFI_MAX_CONN   4
#define OFF_NO_HTTP_MS  300000  // 关闭网页/无操作 5 分钟后关（v25 从 60s 延长）
#define OFF_NO_STA_MS   300000  // 上电后 5 分钟无手机连接则关（v25 从 30s 延长）

static httpd_handle_t s_httpd = NULL;
static volatile bool  s_wifiUp = false;
static volatile bool  s_everConnected = false; // 是否曾有手机连上（连上后进入保持期）
static volatile bool  s_offRequested = false;  // 网页请求关闭
static volatile bool  s_stopping = false;
static uint64_t s_startUs = 0;   // AP 开启时刻（µs）
static volatile uint64_t s_lastHttpUs = 0; // 最后 HTTP 请求时刻（µs）
static char s_cfgStr[64];                  // 启动时缓存 "SSID/IP" 供心跳显示（SSID 最长 32 + IP 15 + 前缀后缀，64 防截断警告）

// ---------- 内部 ----------
static void wifiStopAll();

// ---------- WiFi 事件：手机连上 ----------
static void wifiEvt(void* arg, esp_event_base_t base, int32_t id, void* data) {
    (void)arg; (void)base; (void)data;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        s_everConnected = true;
        printf("[WiFi] 手机已连接（进入保持期）\n");
    }
}

// ---------- HTTP：JSON 状态 ----------
static void touchHttp() { s_lastHttpUs = esp_timer_get_time(); }

static void sendState(httpd_req_t* req) {
    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"mode\":%d,\"color\":%d,\"speed\":%d,\"bright\":%d,\"calib\":%d,\"cal\":%d,\"v\":\"%s\"}",
             webGetMode(), webGetColor(), webGetSpeed(), webGetBright(),
             webGetCalib(), webIsCalibrating() ? 1 : 0, otaVersion());
    httpd_resp_set_hdr(req, "Content-Type", "application/json; charset=utf-8");
    httpd_resp_sendstr(req, buf);
}

// ---------- HTTP：设置 ----------
// /api/set?mode=3&color=1&speed=2&bright=0 （缺省参数不修改）
static esp_err_t handler_set(httpd_req_t* req) {
    touchHttp();
    char q[96];
    char v[8];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "mode", v, sizeof(v)) == ESP_OK)
            webSetMode(atoi(v));
        if (httpd_query_key_value(q, "color", v, sizeof(v)) == ESP_OK)
            webSetColor(atoi(v));
        if (httpd_query_key_value(q, "speed", v, sizeof(v)) == ESP_OK)
            webSetSpeed(atoi(v));
        if (httpd_query_key_value(q, "bright", v, sizeof(v)) == ESP_OK)
            webSetBright(atoi(v));
        if (httpd_query_key_value(q, "count", v, sizeof(v)) == ESP_OK)
            webSetCount(atoi(v));
    }
    webApplyNow(); // 立即生效（灭灯/渲染）
    sendState(req);
    return ESP_OK;
}

// ---------- HTTP：状态 ----------
static esp_err_t handler_state(httpd_req_t* req) {
    touchHttp();
    sendState(req);
    return ESP_OK;
}

// ---------- HTTP：关闭 WiFi ----------
static esp_err_t handler_off(httpd_req_t* req) {
    touchHttp();
    s_offRequested = true; // 由 wifiCfgPoll 执行真正关闭（handler 内不可 stop httpd）
    httpd_resp_set_hdr(req, "Content-Type", "application/json; charset=utf-8");
    httpd_resp_sendstr(req, "{\"ok\":1}");
    return ESP_OK;
}

// ---------- HTTP：主页（内嵌页面，手机适配） ----------
static const char HTML_PAGE[] = R"rawliteral(<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>幻彩灯带控制板</title>
<style>
 body{font-family:system-ui,-apple-system,"Segoe UI",sans-serif;background:#111;color:#eee;margin:0;padding:16px;max-width:560px;margin:0 auto}
 h1{font-size:20px;margin:8px 0 4px} .sub{color:#888;font-size:13px;margin-bottom:14px}
 .card{background:#1d1d1d;border-radius:10px;padding:12px 14px;margin:10px 0}
 .card h3{margin:0 0 10px;font-size:14px;color:#ffb454}
 .btns{display:flex;flex-wrap:wrap;gap:8px}
 button{flex:1 1 auto;min-width:72px;min-height:44px;border:1px solid #333;background:#222;color:#eee;border-radius:8px;font-size:14px}
 button.on{background:#e08e0b;color:#000;border-color:#e08e0b;font-weight:bold}
 .sw{width:44px;height:44px;border-radius:8px;border:2px solid #333}
 .sw.on{border:2px solid #fff;transform:scale(1.08)}
 #st{font-size:13px;color:#9be;background:#14202e;padding:8px 12px;border-radius:8px;margin-bottom:6px}
 #toast{position:fixed;left:0;right:0;bottom:24px;text-align:center;color:#000;background:#ffb454;padding:10px;font-weight:bold;display:none}
 .danger{background:#7a1f1f;border-color:#a33}
</style>
</head>
<body>
<h1>幻彩灯带控制板</h1>
<div class="sub">WiFi 热点：RGBled · 密码 12345678 · 无操作 60 秒后自动关闭</div>
<div id="st">加载中…</div>

<div class="card"><h3>模式</h3><div class="btns" id="modes"></div></div>
<div class="card"><h3>颜色</h3><div class="btns" id="colors"></div></div>
<div class="card"><h3>速度</h3><div class="btns" id="speeds"></div></div>
<div class="card"><h3>亮度</h3><div class="btns" id="brights"></div></div>

<div class="card"><h3>LED 数量</h3>
 <div style="display:flex;gap:8px;align-items:center">
  <input id="cnt" type="number" min="1" max="128" value="55"
         style="flex:1;min-height:44px;background:#222;border:1px solid #333;color:#eee;border-radius:8px;font-size:16px;padding:0 10px">
  <button style="flex:0 0 auto" onclick="setCount()">应用</button>
 </div>
 <div class="btns" style="margin-top:8px">
  <button onclick="setCountV(32)">32</button>
  <button onclick="setCountV(55)">55</button>
  <button onclick="setCountV(60)">60</button>
  <button onclick="setCountV(78)">78</button>
 </div>
 <div style="color:#888;font-size:12px;margin-top:6px">即时生效并断电记忆；与按键标定（长按 3 秒）等效</div>
</div>

<div class="card">
 <button id="btnOff" class="danger" onclick="offWifi()">关闭 WiFi（需重启才能重新开启）</button>
</div>
<div class="card">
 <button style="background:#1f3a52;border-color:#36c" onclick="location.href='/ota'">固件升级（WiFi OTA）</button>
</div>
<div id="toast"></div>

<script>
var M=["飞控(Betaflight)","单色常亮","单色呼吸","单色跑马","双边跑马","单色导向","单色增长","彩虹呼吸","彩虹渐变","双边彩虹"];
var C=["白","红","黄","蓝","绿","青","品红","关"];
var CS=["#fff","#f33","#ff3","#36f","#3f3","#0ff","#f3f","#000"];
var S=["快","中","慢"];
var B=["100%","75%","50%","25%"];
var st={mode:0,color:0,speed:1,bright:2,calib:55,cal:0,v:"v31"};

function render(){
 document.getElementById("st").textContent =
  "模式："+M[st.mode]+" · 颜色："+C[st.color]+" · 速度："+S[st.speed]+
  " · 亮度："+B[st.bright]+" · 灯数："+st.calib+(st.cal?" · 标定中":"")+" · 固件 "+st.v;
 var c=document.getElementById("cnt"); if(c)c.value=st.calib;
 var mm=document.getElementById("modes"); mm.innerHTML="";
 for(var i=0;i<10;i++){var b=document.createElement("button");b.textContent=i+" "+M[i];
  if(i==st.mode)b.className="on";b.onclick=(function(n){return function(){set("mode",n);};})(i);mm.appendChild(b);}
 var cc=document.getElementById("colors"); cc.innerHTML="";
 for(var i=0;i<8;i++){var s=document.createElement("button");s.className="sw";
  s.style.background=CS[i];if(i==st.color)s.classList.add("on");
  s.title=C[i];s.onclick=(function(n){return function(){set("color",n);};})(i);cc.appendChild(s);}
 var ss=document.getElementById("speeds"); ss.innerHTML="";
 for(var i=0;i<3;i++){var b=document.createElement("button");b.textContent=S[i];
  if(i==st.speed)b.className="on";b.onclick=(function(n){return function(){set("speed",n);};})(i);ss.appendChild(b);}
 var bb=document.getElementById("brights"); bb.innerHTML="";
 for(var i=0;i<4;i++){var b=document.createElement("button");b.textContent=B[i];
  if(i==st.bright)b.className="on";b.onclick=(function(n){return function(){set("bright",n);};})(i);bb.appendChild(b);}
}
function set(k,n){fetch("/api/set?"+k+"="+n).then(function(r){return r.json();}).then(function(j){st=j;render();});}
function setCount(){var n=parseInt(document.getElementById("cnt").value,10);
 if(!n||n<1||n>128){toast("请输入 1-128");return;}set("count",n);}
function setCountV(n){document.getElementById("cnt").value=n;set("count",n);}
function offWifi(){if(confirm("确定关闭 WiFi？关闭后需重启设备才能重新开启。")){
 fetch("/api/wifi/off").then(function(){toast("WiFi 已关闭");setTimeout(function(){document.getElementById("btnOff").disabled=true;},300);});}}
function toast(t){var d=document.getElementById("toast");d.textContent=t;d.style.display="block";setTimeout(function(){d.style.display="none";},2500);}
fetch("/api/state").then(function(r){return r.json();}).then(function(j){st=j;render();}).catch(function(){document.getElementById("st").textContent="连接失败";});
</script>
</body></html>)rawliteral";

static esp_err_t handler_root(httpd_req_t* req) {
    touchHttp();
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    httpd_resp_sendstr(req, HTML_PAGE);
    return ESP_OK;
}

// ========== Captive Portal：DNS 劫持（手机探测域名 → 192.168.4.1） ==========
// 手机连上热点后会自动探测网络（generate_204 / captive.apple.com 等），
// 劫持 DNS 应答为 AP 地址，配合 HTTP 302 重定向，手机自动弹出设置页。
static void dnsTask(void* arg) {
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { printf("[WiFi] dns socket FAIL\n"); vTaskDelete(NULL); return; }
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        // 53 被占用：dhcps 可能已内置 DNS 应答，无需自建
        printf("[WiFi] dns bind fail (dhcps DNS likely active)\n");
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    printf("[WiFi] DNS captive: 53 -> 192.168.4.1\n");
    char buf[512];
    for (;;) {
        struct sockaddr_in from = {};
        socklen_t fl = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&from, &fl);
        if (n < 12) continue;
        uint16_t qd = (uint16_t)(((uint8_t)buf[4]) << 8 | (uint8_t)buf[5]);
        if (qd == 0) continue;
        int pos = 12;
        while (pos < n && buf[pos] != 0) pos += (uint8_t)buf[pos] + 1;
        if (pos >= n) continue;
        pos += 5; // 跳过 00 + QTYPE(2) + QCLASS(2)
        if (pos > n || 12 + (pos - 12) + 16 > (int)sizeof(buf)) continue;
        uint16_t nf = (uint16_t)(((uint8_t)buf[2]) << 8 | (uint8_t)buf[3]);
        nf |= 0x8080; nf &= ~0x000F;  // QR=1 RA=1 RCODE=0
        buf[2] = (uint8_t)(nf >> 8); buf[3] = (uint8_t)(nf & 0xFF);
        buf[6] = 0; buf[7] = 1;   // ANCOUNT=1
        buf[8] = 0; buf[9] = 0;   // NSCOUNT=0
        buf[10] = 0; buf[11] = 0; // ARCOUNT=0
        int p = pos;
        buf[p++] = 0xC0; buf[p++] = 0x0C;              // 指针 → 问题名（offset 12）
        buf[p++] = 0; buf[p++] = 1;                    // TYPE A
        buf[p++] = 0; buf[p++] = 1;                    // CLASS IN
        buf[p++] = 0; buf[p++] = 0; buf[p++] = 0; buf[p++] = 60; // TTL 60s
        buf[p++] = 0; buf[p++] = 4;                    // RDLENGTH=4
        uint32_t ip = htonl(0xC0A80101);               // 192.168.4.1
        memcpy(buf + p, &ip, 4); p += 4;
        sendto(sock, buf, p, 0, (struct sockaddr*)&from, fl);
    }
}

// ========== Captive Portal：HTTP 重定向（探测路径 → 设置页首页） ==========
static esp_err_t handler_captive(httpd_req_t* req) {
    if (strncmp(req->uri, "/api/", 5) == 0) { // 不劫持 API，避免干扰网页轮询
        httpd_resp_send_404(req);
        return ESP_OK;
    }
    touchHttp();
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

// ---------- 关闭全部（httpd + wifi） ----------
static void wifiStopAll() {
    if (!s_wifiUp) return;
    s_stopping = true;
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    s_wifiUp = false;
    s_cfgStr[0] = 0;
    printf("[WiFi] 热点已关闭\n");
}

// ---------- 初始化 ----------
void wifiCfgInit() {
    // IDF 初始化顺序（缺一不可）：netif → event_loop → AP netif → wifi_init
    // 每一步收集返回码进 s_cfgStr（心跳每秒可见；启动 printf 可能被 USB 串口吞掉）
    esp_err_t e1 = esp_netif_init();
    esp_err_t e2 = esp_event_loop_create_default();
    if (e2 == ESP_ERR_INVALID_STATE) e2 = ESP_OK; // 已存在视为正常
    esp_netif_t* apn = esp_netif_create_default_wifi_ap();
    esp_err_t eA = apn ? ESP_OK : ESP_FAIL;

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t e3 = esp_wifi_init(&ic);
    esp_err_t e4 = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEvt, NULL);

    wifi_config_t wc = {};
    strncpy((char*)wc.ap.ssid, WIFI_SSID, sizeof(wc.ap.ssid));
    strncpy((char*)wc.ap.password, WIFI_PASS, sizeof(wc.ap.password));
    wc.ap.channel = WIFI_CHANNEL;
    wc.ap.max_connection = WIFI_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    esp_err_t e5 = esp_wifi_set_mode(WIFI_MODE_AP);
    esp_err_t e6 = esp_wifi_set_config(WIFI_IF_AP, &wc);
    esp_err_t e7 = esp_wifi_start();

    wifi_config_t cur = {};
    esp_err_t e8 = esp_wifi_get_config(WIFI_IF_AP, &cur);
    esp_netif_ip_info_t ip;
    esp_err_t e9 = ESP_FAIL;
    if (apn && esp_netif_get_ip_info(apn, &ip) == ESP_OK) e9 = ESP_OK;

    if (e1 != ESP_OK || e2 != ESP_OK || eA != ESP_OK || e3 != ESP_OK ||
        e4 != ESP_OK || e5 != ESP_OK || e6 != ESP_OK || e7 != ESP_OK) {
        snprintf(s_cfgStr, sizeof(s_cfgStr), "ERR %d/%d/%d/%d/%d/%d/%d/%d",
                 e1, e2, eA, e3, e4, e5, e6, e7);
        printf("[WiFi] init FAILED: %s\n", s_cfgStr);
        return;
    }
    char ib[16] = "-";
    if (e9 == ESP_OK) snprintf(ib, sizeof(ib), IPSTR, IP2STR(&ip.ip));
    snprintf(s_cfgStr, sizeof(s_cfgStr), "ok %s/%s%s",
             (e8 == ESP_OK) ? (const char*)cur.ap.ssid : "?",
             ib, (e8 != ESP_OK) ? " cfgE" : (e9 != ESP_OK) ? " ipE" : "");
    printf("[WiFi] AP up: %s\n", s_cfgStr);
    s_wifiUp = true;
    s_startUs = esp_timer_get_time();
    s_lastHttpUs = s_startUs;

    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.max_uri_handlers = 10; // 5 现有 + 2 OTA + 余量
    hc.stack_size = 8192;     // OTA 升级 handler 需较大栈
    if (httpd_start(&s_httpd, &hc) == ESP_OK) {
        static const httpd_uri_t u_root  = {.uri="/",          .method=HTTP_GET, .handler=handler_root,  .user_ctx=NULL};
        static const httpd_uri_t u_state = {.uri="/api/state", .method=HTTP_GET, .handler=handler_state, .user_ctx=NULL};
        static const httpd_uri_t u_set   = {.uri="/api/set",   .method=HTTP_GET, .handler=handler_set,   .user_ctx=NULL};
        static const httpd_uri_t u_off   = {.uri="/api/wifi/off", .method=HTTP_GET, .handler=handler_off, .user_ctx=NULL};
        static const httpd_uri_t u_capt  = {.uri="/*",         .method=HTTP_GET, .handler=handler_captive, .user_ctx=NULL};
        httpd_register_uri_handler(s_httpd, &u_root);
        httpd_register_uri_handler(s_httpd, &u_state);
        httpd_register_uri_handler(s_httpd, &u_set);
        httpd_register_uri_handler(s_httpd, &u_off);
        httpd_register_uri_handler(s_httpd, &u_capt); // 通配放最后：精确路由优先
        otaHttpdRegister(s_httpd);                    // /ota 上传页 + /api/ota
        printf("[WiFi] 热点 RGBled 已开启（密码 12345678），5 分钟无连接自动关闭\n");
    } else {
        printf("[WiFi] httpd FAILED\n");
        s_httpd = NULL;
    }
    xTaskCreate(dnsTask, "dns", 4096, NULL, 5, NULL); // 常驻：热点关闭后无人查询，无害
}

// ---------- 状态查询 ----------
int wifiCfgState() {
    if (!s_wifiUp) return 0;
    return s_everConnected ? 2 : 1;
}

const char* wifiCfgStr() {
    return s_cfgStr;
}

// ---------- 每秒判定关闭 ----------
void wifiCfgPoll(uint32_t nowMs) {
    (void)nowMs;
    if (!s_wifiUp || s_stopping) return;

    if (s_offRequested) { // 网页点击"关闭 WiFi"
        wifiStopAll();
        return;
    }
    uint64_t nowUs = esp_timer_get_time();
    uint64_t sinceHttp = (nowUs - s_lastHttpUs) / 1000;
    uint64_t sinceStart = (nowUs - s_startUs) / 1000;

    if (!s_everConnected) {
        // 从未有手机连接：上电 30s 后关闭
        if (sinceStart > OFF_NO_STA_MS) wifiStopAll();
    } else {
        // 曾连接（保持期）：关闭网页/无操作 60s 后关闭
        if (sinceHttp > OFF_NO_HTTP_MS) wifiStopAll();
    }
}
