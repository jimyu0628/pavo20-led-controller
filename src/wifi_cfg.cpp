#include "wifi_cfg.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_timer.h"

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
#define WIFI_CHANNEL    6
#define WIFI_MAX_CONN   4
#define OFF_NO_HTTP_MS  60000   // 关闭网页（无 HTTP 请求）60s 后关
#define OFF_NO_STA_MS   30000   // 上电后 30s 无手机连接则关

static httpd_handle_t s_httpd = NULL;
static volatile bool  s_wifiUp = false;
static volatile bool  s_everConnected = false; // 是否曾有手机连上（连上后进入保持期）
static volatile bool  s_offRequested = false;  // 网页请求关闭
static volatile bool  s_stopping = false;
static uint64_t s_startUs = 0;   // AP 开启时刻（µs）
static volatile uint64_t s_lastHttpUs = 0; // 最后 HTTP 请求时刻（µs）

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
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"mode\":%d,\"color\":%d,\"speed\":%d,\"bright\":%d,\"calib\":%d,\"cal\":%d}",
             webGetMode(), webGetColor(), webGetSpeed(), webGetBright(),
             webGetCalib(), webIsCalibrating() ? 1 : 0);
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

<div class="card">
 <button id="btnOff" class="danger" onclick="offWifi()">关闭 WiFi（需重启才能重新开启）</button>
</div>
<div id="toast"></div>

<script>
var M=["飞控(Betaflight)","单色常亮","单色呼吸","单色跑马","双边跑马","单色导向","单色增长","彩虹呼吸","彩虹渐变","双边彩虹"];
var C=["白","红","黄","蓝","绿","青","品红","关"];
var CS=["#fff","#f33","#ff3","#36f","#3f3","#0ff","#f3f","#000"];
var S=["快","中","慢"];
var B=["100%","75%","50%","25%"];
var st={mode:0,color:0,speed:1,bright:2,calib:55,cal:0};

function render(){
 document.getElementById("st").textContent =
  "模式："+M[st.mode]+" · 颜色："+C[st.color]+" · 速度："+S[st.speed]+
  " · 亮度："+B[st.bright]+" · 灯数："+st.calib+(st.cal?" · 标定中":"");
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
    printf("[WiFi] 热点已关闭\n");
}

// ---------- 初始化 ----------
void wifiCfgInit() {
    // IDF 初始化顺序（缺一不可，否则 create_default_wifi_ap 报 INVALID_STATE）：
    // 1) esp_netif_init() 网络接口层
    // 2) esp_event_loop_create_default() 默认事件循环（WiFi 事件注册必需）
    // 3) esp_netif_create_default_wifi_ap() 创建 AP 接口
    // 4) esp_wifi_init() 初始化 WiFi
    esp_netif_init();
    esp_err_t el = esp_event_loop_create_default();
    if (el != ESP_OK && el != ESP_ERR_INVALID_STATE) { // 已存在则忽略
        printf("[WiFi] event loop FAILED 0x%x\n", el);
        return;
    }
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&ic) != ESP_OK) {
        printf("[WiFi] init FAILED\n");
        return;
    }
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEvt, NULL);

    wifi_config_t wc = {};
    strncpy((char*)wc.ap.ssid, WIFI_SSID, sizeof(wc.ap.ssid));
    strncpy((char*)wc.ap.password, WIFI_PASS, sizeof(wc.ap.password));
    wc.ap.channel = WIFI_CHANNEL;
    wc.ap.max_connection = WIFI_MAX_CONN;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK ||
        esp_wifi_set_config(WIFI_IF_AP, &wc) != ESP_OK ||
        esp_wifi_start() != ESP_OK) {
        printf("[WiFi] start FAILED\n");
        return;
    }
    s_wifiUp = true;
    s_startUs = esp_timer_get_time();
    s_lastHttpUs = s_startUs;

    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.max_uri_handlers = 8;
    if (httpd_start(&s_httpd, &hc) == ESP_OK) {
        static const httpd_uri_t u_root  = {.uri="/",          .method=HTTP_GET, .handler=handler_root,  .user_ctx=NULL};
        static const httpd_uri_t u_state = {.uri="/api/state", .method=HTTP_GET, .handler=handler_state, .user_ctx=NULL};
        static const httpd_uri_t u_set   = {.uri="/api/set",   .method=HTTP_GET, .handler=handler_set,   .user_ctx=NULL};
        static const httpd_uri_t u_off   = {.uri="/api/wifi/off", .method=HTTP_GET, .handler=handler_off, .user_ctx=NULL};
        httpd_register_uri_handler(s_httpd, &u_root);
        httpd_register_uri_handler(s_httpd, &u_state);
        httpd_register_uri_handler(s_httpd, &u_set);
        httpd_register_uri_handler(s_httpd, &u_off);
        printf("[WiFi] 热点 RGBled 已开启（密码 12345678），30s 无连接自动关闭\n");
    } else {
        printf("[WiFi] httpd FAILED\n");
        s_httpd = NULL;
    }
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
