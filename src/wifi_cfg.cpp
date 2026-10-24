#include "wifi_cfg.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_system.h"   // esp_restart
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ota.h"
#include "ble_uart.h"

// ============================================================
// WiFi 热点 + Web 设置（AP 模式，ESP32-C3）
//
// 关闭逻辑（wifiCfgPoll 每秒判定，与用户需求一致）：
//   1. 网页点击"关闭 WiFi" → 立即关闭
//   2. 已开启且从未有手机连接 → 上电 30s 后关闭
//   3. 曾连接过（保持期）→ 距最后 HTTP 请求 >60s（关闭网页/无操作）→ 关闭
// 备注：关闭后需重启设备才能重新开启热点
// ============================================================

#define WIFI_SSID       "RGBled-32"   // v32.15：临时换名，让手机当成「全新网络」重新做联网探测（见下）
#define WIFI_PASS       "12345678"
#define WIFI_CHANNEL    1
#define WIFI_MAX_CONN   4

// 全局状态（需在 dnsTask / sendState 等使用处之前定义）
static char s_bootInfo[512] = "n/a";     // 开机信息（Web 调试区显示）
static volatile int s_dnsOk = 0;         // DNS 劫持是否绑定成功
// v32.14：自动关闭策略大改。以前的「5 分钟无访问就关」太短——插线、找热点、翻网页
// 很容易超过 5 分钟，用户看到的就是「热点莫名其妙没了」。
// 新规则：只要有设备连在热点上就永不自动关；全部断开后才开始计时。
#define OFF_NO_HTTP_MS 1800000  // 保活期内、全部设备断开且 30 分钟无 HTTP 访问才关（原 5 分钟）
#define OFF_NO_STA_MS   900000  // 上电后 15 分钟仍没有任何设备连过则关（原 5 分钟）

static httpd_handle_t s_httpd = NULL;
static volatile bool  s_wifiUp = false;
static volatile bool  s_everConnected = false; // 是否曾有手机连上（连上后进入保持期）
static volatile bool  s_offRequested = false;  // 网页请求关闭
static volatile bool  s_stopping = false;
static volatile int   s_staCount = 0;       // v32.14：当前连在热点上的设备数（每秒刷新）
static volatile uint32_t s_dnsQ = 0;        // v32.15：累计收到的 DNS 查询数（排查劫持是否真被用到）
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

// v32.15：强制让软 AP 通过 DHCP 把「DNS 服务器 = 本机 192.168.4.1」下发给手机。
// 这是 captive portal 最容易翻车的一环：手机若拿到外部 DNS，就直接向外解析域名，
// 永远不会来敲我们的 53 端口 —— 表现就是「心跳 dns:1（劫持在跑）、手机既不弹窗、
// 手动打开也白屏、[CP] 一条日志都没有」。
//
// 注意 IDF 4.4 的坑：`ESP_NETIF_DOMAIN_NAME_SERVER` 在 dhcps 侧**不是地址**，
// 而是一个 uint8 布尔开关（OFFER_DNS，"要不要下发 DNS"）。地址必须走
// `esp_netif_set_dns_info(ESP_NETIF_DNS_MAIN)`；两者都要在 dhcps 停止状态下改。
static esp_err_t forceDnsToSelf(esp_netif_t* apn) {
    if (!apn) return ESP_FAIL;
    esp_netif_ip_info_t ip = {};
    esp_err_t e = esp_netif_get_ip_info(apn, &ip);
    if (e != ESP_OK) return e;

    esp_netif_dhcps_stop(apn); // 改选项/地址前必须先停 dhcps，否则返回 ALREADY_STARTED

    uint8_t offer = 1; // OFFER_DNS：允许下发 DNS 服务器
    e = esp_netif_dhcps_option(apn, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER,
                               &offer, sizeof(offer));

    esp_netif_dns_info_t dns = {};
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    dns.ip.u_addr.ip4.addr = ip.ip.addr;
    esp_err_t e2 = esp_netif_set_dns_info(apn, ESP_NETIF_DNS_MAIN, &dns);

    esp_netif_dhcps_start(apn);
    return (e == ESP_OK && e2 == ESP_OK) ? ESP_OK : (e != ESP_OK ? e : e2);
}

// v32.14：查询当前连在热点上的设备数（>0 = 手机在线，绝不自动关热点）
static int staOnline() {
    wifi_sta_list_t lst;
    if (esp_wifi_ap_get_sta_list(&lst) != ESP_OK) return 0;
    return (int)lst.num;
}

// ---------- HTTP：JSON 状态 ----------
static void touchHttp() { s_lastHttpUs = esp_timer_get_time(); }

static void sendState(httpd_req_t* req) {
    // v32.5：boot 最多 512 字节，原 640 缓冲放全套字段会截断 JSON（网页白屏），放宽到 1024
    char buf[1024];
    snprintf(buf, sizeof(buf),
             "{\"mode\":%d,\"color\":%d,\"speed\":%d,\"bright\":%d,\"calib\":%d,\"cal\":%d,\"v\":\"%s\",\"ws\":\"%s\",\"ble\":\"%s\",\"bdiag\":\"%s\",\"dns\":%d,\"boot\":\"%s\"}",
             webGetMode(), webGetColor(), webGetSpeed(), webGetBright(),
             webGetCalib(), webIsCalibrating() ? 1 : 0, otaVersion(), wifiCfgStr(),
             bleUartState(), bleUartDiag(),
             wifiCfgDnsOk(), wifiCfgBootInfo());
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

// 软件重启：板载 RESET 键失效时的替代手段（实测该 C3 小板按 RST 不复位）
static void rebootTask(void* arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(300)); // 先让 HTTP 响应发出去
    printf("[SYS] /api/reboot → esp_restart()\n");
    fflush(stdout);
    esp_restart();
}

static esp_err_t handler_reboot(httpd_req_t* req) {
    touchHttp();
    httpd_resp_set_hdr(req, "Content-Type", "application/json; charset=utf-8");
    httpd_resp_sendstr(req, "{\"ok\":1,\"reboot\":1}");
    xTaskCreate(rebootTask, "reboot", 2048, NULL, 6, NULL);
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
<h1>幻彩灯带控制板 <span id="ver" style="font-size:13px;color:#888">v32</span></h1>
<div class="card" style="border:2px solid #2d7ff9;background:#eef4ff">
 <h3 style="margin-top:0;color:#15427f">飞控调参</h3>
 <a href="https://app.betaflight.com" target="_blank" rel="noopener"
    style="display:block;text-align:center;padding:16px 12px;background:#2d7ff9;color:#fff;border-radius:12px;font-size:18px;font-weight:700;text-decoration:none;box-shadow:0 4px 10px rgba(45,127,249,.35)">
  打开 Betaflight 调参 &rarr;
 </a>
 <div style="font-size:12px;color:#555;margin-top:8px;line-height:1.7">
  ① 首次使用请先在有网时打开一次，并在地址栏右侧点「安装」装成应用，之后连着本热点也能离线用；<br>
  ② 蓝牙方式：系统蓝牙里连 <b>PavoLED</b>，再到页面里选蓝牙设备；<br>
  ③ 电脑装桌面版的话，端口选 <code>Manual Selection</code>，填 <code>tcp://192.168.4.1</code>。
 </div>
</div>
<div class="sub">WiFi 热点：RGBled · 密码 12345678 · 无操作 60 秒后自动关闭</div>
<div id="st">加载中…</div>
<label style="display:block;margin:6px 0;font-size:13px;color:#9be">
 <input type="checkbox" id="dbgChk" onchange="toggleDbg()" style="width:20px;height:20px;vertical-align:middle;margin-right:6px">调试信息
</label>
<div id="dbg" style="display:none;background:#0c1a12;color:#8f9;font-family:monospace;font-size:12px;padding:8px 12px;border-radius:8px;margin:6px 0;white-space:pre-wrap;line-height:1.8"></div>

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
var st={mode:0,color:0,speed:1,bright:2,calib:55,cal:0,v:"v32",ws:"",ble:"",bdiag:"",dns:0,boot:""};

function render(){
 var ve=document.getElementById("ver"); if(ve)ve.textContent=st.v;
 document.getElementById("st").textContent =
  "模式："+M[st.mode]+" · 颜色："+C[st.color]+" · 速度："+S[st.speed]+
  " · 亮度："+B[st.bright]+" · 灯数："+st.calib+(st.cal?" · 标定中":"");
 dbgRefresh();
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
function toggleDbg(){var d=document.getElementById("dbg");
 if(document.getElementById("dbgChk").checked){d.style.display="block";dbgRefresh();}else d.style.display="none";}
function dbgRefresh(){var d=document.getElementById("dbg");
 if(!d||d.style.display=="none")return;
 d.textContent="固件版本 : "+(st.v||"n/a")+
  "\nWiFi 状态 : "+(st.ws||"n/a")+
  "\nDNS 劫持 : "+(st.dns?"ok":"fail/未启动")+
  "\nBLE 状态 : "+(st.ble||"n/a")+
  "\nBLE 透传 : "+(st.bdiag||"n/a")+"   （sub=已订阅 mtu=协商值 nf=通知成功/失败 probe=飞控探测）"+
  "\n开机信息 : "+(st.boot||"n/a")+
  "\n模式/颜色/速度/亮度 : "+st.mode+"/"+st.color+"/"+st.speed+"/"+st.bright+
  "\nLED 数量 : "+st.calib+" 颗"+(st.cal?"（标定中）":"")+
  "\n心跳 : "+JSON.stringify(st);}
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
    int sock = -1;
    // v32.12：dhcps 起来前后可能短暂占端口，重试几次再放弃（原来一次失败就退出，
    // 于是「DNS 劫持」一直显示 fail，手机连上热点永远不弹页面）
    for (int i = 0; i < 20; i++) {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock >= 0) {
            struct sockaddr_in addr = {};
            addr.sin_family = AF_INET;
            addr.sin_port = htons(53);
            addr.sin_addr.s_addr = INADDR_ANY;
            if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == 0) break;
            close(sock);
            sock = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (sock < 0) {
        printf("[WiFi] dns bind fail errno=%d\n", errno);
        s_dnsOk = 0;
        vTaskDelete(NULL);
        return;
    }
    s_dnsOk = 1;
    printf("[WiFi] DNS captive: 53 -> 192.168.4.1\n");
    char buf[512];
    uint8_t out[600];
    for (;;) {
        struct sockaddr_in from = {};
        socklen_t fl = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr*)&from, &fl);
        if (n < 12) continue;
        s_dnsQ++;                                     // v32.15：统计收到的 DNS 查询数（q=）
        // v32.12：按 QTYPE 分开应答——A 查询给 192.168.4.1，AAAA/其它回空(NOERROR)。
        // 原来不分类型一律塞 A 记录，手机解析 AAAA 时对不上，网络探测反而失败。
        int pos = 12;
        while (pos < n && buf[pos] != 0) {
            if (((uint8_t)buf[pos] & 0xC0) == 0xC0) { pos += 2; break; }
            pos += (uint8_t)buf[pos] + 1;
        }
        if (pos < n && buf[pos] == 0) pos += 1;
        if (pos + 4 > n) continue;
        uint16_t qtype = (uint16_t)(((uint8_t)buf[pos]) << 8 | (uint8_t)buf[pos + 1]);
        pos += 4;                                     // 跳过 QTYPE + QCLASS
        int r = pos;
        memcpy(out, buf, r);                          // 问题段原样回显
        out[2] = 0x81; out[3] = 0x80;                 // QR=1 RD=1 RA=1 RCODE=0
        int an = 0;
        if (qtype == 1 && r + 16 <= (int)sizeof(out)) { // 只对 A 查询给地址
            out[r++] = 0xC0; out[r++] = 0x0C;         // 指针 → 问题名（offset 12）
            out[r++] = 0x00; out[r++] = 0x01;         // TYPE A
            out[r++] = 0x00; out[r++] = 0x01;         // CLASS IN
            out[r++] = 0x00; out[r++] = 0x00; out[r++] = 0x00; out[r++] = 30; // TTL 30s
            out[r++] = 0x00; out[r++] = 0x04;         // RDLENGTH=4
            out[r++] = 192; out[r++] = 168; out[r++] = 4; out[r++] = 1;
            an = 1;
        }
        out[6] = 0; out[7] = (uint8_t)an;             // ANCOUNT
        out[8] = 0; out[9] = 0;                       // NSCOUNT
        out[10] = 0; out[11] = 0;                     // ARCOUNT
        sendto(sock, out, r, 0, (struct sockaddr*)&from, fl);
    }
}

// ========== Captive Portal：HTTP 重定向（探测路径 → 设置页首页） ==========
static esp_err_t handler_captive(httpd_req_t* req) {
    if (strncmp(req->uri, "/api/", 5) == 0) { // 不劫持 API，避免干扰网页轮询
        httpd_resp_send_404(req);
        return ESP_OK;
    }
    touchHttp();
    // v32.14：把探测请求打印出来——自动弹窗不生效时，先看清手机的探测到底打到哪条路径
    // （安卓 generate_204 / 苹果 hotspot-detect.html / 小米 connect.rom.miui.com / 华为 …）
    {
        char host[64] = "-";
        if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK) {
            host[0] = '-';
            host[1] = 0;
        }
        printf("[CP] %s%s -> 302\n", host, req->uri);
    }
    // 绝对地址重定向：手机探测逻辑按完整 URL 判断是否 captive（相对地址部分机型不弹窗）。
    // v32.12：加 no-store，避免探测结果被缓存后第二次连热点不再弹窗。
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<html><meta http-equiv=\"refresh\" content=\"0;url=http://192.168.4.1/\">"
        "<body style=\"font-family:sans-serif;text-align:center;padding-top:40px\">"
        "<p>正在进入灯带控制板…</p><p><a href=\"http://192.168.4.1/\">点这里</a></p>"
        "</body></html>");
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
    // v32.15：AP 起来后立刻把 DHCP 下发的 DNS 改成热点自己（详见 forceDnsToSelf 注释）
    {
        esp_err_t ed = forceDnsToSelf(apn);
        printf("[WiFi] 强制 DHCP DNS=192.168.4.1：%s (0x%x)\n", ed == ESP_OK ? "ok" : "FAIL", ed);
    }

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
    hc.stack_size = 4096;     // OTA handler 大缓冲均为 static，不占任务栈（省内存给 BLE）
    if (httpd_start(&s_httpd, &hc) == ESP_OK) {
        static const httpd_uri_t u_root  = {.uri="/",          .method=HTTP_GET, .handler=handler_root,  .user_ctx=NULL};
        static const httpd_uri_t u_state = {.uri="/api/state", .method=HTTP_GET, .handler=handler_state, .user_ctx=NULL};
        static const httpd_uri_t u_set   = {.uri="/api/set",   .method=HTTP_GET, .handler=handler_set,   .user_ctx=NULL};
        static const httpd_uri_t u_off   = {.uri="/api/wifi/off", .method=HTTP_GET, .handler=handler_off, .user_ctx=NULL};
        static const httpd_uri_t u_rb    = {.uri="/api/reboot",   .method=HTTP_GET, .handler=handler_reboot, .user_ctx=NULL};
        static const httpd_uri_t u_capt  = {.uri="/*",         .method=HTTP_GET, .handler=handler_captive, .user_ctx=NULL};
        httpd_register_uri_handler(s_httpd, &u_root);
        httpd_register_uri_handler(s_httpd, &u_state);
        httpd_register_uri_handler(s_httpd, &u_set);
        httpd_register_uri_handler(s_httpd, &u_off);
        httpd_register_uri_handler(s_httpd, &u_rb);
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
void wifiCfgSetBootInfo(const char* s) {
    strncpy(s_bootInfo, s, sizeof(s_bootInfo) - 1);
    s_bootInfo[sizeof(s_bootInfo) - 1] = 0;
}
const char* wifiCfgBootInfo() { return s_bootInfo; }
int wifiCfgDnsOk() { return s_dnsOk; }

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

    // v32.14：只要还有设备连在热点上，就绝不自动关（并持续刷新两个计时器）
    int n = staOnline();
    s_staCount = n;
    if (n > 0) {
        s_everConnected = true;
        s_lastHttpUs = nowUs;
        s_startUs    = nowUs;
        return;
    }

    uint64_t sinceHttp = (nowUs - s_lastHttpUs) / 1000;
    uint64_t sinceStart = (nowUs - s_startUs) / 1000;

    if (!s_everConnected) {
        // 从未有手机连接：上电 15 分钟后仍没人来，关掉省电
        if (sinceStart > OFF_NO_STA_MS) wifiStopAll();
    } else {
        // 曾连接（保持期）：全部设备断开后 30 分钟无访问才关
        if (sinceHttp > OFF_NO_HTTP_MS) wifiStopAll();
    }
}

// v32.14：热点诊断串 "sta=N off=Ms"（off<0 表示有人在线、不会自动关；单为秒）
const char* wifiCfgDiag() {
    static char d[56];
    if (!s_wifiUp) {
        snprintf(d, sizeof(d), "sta=0 off=- dq=%u", (unsigned)s_dnsQ);
        return d;
    }
    int n = s_staCount;
    int left = -1;
    if (n == 0) {
        uint64_t nowUs = esp_timer_get_time();
        uint64_t limit = s_everConnected ? OFF_NO_HTTP_MS : OFF_NO_STA_MS;
        uint64_t used  = s_everConnected ? (nowUs - s_lastHttpUs) / 1000
                                         : (nowUs - s_startUs) / 1000;
        left = (used >= limit) ? 0 : (int)((limit - used) / 1000);
    }
    snprintf(d, sizeof(d), "sta=%d off=%d dq=%u", n, left, (unsigned)s_dnsQ);
    return d;
}
