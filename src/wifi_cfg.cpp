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
#include "device_name.h"   // DEV_NAME：热点 SSID 与 BLE 名同源
#include "web_page.h"     // v32.21：内嵌网页（A+ 三功能区界面）
#include "config.h"        // STRIP_MAX_LEDS（网页里「灯带长度」的上限）
#include "fc_input.h"      // 网页调试区显示飞控帧计数
#include "msp_tcp.h"       // v32.21：mspTcpDiag()，「连接」页显示飞控链路状态

// ============================================================
// WiFi 热点 + Web 设置（AP 模式，ESP32-C3）
//
// 关闭逻辑（wifiCfgPoll 每秒判定，与用户需求一致）：
//   1. 网页点击"关闭 WiFi" → 立即关闭
//   2. 已开启且从未有手机连接 → 上电 30s 后关闭
//   3. 曾连接过（保持期）→ 距最后 HTTP 请求 >60s（关闭网页/无操作）→ 关闭
// 备注：关闭后需重启设备才能重新开启热点
// ============================================================

#define WIFI_SSID       DEV_NAME      // v32.18：与 BLE 广播名同源（device_name.h），不再各写各的
#define WIFI_PASS       "12345678"
#define WIFI_CHANNEL    1
#define WIFI_MAX_CONN   4

// 全局状态（需在 dnsTask / sendState 等使用处之前定义）
static char s_bootInfo[512] = "n/a";     // 开机信息（Web 调试区显示）
static char s_ip[16] = "192.168.4.1";    // v32.21：网页「连接」页要显示，init 里取一次真实值
static volatile int s_dnsOk = 0;         // DNS 劫持是否绑定成功
// v32.14：自动关闭策略大改。以前的「5 分钟无访问就关」太短——插线、找热点、翻网页
// 很容易超过 5 分钟，用户看到的就是「热点莫名其妙没了」。
// 新规则：只要有设备连在热点上就永不自动关；全部断开后才开始计时。
// v32.20：改成「一分钟不用就关」——用户要的是开机后没人来调参就彻底断电省电，
// 不是挂在那儿等。判定口径见 wifiCfgPoll：没有手机连热点、没有 HTTP 请求、
// 没有蓝牙连接，三者同时安静满 60 秒才动手。
#define IDLE_OFF_MS     60000   // 无线闲置阈值：60 秒
#define OFF_NO_HTTP_MS 1800000 // 保留给「诊断串」显示剩余时间的老口径，逻辑上已不参与判定
#define OFF_NO_STA_MS   1800000

static httpd_handle_t s_httpd = NULL;
static volatile bool  s_wifiUp = false;
static volatile bool  s_everConnected = false; // 是否曾有手机连上（连上后进入保持期）
static volatile int   s_staCount = 0;       // v32.14：当前连在热点上的设备数（每秒刷新）
static volatile uint32_t s_dnsQ = 0;        // v32.15：累计收到的 DNS 查询数（排查劫持是否真被用到）
static uint64_t s_startUs = 0;   // AP 开启时刻（µs）
static volatile uint64_t s_lastHttpUs = 0; // 最后 HTTP 请求时刻（µs）
static volatile uint64_t s_lastActUs  = 0; // v32.20：最后一次「有人在用无线」的时刻（µs）
static char s_cfgStr[64];                  // 启动时缓存 "SSID/IP" 供心跳显示（SSID 最长 32 + IP 15 + 前缀后缀，64 防截断警告）

// ---------- 内部 ----------
static esp_err_t wifiStopAll(); // v32.21：返回错误码，关闭失败要能重试

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
static void touchHttp() {
    uint64_t t = esp_timer_get_time();
    s_lastHttpUs = t;
    s_lastActUs  = t; // v32.20：网页有请求就算「在用」，闲置计时归零
}

// v32.21：新界面（灯光 / 连接 / 设备 三页）一次要把三页的数值都拿到，
// 所以 /api/state 一条就全给齐，页面每秒轮询这一次就够，不再另开接口。
//   mode/color/spd/bright/count —— 当前生效的设置（spd 1~100，bright 0~100）
//   maxc —— 灯带缓冲上限（页面据此画「灯带长度」轨道，改 config.h 页面自动跟）
//   ssid/ip/sta/idle —— 热点信息，idle = 距自动关闭的秒数（-1 表示有人在用、不会关）
//   v/bld/up/rs —— 版本、构建时间、已运行秒数、复位原因
//   ble/dns/fc/msp/bdiag/boot —— 调试区与「连接」页用
static void sendState(httpd_req_t* req) {
    // boot 字符串本身约 170 字节，加上其余字段整条 500 上下，1400 留足余量
    char buf[1500];   // v32.32：加了 bleen/capd 两个字段，留足余量
    // v32.27：把最近一帧前 4 颗灯的「线路原始字节」也吐给网页（形如 00FF00 00FF00 …）。
    // 飞控设成纯红时：原始字节里哪个位置是 FF，就说明字节序是哪个 —— 排颜色问题时
    // 不用猜、也不用插 USB 看串口，网页日志里直接能对。
    char head[48];
    fcRxHeadHex(head, sizeof(head));
    snprintf(buf, sizeof(buf),
             "{\"mode\":%d,\"color\":%d,\"spd\":%d,\"bright\":%d,\"count\":%d,\"maxc\":%d,"
             "\"cal\":%d,\"ssid\":\"%s\",\"ip\":\"%s\",\"sta\":%d,\"idle\":%d,"
             "\"v\":\"%s\",\"bld\":\"%s %s\",\"up\":%lu,\"rs\":%d,"
             "\"ble\":\"%s\",\"dns\":%d,\"fc\":%lu,\"msp\":\"%s\",\"bdiag\":\"%s\",\"boot\":\"%s\","
             "\"ford\":%d,\"fordn\":\"%s\",\"lend\":%u,\"head\":\"%s\","
             "\"bad\":%u,\"frag\":%u,\"clamp\":%u,\"ghost\":%u,\"mute\":%u,\"hmin\":%u,\"hmax\":%u,\"lomax\":%u,\"thr\":%u,"
             "\"dropb\":%u,\"cap\":%u,\"ratio\":%d,\"need\":%d,\"bleen\":%d,\"capd\":%d,"
             // v32.29：健康诊断
             "\"tx\":%d,\"txf\":%d,\"lpm\":%d,\"sil\":%d,\"rxs\":%d,"
             "\"bits\":%u,\"mis\":%u,\"spk\":%u,\"mrg\":%u,\"near\":%u,\"per\":%u}",
             webGetMode(), webGetColor(), webGetSpeed(), webGetBright(),
             webGetCalib(), STRIP_MAX_LEDS,
             webIsCalibrating() ? 1 : 0,
             DEV_NAME, s_ip, s_staCount, wifiCfgIdleLeft(),
             otaVersion(), __DATE__, __TIME__,
             (unsigned long)(esp_timer_get_time() / 1000000), (int)esp_reset_reason(),
             bleUartState(), wifiCfgDnsOk(), (unsigned long)g_rxFrames,
             mspTcpDiag(), bleUartDiag(), wifiCfgBootInfo(),
             webGetFcOrder(), fcRxOrderName(), (unsigned)g_rxLedAtEnd, head,
             (unsigned)g_rxBad, (unsigned)g_rxFrag, (unsigned)g_rxClamp,
             (unsigned)g_rxGhost, (unsigned)g_rxMute,
             (unsigned)g_rxHiMinNs, (unsigned)g_rxHiMaxNs,
             (unsigned)g_rxLoMaxNs, (unsigned)g_rxThrNs,
             (unsigned)g_rxDropBits, (unsigned)g_rxCapUs,
             webGetRatio(), webGetFcNeed(),
             webGetBleAdv(), webGetCapDrop(),
             webGetTxRate(), webGetTxFail(), webGetLoopGap(),
             webGetFcSilent(), webGetRxReset(),
             (unsigned)g_rxBits, (unsigned)g_rxMisalign,
             (unsigned)g_rxSpike, (unsigned)g_rxMerge,
             (unsigned)g_rxNear, (unsigned)g_rxPeriodNs);
    httpd_resp_set_hdr(req, "Content-Type", "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_sendstr(req, buf);
}

// ---------- HTTP：设置 ----------
// v32.21：网页「应用」按钮一次性把五项参数全发过来：
//   POST /api/set   body: mode=5&color=2&spd=55&bright=80&count=55
// 用 form-urlencoded 而不是 JSON —— httpd_query_key_value 直接就能解析这种串，
// 省掉一个 JSON 解析器（这板子 RAM 还要留给 BLE）。
// 老的 GET /api/set?mode=3 也还认，方便用 curl 单个调。
static esp_err_t handler_set(httpd_req_t* req) {
    touchHttp();
    char q[192];
    if (req->method == HTTP_POST) {
        int len = (int)req->content_len;
        int cap = (int)sizeof(q) - 1;
        int got = 0;
        if (len <= 0) { q[0] = 0; len = 0; }
        while (got < len) {
            int off  = (got < cap) ? got : 0;  // 超长部分丢掉，只留前 cap 字节
            int want = (got < cap) ? (cap - got) : cap;
            if (want > len - got) want = len - got;
            int n = httpd_req_recv(req, q + off, want);
            if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;   // 网络慢，接着收
            if (n <= 0) break;                            // 出错/断线
            got += n;
        }
        q[(got < cap) ? got : cap] = 0;
    } else {
        if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) q[0] = 0;
    }

    if (q[0]) {
        char v[8];
        if (httpd_query_key_value(q, "mode",   v, sizeof(v)) == ESP_OK) webSetMode(atoi(v));
        if (httpd_query_key_value(q, "color",  v, sizeof(v)) == ESP_OK) webSetColor(atoi(v));
        if (httpd_query_key_value(q, "spd",    v, sizeof(v)) == ESP_OK) webSetSpeed(atoi(v));
        if (httpd_query_key_value(q, "bright", v, sizeof(v)) == ESP_OK) webSetBright(atoi(v));
        if (httpd_query_key_value(q, "count",  v, sizeof(v)) == ESP_OK) webSetCount(atoi(v));
        if (httpd_query_key_value(q, "ford",   v, sizeof(v)) == ESP_OK) webSetFcOrder(atoi(v));
        if (httpd_query_key_value(q, "bleen",  v, sizeof(v)) == ESP_OK) webSetBleAdv(atoi(v)); // v32.32
    }
    webApplyNow(); // 立即生效（灭灯/渲染）
    sendState(req); // 回一整套最新状态，网页拿到就直接同步
    return ESP_OK;
}

// ---------- HTTP：状态 ----------
static esp_err_t handler_state(httpd_req_t* req) {
    touchHttp();
    sendState(req);
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

// ---------- HTTP：主页 ----------
// v32.21：页面搬到 src/web_page.h（A+ 界面：灯光 / 连接 / 设备 三功能区）。
// 页面里所有动态数值（设备名、SSID、倒计时、版本…）都从 /api/state 取，
// 所以这里不再做 @@DEVNAME@@ 文本替换，直接整块发出去即可。
static esp_err_t handler_root(httpd_req_t* req) {
    touchHttp();
    httpd_resp_set_hdr(req, "Content-Type", "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN);
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
        // v32.20：WiFi 关掉后这个 socket 会失效，recvfrom 立刻返回 -1。
        // 原来这里 `if (n < 12) continue;` 会变成死循环空转，把一个核吃满，
        // 灯带刷新直接掉帧、严重时触发看门狗。必须睡一下。
        if (n <= 0) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (n < 12) continue;
        s_dnsQ = s_dnsQ + 1;                          // v32.15：统计 DNS 查询数（volatile 用 ++ 会告警）
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
// v32.16：自己发 404，不能调 httpd_resp_send_404()——那会再进我们注册的 404 错误
// 处理器，处理器又调回来，死循环爆栈。
static esp_err_t sendPlain404(httpd_req_t* req) {
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, "not found");
}

static esp_err_t handler_captive(httpd_req_t* req) {
    if (strncmp(req->uri, "/api/", 5) == 0) { // 不劫持 API，避免干扰网页轮询
        return sendPlain404(req);
    }
    touchHttp();
    // v32.17：favicon 单独回 204。之前它也 302 回首页，于是每开一次页面就多一轮
    // 「favicon 302 → 首页 200」，日志里还跟着一串 error in recv : 104（客户端等不到
    // 图标直接 RST）。回空 204 最省事，浏览器不会再要。
    if (strcmp(req->uri, "/favicon.ico") == 0) {
        httpd_resp_set_status(req, "204 No Content");
        return httpd_resp_send(req, NULL, 0);
    }
    // v32.14：把探测请求打印出来——自动弹窗不生效时，先看清手机的探测到底打到哪条路径
    // （安卓 generate_204 / 苹果 hotspot-detect.html / 小米 connect.rom.miui.com / 华为 …）
    // v32.17：带上方法名。华为会 POST /mtuprobe，和 GET 的 generate204 分开看更清楚。
    {
        const char* m = (req->method == HTTP_GET)  ? "GET"
                      : (req->method == HTTP_POST) ? "POST" : "其他";
        char host[64] = "-";
        if (httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK) {
            host[0] = '-';
            host[1] = 0;
        }
        printf("[CP] %s %s%s -> 302\n", m, host, req->uri);
    }
    // 绝对地址重定向：手机探测逻辑按完整 URL 判断是否 captive（相对地址部分机型不弹窗）。
    // v32.12：加 no-store，避免探测结果被缓存后第二次连热点不再弹窗。
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    // v32.17：探测客户端拿到 302 就走了，不会复用这条连接；显式声明 close，
    // 免得 httpd 还挂着 socket 等下一个请求，把宝贵的 socket 槽位占满。
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<html><meta http-equiv=\"refresh\" content=\"0;url=http://192.168.4.1/\">"
        "<body style=\"font-family:sans-serif;text-align:center;padding-top:40px\">"
        "<p>正在进入灯带控制板…</p><p><a href=\"http://192.168.4.1/\">点这里</a></p>"
        "</body></html>");
    return ESP_OK;
}

// v32.16：兜底——POST/HEAD 探测、或任何没被通配接住的 URI，统一走错误处理器重定向。
// 手机探测用 GET 居多，但个别 ROM 会发 POST CFNetwork 探测，405 也要转成 302。
static esp_err_t handler_captive_err(httpd_req_t* req, httpd_err_code_t err) {
    (void)err;
    if (strncmp(req->uri, "/api/", 5) == 0) return sendPlain404(req); // 防 err→handler→err 递归
    return handler_captive(req);
}

// ---------- 关闭全部（WiFi + httpd + BLE） ----------
// v32.21：这一版把「点了关不掉 / 1 分钟也不自动关」彻底改掉。前几版有四个坑：
//   ① **顺序反了**：原来先 httpd_stop() 再 esp_wifi_stop()。httpd_stop() 会等所有
//      keep-alive 连接收尾，而网页每秒轮询 /api/state，于是它一直在等 → 卡死。
//      正确顺序是先 esp_wifi_stop()（所有 TCP 连接立刻断干净），再 httpd_stop()（秒回）。
//   ② **s_stopping 一旦置上就永不复位**，而 wifiCfgPoll 首行是 `if (s_stopping) return;`
//      —— 关闭一旦卡住，手动关不掉、闲置判定也全废。现在它只表示「正在进行」，
//      任务结束（成功或彻底失败）都复位；poll 里还有 20 秒看门狗兜底。
//   ③ 关闭动作原来放在临时任务里，xTaskCreate 失败就没戏。改成初始化时建一个
//      常驻的 pwrTask，靠任务通知唤醒，创建只做一次、永不失败。
//   ④ 每一步都不看返回码，失败也打印「已关闭」。现在三个返回码都打出来，失败重试 3 次。
static volatile bool s_stopping = false;   // 关闭流程进行中
static volatile bool s_pwrDone  = false;   // 已经关完了（之后不再判定）
static volatile uint32_t s_offAtMs = 0;    // 本轮关闭开始时刻（ms，卡住判定用）
static TaskHandle_t s_pwrTask = NULL;

// 真正动手：先停 WiFi → 停 httpd → 释放驱动。返回第一个非 OK 的错误码。
static esp_err_t wifiStopAll() {
    esp_err_t e1 = ESP_OK, e2 = ESP_OK, e3 = ESP_OK;

    // ① 先停 WiFi：连接立刻断，后面 httpd_stop 不会被 keep-alive 挂住（本轮修的核心）
    if (s_wifiUp) {
        e1 = esp_wifi_stop();
        if (e1 == ESP_ERR_WIFI_NOT_STARTED || e1 == ESP_ERR_WIFI_NOT_INIT) e1 = ESP_OK;
        s_wifiUp = false;
    }

    // ② 再停 httpd（此时没有活连接）
    if (s_httpd) {
        e2 = httpd_stop(s_httpd);
        s_httpd = NULL; // 无论成败都置空：stop 过一次就不能再调
    }

    // ③ 最后释放驱动（驱动没停成功就别 deinit，否则可能留个坏状态）
    if (e1 == ESP_OK) {
        e3 = esp_wifi_deinit();
        if (e3 == ESP_ERR_WIFI_NOT_INIT) e3 = ESP_OK;
    }

    s_cfgStr[0] = 0;
    printf("[WiFi] 关闭结果：wifi_stop=%d httpd_stop=%d wifi_deinit=%d\n",
           (int)e1, (int)e2, (int)e3);
    return (e1 != ESP_OK) ? e1 : (e2 != ESP_OK ? e2 : e3);
}

// 常驻关闭任务：等通知 → 关蓝牙 → 关 WiFi/httpd → 失败重试 → 复位标志
static void pwrTask(void* arg) {
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int attempt = 0;
        esp_err_t e = ESP_OK;
        for (;;) {
            bleUartStop();          // 先关蓝牙：独立协议栈，跟 WiFi 无关
            e = wifiStopAll();
            if (e == ESP_OK) break;
            if (++attempt >= 3) {
                printf("[PWR] 关闭失败（0x%x），已重试 %d 次，放弃\n", (unsigned)e, attempt);
                break;
            }
            printf("[PWR] 关闭失败（0x%x），3 秒后第 %d 次重试\n", (unsigned)e, attempt);
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        s_stopping = false;
        s_pwrDone  = true; // 不论成败都不再判定，免得每秒重复触发/刷日志
        printf("[PWR] 完成：热点已关、蓝牙已停，只剩灯带（断电重启可恢复）\n");
    }
}

// 请求关闭：只置标志 + 唤醒任务，耗时动作一律不在主循环里做
static void shutdownRequest(const char* why) {
    if (s_stopping || s_pwrDone) return;
    s_stopping = true;
    s_offAtMs  = (uint32_t)(esp_timer_get_time() / 1000);
    printf("[PWR] 关闭无线，原因：%s\n", why);
    if (!s_pwrTask) {
        // 极端兜底：任务没建起来（内存不足）。就地关，至少不会留个关不掉的热点。
        printf("[PWR] 关闭任务不存在，主循环内直接关\n");
        bleUartStop();
        wifiStopAll();
        s_stopping = false;
        s_pwrDone  = true;
        return;
    }
    xTaskNotifyGive(s_pwrTask);
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
    wc.ap.ssid_len = (uint8_t)strlen(WIFI_SSID); // v32.19：名字变成运行时字符串，长度显式给出更稳妥
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
    if (e9 == ESP_OK) {
        snprintf(ib, sizeof(ib), IPSTR, IP2STR(&ip.ip));
        strncpy(s_ip, ib, sizeof(s_ip) - 1); // 网页「连接」页显示用
        s_ip[sizeof(s_ip) - 1] = 0;
    }
    snprintf(s_cfgStr, sizeof(s_cfgStr), "ok %s/%s%s",
             (e8 == ESP_OK) ? (const char*)cur.ap.ssid : "?",
             ib, (e8 != ESP_OK) ? " cfgE" : (e9 != ESP_OK) ? " ipE" : "");
    printf("[WiFi] AP up: %s\n", s_cfgStr);
    s_wifiUp = true;
    s_startUs = esp_timer_get_time();
    s_lastHttpUs = s_startUs;
    s_lastActUs  = s_startUs; // v32.20：闲置从 AP 起来这一刻算起

    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.max_uri_handlers = 12; // 7 精确路由 + 2 OTA + 2 通配（v32.21 多了 POST /api/set）
    hc.stack_size = 4096;     // OTA handler 大缓冲均为 static，不占任务栈（省内存给 BLE）
    // ===== v32.16 关键修复 =====
    // HTTPD_DEFAULT_CONFIG() 里 uri_match_fn 是 NULL，此时 httpd 内部走的是
    //   hd->config.uri_match_fn ? ... : httpd_uri_match_simple(...)
    // 也就是**精确字符串比较**，不是通配符匹配！所以我们注册的 "/*" 只能匹配
    // 字面量 "/*"，手机探测的 /generate_204、/hotspot-detect.html 全部 404
    // （实测日志铁证：httpd_uri: URI '/generate_204' not found）。
    // 必须显式装上通配符匹配函数，captive portal 才真正生效。
    hc.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&s_httpd, &hc) == ESP_OK) {
        static const httpd_uri_t u_root  = {.uri="/",          .method=HTTP_GET, .handler=handler_root,  .user_ctx=NULL};
        static const httpd_uri_t u_state = {.uri="/api/state", .method=HTTP_GET, .handler=handler_state, .user_ctx=NULL};
        static const httpd_uri_t u_set   = {.uri="/api/set",   .method=HTTP_GET, .handler=handler_set,   .user_ctx=NULL};
        // v32.21：网页「应用」用 POST 一次提交全套参数，GET 那个留着给 curl 调试
        static const httpd_uri_t u_setp  = {.uri="/api/set",   .method=HTTP_POST, .handler=handler_set,  .user_ctx=NULL};
        static const httpd_uri_t u_rb    = {.uri="/api/reboot",   .method=HTTP_GET, .handler=handler_reboot, .user_ctx=NULL};
        static const httpd_uri_t u_capt  = {.uri="/*",         .method=HTTP_GET, .handler=handler_captive, .user_ctx=NULL};
        // v32.17：华为/荣耀的联网探测会 POST /mtuprobe，之前只能落到 405 错误处理器再转 302，
        // 每轮都先冒一行 W httpd_uri: Method '3' not allowed。直接注册 POST 通配，少一层噪音，
        // 也避免某些 ROM 把 405 当成「服务器故障」而不是「需要认证」。
        static const httpd_uri_t u_captp = {.uri="/*",         .method=HTTP_POST, .handler=handler_captive, .user_ctx=NULL};
        httpd_register_uri_handler(s_httpd, &u_root);
        httpd_register_uri_handler(s_httpd, &u_state);
        httpd_register_uri_handler(s_httpd, &u_set);
        httpd_register_uri_handler(s_httpd, &u_setp); // POST 版；必须在通配之前注册
        httpd_register_uri_handler(s_httpd, &u_rb);
        otaHttpdRegister(s_httpd);                    // /ota 上传页 + /api/ota
        // 通配必须最后注册：httpd 按注册顺序线性匹配，先注册的精确路由优先；
        // 若通配在前，/ota 页面会被它抢走并 302 回首页。
        httpd_register_uri_handler(s_httpd, &u_capt);
        httpd_register_uri_handler(s_httpd, &u_captp); // POST 通配同理：必须排在 /api/ota 之后
        // 兜底：404/405 都交给 captive 处理器（POST/HEAD 探测也能被重定向）
        httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, handler_captive_err);
        httpd_register_err_handler(s_httpd, HTTPD_405_METHOD_NOT_ALLOWED, handler_captive_err);
        printf("[WiFi] 热点 %s 已开启；无人使用满 60 秒会自动关 WiFi + 蓝牙\n", WIFI_SSID);
        printf("[CP] 通配路由已启用（uri_match_fn=wildcard），探测请求一律 302 → http://192.168.4.1/\n");
    } else {
        printf("[WiFi] httpd FAILED\n");
        s_httpd = NULL;
    }
    xTaskCreate(dnsTask, "dns", 4096, NULL, 5, NULL); // 常驻：热点关闭后无人查询，无害
    // v32.21：关闭任务只建这一次，靠任务通知唤醒。放在这里而不是关的时候再建，
    // 是因为「关不掉」的最常见原因就是关的时候建任务失败/来不及。
    if (xTaskCreate(pwrTask, "pwrOff", 3072, NULL, 4, &s_pwrTask) != pdPASS) {
        s_pwrTask = NULL;
        printf("[PWR] 关闭任务创建失败：闲置到点会在主循环里关\n");
    }
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
// v32.20：判定口径换成「一分钟没人用就关 WiFi + 蓝牙」。
// 什么算「在用」——三选一，任一成立都把闲置计时归零：
//   ① 有手机连在热点上（staOnline > 0）
//   ② 网页有请求（touchHttp 会刷新 s_lastActUs，页面每秒轮询就守得住）
//   ③ 蓝牙连着（bleUartActive，调到一半不能被掐）
// 光广播不算在用，否则蓝牙一开着闲置计时就永远被顶住，永远关不掉。
void wifiCfgPoll(uint32_t nowMs) {
    (void)nowMs;
    if (s_pwrDone) return; // 已经关完，没什么可判

    if (s_stopping) {
        // v32.21：看门狗。正常关闭 1~2 秒就结束；超过 20 秒说明卡在某个驱动调用里
        // （前几版就是卡在 httpd_stop）。这里把标志解开、重新唤醒任务再来一遍，
        // 保证「一定会关掉」，而不是默默永远关不掉。
        uint32_t el = (uint32_t)(esp_timer_get_time() / 1000) - s_offAtMs;
        if (el > 20000) {
            printf("[PWR] 关闭超过 %u 秒未完成，重新排队\n", (unsigned)(el / 1000));
            s_stopping = false;
            if (s_pwrTask) xTaskNotifyGive(s_pwrTask);
        }
        return;
    }

    if (!s_wifiUp && !bleUartActive()) return; // 都关了，没什么可判

    uint64_t nowUs = esp_timer_get_time();

    // 热点已经停了就别再问驱动要 STA 列表（deinit 之后这调用只会返回错误）
    int n = s_wifiUp ? staOnline() : 0;
    s_staCount = n;

    if (n > 0 || bleUartActive()) { // ① 或 ③：有人在用，无限续期
        s_everConnected = true;
        s_lastActUs = nowUs;
        return;
    }

    // ②③ 都不成立，看距离上次「用过」多久了（touchHttp 已经算过 ②）
    uint64_t idleMs = (nowUs - s_lastActUs) / 1000;
    if (idleMs > IDLE_OFF_MS) {
        shutdownRequest("无线闲置满 60 秒");
        return;
    }
    // 每跨过 10 秒报一次剩余，串口能看到它确实在往 0 走
    static int lastBucket = -1;
    int left = (int)((IDLE_OFF_MS - idleMs) / 1000);
    if (left / 10 != lastBucket) {
        lastBucket = left / 10;
        printf("[PWR] 无人使用，%d 秒后关闭 WiFi 与蓝牙\n", left);
    }
}

// v32.21：网页「连接」页的倒计时。返回距自动关闭的秒数；
//   -1 = 有人在用（有设备连着 / 蓝牙连着 / 已关完），不会自动关
int wifiCfgIdleLeft() {
    if (!s_wifiUp) return -1;
    if (s_pwrDone || s_stopping) return 0;
    if (s_staCount > 0 || bleUartActive()) return -1;
    uint64_t used = (esp_timer_get_time() - s_lastActUs) / 1000;
    if (used >= IDLE_OFF_MS) return 0;
    return (int)((IDLE_OFF_MS - used) / 1000);
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
    // v32.20：统一按「闲置满 60 秒关」算剩余，口径只有一套，不再区分首次/保活期
    if (n == 0 && !bleUartActive()) {
        uint64_t nowUs = esp_timer_get_time();
        uint64_t used  = (nowUs - s_lastActUs) / 1000;
        left = (used >= IDLE_OFF_MS) ? 0 : (int)((IDLE_OFF_MS - used) / 1000);
    }
    snprintf(d, sizeof(d), "sta=%d off=%d dq=%u", n, left, (unsigned)s_dnsQ);
    return d;
}
