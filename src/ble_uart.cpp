// ============================================================
// BLE NUS 透传（v32.5）：NimBLE GATT NUS 服务 + UART1 桥接飞控 MSP
// v32.3 起改用 UART1(GPIO6/7)：GPIO20/21 是 SuperMini 板载 USB 串口引脚，
// 外接飞控会与 CH340 双驱动互拉（WiFi/BLE 射频被干扰），故换专用串口
//
// v32.5 针对「SpeedyBee 连接 2 秒后断开（MSP 握手无回复）」的加固：
//   1) notify 按 (MTU-3) 分片——原来一次塞 256 字节，超过 MTU 会返回
//      BLE_HS_EMSGSIZE 且整包丢弃，App 永远等不到回复
//   2) 跟踪 CCCD 订阅状态（BLE_GAP_EVENT_SUBSCRIBE），未订阅不发 notify
//   3) notify 成功/失败分别计数并记录最后一次错误码，心跳直接看得到
//   4) 主动协商 MTU（247），并记录协商结果
//   5) 连接后请求更快的连接参数，WiFi/BLE 共存时不易被 supervision 超时踢掉
//   6) 开机 5s + 每次断开后自动「MSP 探测」：固件自己发一帧 MSP_API_VERSION，
//      看飞控到底回不回，一条心跳就能分清是飞控侧还是固件侧的问题
//
// v32.6 = 2 秒断开真正的根因：NUS 两个特征的 UUID 装反了
//   标准 NUS：...02 = RX（手机写进来）、...03 = TX（通知给手机）
//   v32.5 之前把 ...03 注册成「只写」、把 ...02 注册成「只读+通知」，于是：
//     - App 往 ...02 写 MSP 请求 → 该特征无 WRITE 属性 → ATT 错误，字节进不来
//       （心跳里 tx=0/s，飞控收不到任何请求）
//     - App 去订阅 ...03 的 CCCD → 该特征无 NOTIFY 属性、没有 CCCD 描述符
//       → 订阅失败（心跳里 sub=0）
//     - App 等 2s 没有任何 MSP 回应 → 自己断开，reason=0x13 Remote User Terminated
//   实测日志完全对上（mtu=247 说明服务发现正常、probe=ok 说明 UART1 正常）。
//   现按标准改正，并额外给写特征也加上 NOTIFY、按实际订阅的句柄发通知，
//   兼容把收发放进同一个特征的 App 变体。
//
// v32.7：补「先写后订阅」的时序漏洞——App 若在开通知之前就发出 MSP 请求，
//   飞控的回复会因为当时没有订阅者而丢掉。现在未订阅期间的 UART 回包先入
//   512 字节暂存区，订阅事件一到立即补发（只补 1s 内的），断开时清空。
//
// v32.8：SpeedyBee App 根本不用 NUS。
//   实测 v32.7：App 连上后只做 MTU 协商（247），sub=0、tx=0，一个 ATT 写都没发，
//   2.6s 后 reason=0x213 主动断开并报「设备初始化失败」；扫描列表里也只显示
//   「未知设备」。说明服务发现后没找到它认的服务。
//   SpeedyBee 官方说明「任何兼容的蓝牙模块都可使用」，它兼容的第三方模块是
//   HM-10/AT-09 系列（服务 FFE0 / 特征 FFE1）。因此现在同时挂三套服务：
//     NUS(6E400001)  —— Betaflight Configurator / Web Bluetooth
//     FFE0/FFE1      —— HM-10、SpeedyBee App
//     FFF0/FFF1/FFF2 —— 部分国产透传模块
//   三套共用同一个 UART 桥，App 认哪套用哪套；FFE0/FFF0 放进扫描响应
//   （主广播包已被 NUS 128-bit UUID + 名字占满 30/31 字节）。
//   另外每条特征访问都会打印 h=句柄 与首字节，下次认不出设备时能直接看
//   App 到底碰了哪个句柄。
//
// v32.9：v32.8 实测仍然 sub=0@0、tx=0、无任何 WR —— App 连上后依然只做 MTU
//   协商就放弃，说明它对 FFE0/FFF0 也没认（或者根本没扫到）。两点调整：
//   1) 把 FFE0/FFF0 挪进**主广播包**，NUS 的 128-bit UUID 挪到扫描响应。
//      有些 App 只解析 adv data、不合并 scan response，按服务 UUID 过滤就漏
//   2) 连接期间把 NimBLE 日志升到 DEBUG，ATT 层每个请求（Read By Group Type /
//      Read By Type / Read / Write / CCCD）都会打出来；断开自动降回 INFO。
//      再配一条「1.5s 无订阅」提示，下次日志能直接看出 App 停在哪一步。
//
// v32.10：实测 Betaflight App 网页版能扫到 PavoLED、能连、MTU=247，但只写了
//   一次 Service Changed(0x2A05) 的 CCCD(句柄 8) 就报「打开串口失败」，
//   全程没碰 NUS/FFE0 任何一个特征。结合官方文档「兼容 SpeedyBee App 支持的
//   飞控」+ 两个开源项目的失败记录，判断这类 App 的 BLE 通道只认厂家模块，
//   第三方桥很难过白名单，继续在 BLE 上耗性价比太低。
//   => 改走 Betaflight Configurator 官方支持的 TCP MSP 路线（见 msp_tcp.cpp，
//      端口 5761，本板 WiFi AP 直接当 ELRS 那样的 TCP-MSP 网关），
//      BLE 三套服务保留不动，nRF Connect 等通用工具仍可直连调试。
//
// v32.11：找到 Betaflight App 源码（src/js/protocols/devices.ts），蓝牙 profile 是
//   写死的列表，逐条对下来终于知道它要什么：
//     CC2541       = [服务 ffe0, 写 ffe1, 读/通知 ffe2]        ← 之前我们缺 FFE2
//     HM-10        = [服务 ffe1, 写 ffe1, 读 ffe1]
//     HM-11/NRF    = [服务 6e400001, 写 6e400003, 读 6e400002] ← 写/读方向跟我们相反
//     SpeedyBee V2 = [服务 abf0, 写 abf1, 读 abf2] = ESP-IDF ble_spp_server
//   它先按服务 UUID 认设备，认到之后拿两个特征 UUID 去 GATT 表里找；找不到就在
//   getCharacteristics() 抛错 → 界面「打开串口失败」。这正好解释实测：连上了、
//   MTU 也协商了，却一次特征都没碰。
//   对策：FFE0 下补 FFE2；NUS 两个特征都改成读写通知全开；再补一套 ABF0/ABF1/ABF2。
//   用户那份教程里的「AT+CHAR=FFE2」就是把模块特征改成 FFE2 去凑 CC2541 profile。
// ============================================================
#include "ble_uart.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"
#include "esp_log.h"
#include "msp_tcp.h"   // v32.10：同一条 UART1 同时挂 BLE 与 TCP 两个出口
#include "device_name.h"  // v32.18：BLE 广播名与 WiFi SSID 同源
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NUS_UART      UART_NUM_1
#define NUS_TX_PIN    6    // ESP32 GPIO6 -> 飞控 UART1 RX(A10)
#define NUS_RX_PIN    7    // 飞控 UART1 TX(A09) -> ESP32 GPIO7
#define NUS_BAUD      115200
#define NUS_MTU_WANT  247  // 期望 MTU（247 -> notify 载荷 244）

static volatile int s_connected = 0;
static uint16_t s_connHandle = 0;   // 当前连接句柄（通知用）
static uint16_t s_wrHandle = 0;     // 写特征句柄（NUS RX，手机->飞控）
static uint16_t s_ntHandle = 0;     // 通知特征句柄（NUS TX，飞控->手机）
static uint16_t s_ffHandle = 0;     // HM-10 FFE1（读写+通知，二合一）
static uint16_t s_ff2Handle = 0;    // CC2541 FFE2（读/通知）
static uint16_t s_abWrHandle = 0;   // ABF1 写
static uint16_t s_abNtHandle = 0;   // ABF2 读/通知
static uint16_t s_f0NtHandle = 0;   // FFF1 通知
static uint16_t s_f0WrHandle = 0;   // FFF2 写
static volatile uint16_t s_subHandle = 0; // 实际被订阅通知的句柄（0=未订阅）
static volatile int s_initErr = 0;   // BLE 初始化失败标记
static volatile int s_advOk = 0;     // 广播是否成功发出
static volatile uint32_t s_uartTx = 0; // 透传：手机->飞控 累计字节
static volatile uint32_t s_uartRx = 0; // 透传：飞控->手机 累计字节（UART 实际收到）
static volatile uint32_t s_nfOk = 0;   // notify 成功送达字节
static volatile uint32_t s_nfErr = 0;  // notify 失败次数
static volatile int s_nfLastErr = 0;   // 最后一次 notify 错误码
static volatile int s_mtu = 23;        // 当前 MTU（默认 23）
static volatile int s_sub = 0;         // CCCD 是否已订阅
static volatile int s_probe = -1;      // MSP 探测：-1 未测 / 0 飞控无回复 / 1 有回复
static volatile int s_bleOn = 0;       // v32.20：BLE 是否在跑（1=广播/连接中，0=已关闭省电）

// 未订阅期间暂存的飞控回包（订阅一开就补发）
#define PEND_CAP 512
static uint8_t s_pend[PEND_CAP];
static volatile int s_pendLen = 0;
static volatile uint32_t s_pendMs = 0;

uint32_t bleUartTxCount(void) { return s_uartTx; }
uint32_t bleUartRxCount(void) { return s_uartRx; }
int bleUartProbe(void) { return s_probe; }

// v32.20：闲置判定只看「有没有手机连着」，光在广播不算活跃——
// 否则只要没关 BLE，闲置计时永远被广播状态顶住，热点和蓝牙就永远关不掉。
int bleUartActive(void) { return (s_bleOn && s_connected) ? 1 : 0; }

// v32.20：关闭 BLE 省电。顺序很重要：
//   1) 先断连接（否则 nimble_port_stop 会等连接事件收尾）
//   2) 停广播
//   3) 停 host 任务、反初始化协议栈
// 关掉之后 uart_rx_task 仍在跑（TCP 通道还要用 UART1），
// 但 s_connected 已清零，不会再调 notify——协议栈已卸载，再调就崩。
void bleUartStop(void) {
    if (!s_bleOn) return;
    s_bleOn = 0;
    int wasConn = s_connected;

    if (s_connected && s_connHandle) {
        ble_gap_terminate(s_connHandle, BLE_ERR_REM_USER_CONN_TERM);
    }
    s_connected  = 0;
    s_sub        = 0;
    s_subHandle  = 0;
    s_connHandle = 0;

    ble_gap_adv_stop();
    s_advOk = 0;

    int r1 = nimble_port_stop();
    esp_err_t r2 = nimble_port_deinit();
    printf("[BLE] 已关闭（原连接=%d stop=%d deinit=%d）——重启才能恢复\n",
           wasConn, r1, (int)r2);
}

// 心跳/网页用的一行诊断串
// 用双缓冲交替：心跳（主任务）和 /api/state（httpd 任务）会并发调用，
// 共用一个 static buffer 可能互相覆盖，导致调试区里数字串行
const char* bleUartDiag(void) {
    static char b[2][96];
    static volatile int idx = 0;
    int i = idx;
    idx = 1 - i;
    snprintf(b[i], sizeof(b[i]), "sub=%d@%u mtu=%d nf=%u/%u e%d p=%d probe=%s",
             s_sub, (unsigned)s_subHandle, s_mtu,
             (unsigned)s_nfOk, (unsigned)s_nfErr, s_nfLastErr, s_pendLen,
             s_probe < 0 ? "?" : (s_probe ? "ok" : "fail"));
    return b[i];
}

// NUS UUID：6E400001-B5A3-F393-E0A9-E50E24DCCA9E（128-bit LE 字节序）
// 标准定义必须严格遵守：
//   6E400002 = RX ：手机写进来的数据（本机 -> 飞控）
//   6E400003 = TX ：通知给手机的数据（飞控 -> 本机）
// v32.5 及以前这两个后缀写反了，导致 App 写不进、也订阅不上（见文件头说明）
static const ble_uuid128_t nus_svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0x93, 0xA3, 0xB5, 0x01, 0x40, 0x6E); // 服务
static const ble_uuid128_t nus_wr_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0x93, 0xA3, 0xB5, 0x02, 0x40, 0x6E); // RX 写（手机->飞控）
static const ble_uuid128_t nus_nt_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0x93, 0xA3, 0xB5, 0x03, 0x40, 0x6E); // TX 通知（飞控->手机）

// v32.8：SpeedyBee App 认不出 NUS，改用 HM-10/AT-09 那一套（FFE0/FFE1）。
// 官方说明「任何兼容的蓝牙模块都可使用」，它兼容的第三方模块就是 HM-10 系列；
// 我们只挂 NUS 时，App 服务发现后找不到目标服务 → 直接报「初始化失败」并断开
// （实测：连上后只做 MTU 协商，sub=0、tx=0，一个 ATT 写都没发）。
// 现在三套服务同时挂上、共用同一个 UART 桥，App 认哪套就用哪套。
static const ble_uuid16_t ff_svc_uuid = BLE_UUID16_INIT(0xFFE0); // HM-10 服务
static const ble_uuid16_t ff_chr_uuid = BLE_UUID16_INIT(0xFFE1); // HM-10 收发特征
// v32.11：Betaflight App 源码里 CC2541 profile = [服务 ffe0, 写 ffe1, 读/通知 ffe2]。
// 我们原来只有 FFE1，App 拿不到 FFE2 就在 getCharacteristics() 抛错，界面报
// 「打开串口失败」——正好对上实测现象（连上、MTU 247、一次特征都没碰）。
static const ble_uuid16_t ff2_nt_uuid = BLE_UUID16_INIT(0xFFE2); // CC2541 读/通知
static const ble_uuid16_t f0_svc_uuid = BLE_UUID16_INIT(0xFFF0); // 部分国产模块
static const ble_uuid16_t f0_nt_uuid  = BLE_UUID16_INIT(0xFFF1); // FFF1 通知
static const ble_uuid16_t f0_wr_uuid  = BLE_UUID16_INIT(0xFFF2); // FFF2 写
// Betaflight App 的「SpeedyBee V2」profile = [abf0, abf1 写, abf2 读]，
// 也就是 ESP-IDF ble_spp_server 那套；SpeedyBee App 走的就是它（开源项目实测能连上并发出 MSP 请求）
static const ble_uuid16_t abf_svc_uuid = BLE_UUID16_INIT(0xABF0);
static const ble_uuid16_t abf_wr_uuid  = BLE_UUID16_INIT(0xABF1);
static const ble_uuid16_t abf_nt_uuid  = BLE_UUID16_INIT(0xABF2);

const char* bleUartState(void) {
    if (!s_bleOn) return "ble off"; // v32.20：闲置超时或网页主动关闭
    if (s_initErr) return "ble err";
    if (!s_advOk) return "ble adv fail";
    if (!s_connected) return "ble adv";
    return s_sub ? "ble conn" : "ble conn(no sub)";
}

// ---------------- 分片通知（修复 MTU 超限整包丢弃） ----------------
// 发到「实际被订阅的句柄」；万一没收到订阅事件，退回标准 TX 句柄试一次，
// 失败会计进 nf 计数，不会变成静默黑洞
static void notifyBytes(const uint8_t* p, int n) {
    if (!s_connected || s_connHandle == 0) return;
    uint16_t h = s_subHandle ? s_subHandle : s_ntHandle;
    if (h == 0) return;
    int maxPayload = s_mtu - 3;
    if (maxPayload < 20) maxPayload = 20; // ATT 默认最小值兜底
    int off = 0;
    while (off < n) {
        int chunk = n - off;
        if (chunk > maxPayload) chunk = maxPayload;
        struct os_mbuf* om = ble_hs_mbuf_from_flat(p + off, chunk);
        if (!om) { s_nfErr++; s_nfLastErr = -1; return; }
        int rc = ble_gatts_notify_custom(s_connHandle, h, om);
        if (rc != 0) {
            // 未订阅(EDONE)/缓冲区满(ENOMEM)/超长(EMSGSIZE) 都记下来
            s_nfErr++;
            s_nfLastErr = rc;
            return;
        }        s_nfOk += (uint32_t)chunk;
        off += chunk;
    }
}

// ---------------- 未订阅时的回包暂存（v32.6） ----------------
// 部分 App 连上后先写 MSP_API_VERSION、再开通知订阅。若飞控回复先到，
// 此时没有订阅者，notify 会直接失败，App 依旧白等。这里把这段时间的
// 回包攒起来，订阅一开立刻补发（只补 1s 内的，太旧的对 MSP 无意义）
static void pendAppend(const uint8_t* p, int n) {
    if (n >= PEND_CAP) { p += (n - PEND_CAP); n = PEND_CAP; s_pendLen = 0; }
    if (s_pendLen + n > PEND_CAP) { // 溢出：丢掉最旧的
        int keep = PEND_CAP - n;
        if (keep < 0) keep = 0;
        if (keep > 0) memmove(s_pend, s_pend + (s_pendLen - keep), keep);
        s_pendLen = keep;
    }
    memcpy(s_pend + s_pendLen, p, n);
    s_pendLen += n;
    s_pendMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void pendFlush(void) {
    int n = s_pendLen;
    if (n <= 0) return;
    uint32_t age = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - s_pendMs;
    s_pendLen = 0;
    if (age < 1000 && s_subHandle) {
        notifyBytes(s_pend, n);
    }
}

// ---------------- GATT 特征回调 ----------------
// v32.8：所有写特征共用一个回调；顺手把「App 到底碰了哪个句柄」打到串口，
// 下次 App 认不出设备时，一眼就能看出它有没有读写我们的特征
static volatile int s_dbgLeft = 0; // 每次连接后前 N 条访问打印，防刷屏

static void dbgAccess(const char* tag, uint16_t h, int n, const uint8_t* p) {
    if (s_dbgLeft <= 0) return;
    s_dbgLeft--;
    if (n > 0 && p) {
        printf("[BLE] %s h=%u len=%d [%02X %02X %02X %02X %02X %02X]\n", tag,
               (unsigned)h, n, p[0], n > 1 ? p[1] : 0, n > 2 ? p[2] : 0,
               n > 3 ? p[3] : 0, n > 4 ? p[4] : 0, n > 5 ? p[5] : 0);
    } else {
        printf("[BLE] %s h=%u len=%d\n", tag, (unsigned)h, n);
    }
}

static int chr_wr_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt* ctxt, void* arg) {
    (void)conn_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        int n = os_mbuf_len(ctxt->om);
        if (n > 0) {
            uint8_t tmp[256];
            if (n > (int)sizeof(tmp)) n = (int)sizeof(tmp);
            os_mbuf_copydata(ctxt->om, 0, n, tmp);
            dbgAccess("WR", attr_handle, n, tmp);
            // HM-10 的 AT 探测：这类模块会对 "AT" 回 "OK"。不能转给飞控
            // （会污染 MSP 流），这里自己回一句，让 App 认为接的是透传模块。
            // MSP 帧一定以 '$'(0x24) 开头，不可能误判。
            if (n >= 2 && tmp[0] == 'A' && tmp[1] == 'T') {
                static const char ok[] = "OK\r\n";
                notifyBytes((const uint8_t*)ok, 4);
                return 0;
            }
            uart_write_bytes(NUS_UART, tmp, n); // -> 飞控
            s_uartTx += (uint32_t)n;
        }
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        dbgAccess("RD", attr_handle, 0, NULL);
    }
    return 0;
}

static int chr_nt_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt* ctxt, void* arg) {
    (void)conn_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) dbgAccess("RD", attr_handle, 0, NULL);
    return 0; // 只用于通知，读不返回数据
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    // ① Nordic UART（Betaflight Configurator / Web Bluetooth 用这套）
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            // NUS RX（...02）：手机写 -> 飞控。同时也开 NOTIFY，
            // 兼容把收发放进同一个特征的 App 变体
            {.uuid = &nus_wr_uuid.u,
             .access_cb = chr_wr_cb,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                      BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_wrHandle},
            // NUS TX（...03）：飞控 -> 手机通知。
            // v32.11：Betaflight App 的 NUS profile 把「写」放在 ...03、「读/通知」放在 ...02
            // （src/js/protocols/devices.ts 里 NORDIC_UART = [svc, 6003, 6002]，跟我们原来的假设相反），
            // 所以这个特征也补上 WRITE，两个方向都认。
            {.uuid = &nus_nt_uuid.u,
             .access_cb = chr_wr_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                      BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_ntHandle},
            {0}
        }
    },
    // ② HM-10 / AT-09 / CC2541（服务 FFE0，特征 FFE1 收发 + FFE2 读/通知）
    // v32.11：Betaflight App 的 CC2541 profile 硬性要 [ffe0, ffe1 写, ffe2 读]，
    // 缺 FFE2 会在 getCharacteristics() 直接抛错（界面提示「打开串口失败」）。
    // 很多 HM-10 固件把收发都放在 FFE1 且没有 FFE2，App 有 fallback 兜底，
    // 但那是「FFE1 带 notify 属性」时才走；两个都给上最稳。
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &ff_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = &ff_chr_uuid.u,
             .access_cb = chr_wr_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                      BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_ffHandle},
            {.uuid = &ff2_nt_uuid.u,
             .access_cb = chr_nt_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_ff2Handle},
            {0}
        }
    },
    // ③ ABF0/ABF1/ABF2 = ESP-IDF ble_spp_server 那套，Betaflight App 里叫「SpeedyBee V2」，
    // SpeedyBee App 走的也是它（开源项目实测能连上并发出 MSP 请求）
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &abf_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = &abf_wr_uuid.u,
             .access_cb = chr_wr_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                      BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_abWrHandle},
            {.uuid = &abf_nt_uuid.u,
             .access_cb = chr_nt_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_abNtHandle},
            {0}
        }
    },
    // ④ FFF0（部分国产透传模块：FFF1 通知 / FFF2 写）
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &f0_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = &f0_nt_uuid.u,
             .access_cb = chr_nt_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_f0NtHandle},
            {.uuid = &f0_wr_uuid.u,
             .access_cb = chr_wr_cb,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
             .val_handle = &s_f0WrHandle},
            {0}
        }
    },
    {0}
};

// ---------------- MSP 探测：固件自己问一句飞控 ----------------
// 发 $M< 0x00 0x01 0x01（MSP_API_VERSION 请求，size=0 cmd=1 crc=1），
// 300ms 内 UART1 收到任何字节 → 判飞控有回复。App 未连接时才执行，不干扰握手。
static void probeTask(void* arg) {
    (void)arg;
    static const uint8_t req[] = {0x24, 0x4D, 0x3C, 0x00, 0x01, 0x01};
    uint32_t before = s_uartRx;
    uart_write_bytes(NUS_UART, req, sizeof(req));
    uart_wait_tx_done(NUS_UART, pdMS_TO_TICKS(100));
    for (int i = 0; i < 8; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (s_uartRx != before) break;
    }
    s_probe = (s_uartRx != before) ? 1 : 0;
    printf("[MSP] 探测：%s\n", s_probe ? "飞控有回复（UART1 通）"
                                      : "飞控无回复（查线序/飞控 serial 0 64/波特率）");
    vTaskDelete(NULL);
}

// ---------------- 广播 ----------------
static int gap_event(struct ble_gap_event* event, void* arg); // 前向声明

// v32.9：连接后 1.5s 仍无订阅就给一句明确提示，省得对着一堆日志猜
static void subWatchTask(void* arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    if (s_connected && !s_sub) {
        printf("[BLE] 1.5s 内没有订阅（sub=0）：App 没找到/没启用任何通知特征\n");
    }
    vTaskDelete(NULL);
}

static void adv_start(void) {
    struct ble_gap_adv_params adv = {0};
    struct ble_hs_adv_fields f = {0};
    const char* dn = DEV_NAME; // v32.19：运行时名字，带 MAC 后缀，与 WiFi SSID 同一个来源
    // v32.11：主包放 FFE0 + ABF0（Betaflight App 的 CC2541 / SpeedyBee V2 两个 profile
    // 的服务 UUID，扫到任意一个都能匹配；acceptAllDevices 的 App 不看这个也不影响）。
    // 主包：flags(3) + FFE0(4) + ABF0(4) + 名字(12，含 1 长度 + 1 类型 + "PavoLED-A3" 10 字符)
    //       = 23 字节；NUS 128-bit 放扫描响应。上限 31 字节，还剩 8 字节余量。
    static const ble_uuid16_t adv_uuids[] = { BLE_UUID16_INIT(0xFFE0),
                                              BLE_UUID16_INIT(0xABF0) };
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP; // 可发现+仅LE：手机列表才显示
    f.uuids16 = adv_uuids;
    f.num_uuids16 = 2;
    f.uuids16_is_complete = 1;
    f.name = (uint8_t*)dn;
    f.name_len = (uint8_t)strlen(dn);
    f.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc != 0) { printf("[BLE] adv set fields err %d\n", rc); s_advOk = 0; return; }

    struct ble_hs_adv_fields rsp = {0};
    rsp.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    rsp.uuids128 = &nus_svc_uuid;   // Betaflight Configurator / Web Bluetooth 用 NUS
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    int rc2 = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc2 != 0) printf("[BLE] adv rsp err %d（不影响连接）\n", rc2);

    adv.conn_mode = BLE_GAP_CONN_MODE_UND;   // 可连接
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;   // 可发现
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv, gap_event, NULL);
    if (rc != 0) { printf("[BLE] adv start err %d\n", rc); s_advOk = 0; return; }
    s_advOk = 1;
    printf("[BLE] 广播已启动：%s（adv=FFE0/FFF0，rsp=NUS）\n", DEV_NAME);
}

static int gap_event(struct ble_gap_event* event, void* arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connected = 1;
            s_connHandle = event->connect.conn_handle;
            s_sub = 0;
            s_subHandle = 0;
            s_dbgLeft = 40; // 每次连接打印前 40 条特征访问，便于看 App 碰了谁
            // v32.9：连接期间把 NimBLE 主机日志调到 DEBUG，ATT 层每个请求
            // （Read By Group Type / Read By Type / Read / Write / CCCD 写）
            // 都会打出来。App 认不出设备时，这行日志能直接看出它到底停在哪一步
            esp_log_level_set("NimBLE", ESP_LOG_DEBUG);
            printf("[BLE] 已连接（NUS wr=%u nt=%u | FF: %u %u | ABF: %u %u | FFF: %u %u）\n",
                   (unsigned)s_wrHandle, (unsigned)s_ntHandle,
                   (unsigned)s_ffHandle, (unsigned)s_ff2Handle,
                   (unsigned)s_abWrHandle, (unsigned)s_abNtHandle,
                   (unsigned)s_f0NtHandle, (unsigned)s_f0WrHandle);
            xTaskCreate(subWatchTask, "bleSubW", 2048, NULL, 3, NULL);
            // 请求更快的连接参数：WiFi AP 同时在跑（单射频分时），
            // 连接间隔太大会让 MSP 回复迟到甚至被 supervision 超时踢掉
            struct ble_gap_upd_params up = {0};
            up.itvl_min = 12;   // 15ms
            up.itvl_max = 24;   // 30ms
            up.latency = 0;
            up.supervision_timeout = 400; // 4s 监督超时（单位 10ms）
            ble_gap_update_params(event->connect.conn_handle, &up);
        } else {
            s_connected = 0;
            adv_start(); // 连接失败继续广播
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        s_connected = 0;
        s_connHandle = 0;
        s_sub = 0;
        s_subHandle = 0;
        s_pendLen = 0; // 清掉上一轮的残包，避免污染下一次握手
        esp_log_level_set("NimBLE", ESP_LOG_INFO); // 断开后关掉 ATT 追踪，别刷屏
        printf("[BLE] 断开（reason=0x%x），重新广播\n", event->disconnect.reason);
        adv_start();
        // App 走了再探测，避免干扰握手
        xTaskCreate(probeTask, "mspProbe", 2560, NULL, 4, NULL);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        adv_start(); // 广播结束（超时）重新广播
        break;
    case BLE_GAP_EVENT_MTU:
        s_mtu = event->mtu.value;
        printf("[BLE] MTU=%d（notify 载荷 %d）\n", s_mtu, s_mtu - 3);
        break;
    case BLE_GAP_EVENT_SUBSCRIBE: {
        uint16_t h = event->subscribe.attr_handle;
        int on = event->subscribe.cur_notify ? 1 : 0;
        // v32.8：不再限定某个服务——App 订阅哪个句柄就往哪个发，
        // 这样 NUS / FFE1 / FFF1 三套都能工作
        if (on) {
            s_subHandle = h;
            s_sub = 1;
        } else if (h == s_subHandle) {
            s_subHandle = 0;
            s_sub = 0;
        }
        printf("[BLE] 订阅事件 h=%u notify=%d -> sub=%d@%u\n",
               (unsigned)h, on, s_sub, (unsigned)s_subHandle);
        if (s_sub) pendFlush(); // 订阅前一瞬到达的回包，补发给 App
        break;
    }
    default:
        break;
    }
    return 0;
}

// ---------------- UART RX -> BLE 通知 ----------------
static void uart_rx_task(void* arg) {
    (void)arg;
    uint8_t buf[256];
    while (1) {
        int n = uart_read_bytes(NUS_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0) {
            s_uartRx += (uint32_t)n; // 无论是否连接都计数：用于判断飞控有没有回
            mspTcpSend(buf, n);      // v32.10：有 TCP 客户端时同时转发给 PC
            if (s_connected && s_subHandle) notifyBytes(buf, n);
            else pendAppend(buf, n);     // 还没订阅：先攒着，订阅后补发
        }
    }
}

// ---------------- NimBLE 主机事件 ----------------
static void on_sync(void) {
    adv_start(); // 公共地址广播（C3 有出厂 MAC），无需地址类型推断
}

static void on_reset(int reason) {
    printf("[BLE] NimBLE reset: %d\n", reason);
}

static void host_task(void* param) {
    (void)param;
    nimble_port_run(); // 阻塞，NimBLE 主机任务
}

void bleUartInit(void) {
    // UART1：GPIO6 TX -> 飞控 RX(A10)，GPIO7 RX <- 飞控 TX(A09)，115200
    // 注意：不能用 GPIO20/21（UART0）——它们与板载 USB 串口(CH340)共用，
    // 外接飞控会双驱动互拉导致 WiFi/BLE 异常
    uart_config_t u = {
        .baud_rate = NUS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_param_config(NUS_UART, &u);
    uart_set_pin(NUS_UART, NUS_TX_PIN, NUS_RX_PIN, -1, -1);
    uart_driver_install(NUS_UART, 1024, 2048, 0, NULL, 0);

    // NimBLE 主机
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        s_initErr = 1;
        printf("[BLE] nimble_port_init FAIL %d\n", (int)ret);
        return;
    }
    ble_att_set_preferred_mtu(NUS_MTU_WANT); // 主动要 247，避免默认 23 把 MSP 帧切碎
    ble_svc_gap_device_name_set(DEV_NAME); // v32.18：与 WiFi SSID 同源
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    nimble_port_freertos_init(host_task);

    s_bleOn = 1; // v32.20：从这里开始才算「蓝牙在跑」，闲置计时才能判
    xTaskCreate(uart_rx_task, "bleuart", 3072, NULL, 5, NULL);
    // 开机 5s 自动探测一次飞控（此时飞控已启动完），结果进心跳/网页调试区
    xTaskCreate([](void*) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        probeTask(NULL);
    }, "mspProbe0", 2560, NULL, 4, NULL);
}
