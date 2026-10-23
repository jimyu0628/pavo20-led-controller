// ============================================================
// BLE NUS 透传（v32.5）：NimBLE GATT NUS 服务 + UART1 桥接飞控 MSP
// v32.3 起改用 UART1(GPIO6/7)：GPIO20/21 是 SuperMini 板载 USB 串口引脚，
// 外接飞控会与 CH340 双驱动互拉（WiFi/BLE 射频被干扰），故换专用串口
//
// v32.5 针对「SpeedyBee 连接 2 秒后断开（MSP 握手无回复）」的加固：
//   1) notify 按 (MTU-3) 分片——原来一次塞 256 字节，超过 MTU 会返回
//      BLE_HS_EMSGSIZE 且整包丢弃，App 永远等不到回复（最可能的真凶）
//   2) 跟踪 CCCD 订阅状态（BLE_GAP_EVENT_SUBSCRIBE），未订阅不发 notify
//   3) notify 成功/失败分别计数并记录最后一次错误码，心跳直接看得到
//   4) 主动协商 MTU（247），并记录协商结果
//   5) 连接后请求更快的连接参数，WiFi/BLE 共存时不易被 supervision 超时踢掉
//   6) 开机 5s + 每次断开后自动「MSP 探测」：固件自己发一帧 MSP_API_VERSION，
//      看飞控到底回不回，一条心跳就能分清是飞控侧还是固件侧的问题
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
static uint16_t s_connHandle = 0; // 当前连接句柄（通知用）
static uint16_t s_txHandle = 0;
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

uint32_t bleUartTxCount(void) { return s_uartTx; }
uint32_t bleUartRxCount(void) { return s_uartRx; }
int bleUartProbe(void) { return s_probe; }

// 心跳/网页用的一行诊断串
// 用双缓冲交替：心跳（主任务）和 /api/state（httpd 任务）会并发调用，
// 共用一个 static buffer 可能互相覆盖，导致调试区里数字串行
const char* bleUartDiag(void) {
    static char b[2][96];
    static volatile int idx = 0;
    int i = idx;
    idx = 1 - i;
    snprintf(b[i], sizeof(b[i]), "sub=%d mtu=%d nf=%u/%u e%d probe=%s",
             s_sub, s_mtu, (unsigned)s_nfOk, (unsigned)s_nfErr, s_nfLastErr,
             s_probe < 0 ? "?" : (s_probe ? "ok" : "fail"));
    return b[i];
}

// NUS UUID：6E400001/2/3-B5A3-F393-E0A9-E50E24DCCA9E（128-bit LE 字节序）
static const ble_uuid128_t nus_svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0x93, 0xA3, 0xB5, 0x01, 0x40, 0x6E); // 服务
static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0x93, 0xA3, 0xB5, 0x03, 0x40, 0x6E); // 写（手机->飞控）
static const ble_uuid128_t nus_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0x93, 0xA3, 0xB5, 0x02, 0x40, 0x6E); // 通知（飞控->手机）

const char* bleUartState(void) {
    if (s_initErr) return "ble err";
    if (!s_advOk) return "ble adv fail";
    if (!s_connected) return "ble adv";
    return s_sub ? "ble conn" : "ble conn(no sub)";
}

// ---------------- 分片通知（修复 MTU 超限整包丢弃） ----------------
static void notifyBytes(const uint8_t* p, int n) {
    if (!s_connected || s_connHandle == 0 || s_txHandle == 0) return;
    int maxPayload = s_mtu - 3;
    if (maxPayload < 20) maxPayload = 20; // ATT 默认最小值兜底
    int off = 0;
    while (off < n) {
        int chunk = n - off;
        if (chunk > maxPayload) chunk = maxPayload;
        struct os_mbuf* om = ble_hs_mbuf_from_flat(p + off, chunk);
        if (!om) { s_nfErr++; s_nfLastErr = -1; return; }
        int rc = ble_gatts_notify_custom(s_connHandle, s_txHandle, om);
        if (rc != 0) {
            // 未订阅(EDONE)/缓冲区满(ENOMEM)/超长(EMSGSIZE) 都记下来
            s_nfErr++;
            s_nfLastErr = rc;
            return;
        }
        s_nfOk += (uint32_t)chunk;
        off += chunk;
    }
}

// ---------------- GATT 特征回调 ----------------
static int nus_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt* ctxt, void* arg) {
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        int n = os_mbuf_len(ctxt->om);
        if (n > 0) {
            uint8_t tmp[256];
            if (n > (int)sizeof(tmp)) n = (int)sizeof(tmp);
            os_mbuf_copydata(ctxt->om, 0, n, tmp);
            uart_write_bytes(NUS_UART, tmp, n); // -> 飞控
            s_uartTx += (uint32_t)n;
        }
    }
    return 0;
}

static int nus_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt* ctxt, void* arg) {
    (void)conn_handle; (void)attr_handle; (void)ctxt; (void)arg;
    return 0; // 只用于通知，读不返回数据
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {.uuid = &nus_rx_uuid.u,
             .access_cb = nus_rx_cb,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
            {.uuid = &nus_tx_uuid.u,
             .access_cb = nus_tx_cb,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
             .val_handle = &s_txHandle},
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

static void adv_start(void) {
    struct ble_gap_adv_params adv = {0};
    struct ble_hs_adv_fields f = {0};
    const char* devName = "PavoLED";
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP; // 可发现+仅LE：手机列表才显示
    f.uuids128 = &nus_svc_uuid;          // 广播带 NUS 服务 UUID（SpeedyBee 等按服务识别）
    f.num_uuids128 = 1;
    f.uuids128_is_complete = 1;
    f.name = (uint8_t*)devName;
    f.name_len = (uint8_t)strlen(devName);
    f.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&f);
    if (rc != 0) { printf("[BLE] adv set fields err %d\n", rc); s_advOk = 0; return; }
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;   // 可连接
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;   // 可发现
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv, gap_event, NULL);
    if (rc != 0) { printf("[BLE] adv start err %d\n", rc); s_advOk = 0; return; }
    s_advOk = 1;
    printf("[BLE] 广播已启动：PavoLED\n");
}

static int gap_event(struct ble_gap_event* event, void* arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connected = 1;
            s_connHandle = event->connect.conn_handle;
            s_sub = 0;
            printf("[BLE] 已连接\n");
            // 请求更快的连接参数：WiFi AP 同时在跑（单射频分时），
            // 连接间隔太大会让 MSP 回复迟到甚至被 supervision 超时踢掉
            struct ble_gap_upd_params up = {0};
            up.itvl_min = 12;   // 15ms
            up.itvl_max = 24;   // 30ms
            up.latency = 0;
            up.timeout = 400;   // 4s 监督超时
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
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_txHandle) {
            s_sub = event->subscribe.cur_notify ? 1 : 0;
            printf("[BLE] 通知订阅=%d\n", s_sub);
        }
        break;
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
            notifyBytes(buf, n);
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
    ble_svc_gap_device_name_set("PavoLED");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    nimble_port_freertos_init(host_task);

    xTaskCreate(uart_rx_task, "bleuart", 3072, NULL, 5, NULL);
    // 开机 5s 自动探测一次飞控（此时飞控已启动完），结果进心跳/网页调试区
    xTaskCreate([](void*) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        probeTask(NULL);
    }, "mspProbe0", 2560, NULL, 4, NULL);
}
