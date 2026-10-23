// ============================================================
// BLE NUS 透传（v32）：NimBLE GATT NUS 服务 + UART0 桥接飞控 MSP
// 参考 IDF bleprph / NUS 例程；只做 Peripheral，省内存
// ============================================================
#include "ble_uart.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "host/ble_hs.h"
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

#define NUS_UART      UART_NUM_0
#define NUS_TX_PIN    21   // ESP32 GPIO21 -> 飞控 UART3 RX(B11)
#define NUS_RX_PIN    20   // 飞控 UART3 TX(B10) -> ESP32 GPIO20
#define NUS_BAUD      115200

static volatile int s_connected = 0;
static uint16_t s_connHandle = 0; // 当前连接句柄（通知用）
static uint16_t s_txHandle = 0;

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
    if (!s_connected) return "ble adv";
    return "ble conn";
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
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
             .access_cb = nus_rx_cb},
            {.uuid = &nus_tx_uuid.u,
             .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
             .access_cb = nus_tx_cb, .val_handle = &s_txHandle},
            {0}
        }
    },
    {0}
};

// ---------------- 广播 ----------------
static int gap_event(struct ble_gap_event* event, void* arg); // 前向声明

static void adv_start(void) {
    struct ble_gap_adv_params adv = {0};
    struct ble_hs_adv_fields f = {0};
    const char* devName = "PavoLED";
    f.name = (uint8_t*)devName;
    f.name_len = (uint8_t)strlen(devName);
    f.name_is_complete = 1;
    ble_gap_adv_set_fields(&f);
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;   // 可连接
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;   // 可发现
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv, gap_event, NULL);
}

static int gap_event(struct ble_gap_event* event, void* arg) {
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connected = 1;
            s_connHandle = event->connect.conn_handle;
            printf("[BLE] 已连接\n");
        } else {
            s_connected = 0;
            adv_start(); // 连接失败继续广播
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        s_connected = 0;
        s_connHandle = 0;
        printf("[BLE] 断开，重新广播\n");
        adv_start();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        adv_start(); // 广播结束（超时）重新广播
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
        int n = uart_read_bytes(NUS_UART, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n > 0 && s_connected) {
            struct os_mbuf* om = ble_hs_mbuf_from_flat(buf, n);
            if (om) {
                ble_gatts_notify_custom(s_connHandle, s_txHandle, om);
            }
        }
    }
}

// ---------------- NimBLE 主机事件 ----------------
static void on_sync(void) {
    ble_hs_util_ensure_addr_type_default();
    adv_start();
    printf("[BLE] 广播中：PavoLED（NUS 透传，等待 SpeedyBee App 连接）\n");
}

static void on_reset(int reason, void* arg) {
    (void)reason; (void)arg;
    printf("[BLE] NimBLE reset\n");
}

static void host_task(void* param) {
    (void)param;
    nimble_port_run(); // 阻塞，NimBLE 主机任务
}

void bleUartInit(void) {
    // UART0：GPIO21 TX -> 飞控 RX，GPIO20 RX <- 飞控 TX，115200
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
    nimble_port_init();
    ble_svc_gap_device_name_set("PavoLED");
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    nimble_port_freertos_init(host_task);

    xTaskCreate(uart_rx_task, "bleuart", 2048, NULL, 5, NULL);
}
