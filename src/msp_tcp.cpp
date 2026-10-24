#include "msp_tcp.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>

// 与 ble_uart.cpp 用同一个物理串口（UART1，GPIO6/7），BLE 与 TCP 共用一条 MSP 通道
#define MSP_UART      UART_NUM_1
#define MSP_TCP_PORT  5761   // Betaflight Configurator 的 TCP MSP 约定端口

static volatile int s_fd = -1;        // 当前 TCP 客户端（-1 = 无）
static volatile int s_rx = 0;         // 客户端 -> 飞控（字节）
static volatile int s_tx = 0;         // 飞控 -> 客户端（字节）
static volatile int s_active = 0;     // 累计接入次数（用于心跳观察）
static SemaphoreHandle_t s_lock = NULL;

int mspTcpConnected(void) { return s_fd >= 0 ? 1 : 0; }

int mspTcpSend(const uint8_t* p, int n) {
    if (!p || n <= 0) return 0;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    int fd = s_fd;
    int r = 0;
    if (fd >= 0) {
        // 非阻塞发送：MSP 是小包，正常一次就发完；对端不收时不要卡住 UART 任务
#ifdef MSG_DONTWAIT
        r = send(fd, p, n, MSG_DONTWAIT);
#else
        r = send(fd, p, n, 0);
#endif
        if (r > 0) s_tx += r;
    }
    if (s_lock) xSemaphoreGive(s_lock);
    return r;
}

const char* mspTcpDiag(void) {
    static char b[2][48];
    static volatile int idx = 0;
    int i = idx;
    idx = 1 - i;
    snprintf(b[i], sizeof(b[i]), "tcp=%d#%d rx=%d tx=%d",
             (s_fd >= 0) ? 1 : 0, s_active, s_rx, s_tx);
    return b[i];
}

static void tcp_srv_task(void* arg) {
    (void)arg;
    uint8_t buf[512];
    for (;;) {
        int ls = socket(AF_INET, SOCK_STREAM, 0);
        if (ls < 0) {
            printf("[TCP] socket 失败 errno=%d，2s 后重试\n", errno);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        int yes = 1;
        setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
        struct sockaddr_in a;
        memset(&a, 0, sizeof(a));
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_ANY);
        a.sin_port = htons(MSP_TCP_PORT);
        if (bind(ls, (struct sockaddr*)&a, sizeof(a)) != 0 || listen(ls, 2) != 0) {
            // WiFi 还没起来时这里会一直失败，重试即可
            close(ls);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        printf("[TCP] MSP 服务已监听 %d（BF Configurator 填 tcp://192.168.4.1）\n", MSP_TCP_PORT);

        for (;;) {
            struct sockaddr_in ca;
            socklen_t cl = sizeof(ca);
            int c = accept(ls, (struct sockaddr*)&ca, &cl);
            if (c < 0) {
                // WiFi 掉了/网卡复位：整个重来
                close(ls);
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }
            int one = 1;
#ifdef TCP_NODELAY
            setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
#endif
            if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
            int old = s_fd;
            s_fd = c;
            s_active++;
            if (s_lock) xSemaphoreGive(s_lock);
            if (old >= 0) {
                // 只保留最新一个客户端，旧连接踢掉（BF Configurator 重连会这样）
                shutdown(old, 0);
                close(old);
            }
            printf("[TCP] 客户端接入 %s:%u\n",
                   inet_ntoa(ca.sin_addr), (unsigned)ntohs(ca.sin_port));

            for (;;) {
                int n = recv(c, buf, sizeof(buf), 0);
                if (n <= 0) break;
                uart_write_bytes(MSP_UART, buf, n); // -> 飞控
                s_rx += n;
            }

            if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
            if (s_fd == c) s_fd = -1;
            if (s_lock) xSemaphoreGive(s_lock);
            close(c);
            printf("[TCP] 客户端断开（累计 rx=%d tx=%d）\n", s_rx, s_tx);
        }
    }
}

void mspTcpInit(void) {
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    // UART 由 bleUartInit() 初始化；这里只确保任务在 UART 就绪后再起
    xTaskCreate(tcp_srv_task, "mspTcp", 4096, NULL, 4, NULL);
}
