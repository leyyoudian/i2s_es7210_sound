/*
 * 例程 3: UART 双路互通
 *
 * 板上两路 UART 排针交叉互连 (需短接或已在 PCB 上交叉):
 *   UART1: TX=GPIO12, RX=GPIO13
 *   UART2: TX=GPIO47, RX=GPIO48
 *   交叉连接: UART1.TX(GPIO12) -> UART2.RX(GPIO48)
 *             UART2.TX(GPIO47) -> UART1.RX(GPIO13)
 *
 * UART1 每秒发送 "hello", UART2 每秒发送 "world",
 * 双方互相接收并打印对方的数据, 形成回环演示.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "examples.h"

static const char *TAG = "uart_demo";

#define UART1_PORT   (UART_NUM_1)
#define UART1_TX_IO  (12)
#define UART1_RX_IO  (13)

#define UART2_PORT   (UART_NUM_2)
#define UART2_TX_IO  (47)
#define UART2_RX_IO  (48)

#define BAUD         (115200)
#define BUF_SIZE     (256)

static void uart_init_port(uart_port_t port, int tx_io, int rx_io)
{
    const uart_config_t cfg = {
        .baud_rate = BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_io, rx_io,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void example_uart_start(void)
{
    ESP_LOGI(TAG, "=== UART 双路互通例程 (波特率 %d) ===", BAUD);
    uart_init_port(UART1_PORT, UART1_TX_IO, UART1_RX_IO);
    uart_init_port(UART2_PORT, UART2_TX_IO, UART2_RX_IO);

    const char *tx1 = "hello from UART1\r\n";
    const char *tx2 = "world from UART2\r\n";

    while (1) {
        uart_write_bytes(UART1_PORT, tx1, strlen(tx1));
        uart_write_bytes(UART2_PORT, tx2, strlen(tx2));
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint8_t data[BUF_SIZE];
        int len1 = uart_read_bytes(UART2_PORT, data, sizeof(data) - 1, 0);
        if (len1 > 0) {
            data[len1] = '\0';
            printf("UART2 收到: %s", (char *)data);
        }
        int len2 = uart_read_bytes(UART1_PORT, data, sizeof(data) - 1, 0);
        if (len2 > 0) {
            data[len2] = '\0';
            printf("UART1 收到: %s", (char *)data);
        }
    }
}
