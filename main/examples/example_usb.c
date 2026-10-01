/*
 * 例程 6: USB 串口 (USB Serial/JTAG 回显)
 *
 * ESP32-S3 内置 USB Serial/JTAG 控制器 (物理引脚 GPIO19=D-, GPIO20=D+,
 * 经板上 USB-C 口引出), 用一根 USB-C 线即可与 PC 通信,
 * 无需外部 USB-UART 芯片.
 *
 * 引脚速查:
 *   USB D- = GPIO19, USB D+ = GPIO20 (芯片内部已连接, 无需接线)
 *   串口参数: 115200 8N1
 *
 * 本例程把 USB CDC 口注册为 VFS 设备, 循环回显收到的每一行,
 * 并演示 printf 直接从 USB 口输出.
 *
 * 使用: USB 线连接板上 USB 口 -> PC 打开串口终端 (115200 8N1)
 *       输入任意文本, 板子原样回显.
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "examples.h"

static const char *TAG = "usb_demo";

void example_usb_start(void)
{
    ESP_LOGI(TAG, "=== USB Serial/JTAG 回显例程 ===");

    /* 注册 USB Serial/JTAG 为 VFS 设备并启用驱动缓冲 */
    ESP_ERROR_CHECK(usb_serial_jtag_vfs_register());
    usb_serial_jtag_vfs_use_driver();
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    printf("\nUSB 串口已就绪. 输入任意文本并以回车结束, 板子将回显.\n");
    printf("输入 quit 可退出例程并重启.\n\n");

    char line[256];
    while (1) {
        printf("usb> ");
        fflush(stdout);

        /* 阻塞读一行 (fgets 走 VFS -> USB CDC) */
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* 去掉行尾换行 */
        line[strcspn(line, "\r\n")] = '\0';
        if (strlen(line) == 0) {
            continue;
        }

        if (strcmp(line, "quit") == 0) {
            printf("收到 quit, 5 秒后重启...\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_restart();
        }

        printf("  回显: [%s]  (长度 %d)\n", line, (int)strlen(line));
    }
}
