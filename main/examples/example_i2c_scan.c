/*
 * 例程 1: I2C 总线扫描
 *
 * 依次初始化板上的 I2C 总线, 扫描 0x08~0x77 地址段,
 * 打印所有有应答的设备地址.
 *
 * 引脚速查 (板载/排针):
 *   I2C1 排针: SDA=GPIO11, SCL=GPIO10   (外接传感器)
 *   I2C2 排针: SDA=GPIO14, SCL=GPIO7    (外接传感器)
 *   ES7210 板载: SDA=GPIO8, SCL=GPIO9   (音频 ADC, 地址 0x40~0x43)
 *
 * 板上至少能看到 ES7210 (0x40) 应答; 若在排针外接传感器,
 * 对应总线也会出现新地址.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "examples.h"

static const char *TAG = "i2c_scan";

typedef struct {
    const char *name;
    i2c_port_t port;
    int sda_io;
    int scl_io;
} i2c_bus_t;

/* 三条总线共用两个 I2C 端口: 每次扫描前重新配置引脚 */
static const i2c_bus_t s_buses[] = {
    { "I2C1 排针 (GPIO11/10)", I2C_NUM_0, 11, 10 },
    { "I2C2 排针 (GPIO14/7)",  I2C_NUM_1, 14, 7  },
    { "ES7210 板载 (GPIO8/9)", I2C_NUM_0, 8,  9  },
};

static void scan_bus(i2c_port_t port, const char *name)
{
    printf("\n--- 扫描 %s ---\n", name);
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (cmd == NULL) {
            continue;
        }
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("  发现设备: 0x%02X\n", addr);
            found++;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    printf("  共 %d 个设备应答\n", found);
}

void example_i2c_scan_start(void)
{
    ESP_LOGI(TAG, "=== I2C 总线扫描例程 ===");

    for (size_t i = 0; i < sizeof(s_buses) / sizeof(s_buses[0]); i++) {
        /* 同一端口换引脚前先卸载旧驱动 */
        i2c_driver_delete(s_buses[i].port);

        i2c_config_t cfg = {
            .sda_io_num = s_buses[i].sda_io,
            .scl_io_num = s_buses[i].scl_io,
            .mode = I2C_MODE_MASTER,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = 100000,
        };
        ESP_ERROR_CHECK(i2c_param_config(s_buses[i].port, &cfg));
        ESP_ERROR_CHECK(i2c_driver_install(s_buses[i].port, I2C_MODE_MASTER, 0, 0, 0));

        scan_bus(s_buses[i].port, s_buses[i].name);
    }

    ESP_LOGI(TAG, "扫描完成, 5 秒后重启");
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_restart();
}
