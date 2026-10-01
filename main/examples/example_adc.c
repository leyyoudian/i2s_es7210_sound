/*
 * 例程 2: ADC 电压读取
 *
 * 周期读取板上两路 ADC 引脚并打印电压:
 *   ADC_3V3 排针 = GPIO6 (ADC1_CH5, 直连)
 *   ADC_5V 排针  = GPIO5 (ADC1_CH4, 22k/39k 分压, 需按比例还原)
 *
 * 使用 ESP-IDF 校准 (eFuse 出厂曲线), 输出为毫伏级准确电压.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "examples.h"

static const char *TAG = "adc_read";

typedef struct {
    const char *name;
    int gpio;
    adc_channel_t channel;
    float divider;          /* 实际电压 = 读数 * divider */
} adc_pin_t;

/* 5V 通道经 39k(上)/22k(下) 分压: 比例 = (39+22)/22 ≈ 2.77 */
static const adc_pin_t s_pins[] = {
    { "ADC_3V3 排针 (GPIO6)", 6, ADC_CHANNEL_5, 1.0f },
    { "ADC_5V  排针 (GPIO5)", 5, ADC_CHANNEL_4, (39.0f + 22.0f) / 22.0f },
};

void example_adc_start(void)
{
    ESP_LOGI(TAG, "=== ADC 电压读取例程 ===");

    adc_oneshot_unit_handle_t adc1;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc1));

    adc_cali_handle_t cali = NULL;
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali);

    for (size_t i = 0; i < sizeof(s_pins) / sizeof(s_pins[0]); i++) {
        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, s_pins[i].channel, &chan_cfg));
    }

    while (1) {
        for (size_t i = 0; i < sizeof(s_pins) / sizeof(s_pins[0]); i++) {
            int raw = 0, mv = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1, s_pins[i].channel, &raw));

            float volts = 0.0f;
            if (cali_ret == ESP_OK && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) {
                volts = mv / 1000.0f * s_pins[i].divider;
            } else {
                volts = raw / 4095.0f * 3.3f * s_pins[i].divider;   /* 无校准的粗略换算 */
            }

            printf("%s: raw=%4d  %.3f V\n", s_pins[i].name, raw, volts);
        }
        printf("---\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
