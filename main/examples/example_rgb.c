/*
 * 例程 4: RGB 灯环 (WS2812)
 *
 * 板载 16 颗 WS2812 RGB LED 灯环 (GPIO1), 循环演示:
 *   1. 彩虹流水: 色相沿灯环旋转
 *   2. 全环呼吸: 白色亮度渐亮渐灭
 *   3. 单灯追逐: 红色光点绕环
 */

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "examples.h"

static const char *TAG = "rgb_ring";

#define LED_GPIO     (1)
#define LED_COUNT    (16)
#define STEP_MS      (60)

static led_strip_handle_t s_strip = NULL;

static void strip_init(void)
{
    const led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    const led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
    led_strip_clear(s_strip);
}

/* HSV -> RGB (h: 0~360, s/v: 0~1) */
static void hsv_to_rgb(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float rf = 0, gf = 0, bf = 0;
    if (h < 60)       { rf = c; gf = x; }
    else if (h < 120) { rf = x; gf = c; }
    else if (h < 180) { gf = c; bf = x; }
    else if (h < 240) { gf = x; bf = c; }
    else if (h < 300) { rf = x; bf = c; }
    else              { rf = c; bf = x; }
    *r = (uint8_t)((rf + m) * 255.0f);
    *g = (uint8_t)((gf + m) * 255.0f);
    *b = (uint8_t)((bf + m) * 255.0f);
}

static void rainbow_flow(int loop_ms)
{
    int steps = loop_ms / STEP_MS;
    for (int s = 0; s < steps; s++) {
        for (int i = 0; i < LED_COUNT; i++) {
            float hue = fmodf(360.0f * i / LED_COUNT + 360.0f * s / steps, 360.0f);
            uint8_t r, g, b;
            hsv_to_rgb(hue, 1.0f, 0.17f, &r, &g, &b);
            led_strip_set_pixel(s_strip, i, g, r, b);
        }
        led_strip_refresh(s_strip);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
}

static void white_breath(int loop_ms)
{
    int steps = loop_ms / STEP_MS;
    for (int s = 0; s < steps; s++) {
        float t = (float)s / steps;
        float level = (0.5f - 0.5f * cosf(2.0f * (float)M_PI * t)) * 0.17f;
        uint8_t v = (uint8_t)(level * 255.0f);
        for (int i = 0; i < LED_COUNT; i++) {
            led_strip_set_pixel(s_strip, i, v, v, v);
        }
        led_strip_refresh(s_strip);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
}

static void red_chase(int loop_ms)
{
    int steps = loop_ms / STEP_MS;
    for (int s = 0; s < steps; s++) {
        led_strip_clear(s_strip);
        led_strip_set_pixel(s_strip, s % LED_COUNT, 0, 90, 0);
        led_strip_refresh(s_strip);
        vTaskDelay(pdMS_TO_TICKS(STEP_MS));
    }
}

void example_rgb_start(void)
{
    ESP_LOGI(TAG, "=== RGB 灯环例程 (%d 颗 WS2812 @ GPIO%d) ===", LED_COUNT, LED_GPIO);
    strip_init();

    while (1) {
        ESP_LOGI(TAG, "彩虹流水 8s");
        rainbow_flow(8000);
        ESP_LOGI(TAG, "白色呼吸 8s");
        white_breath(8000);
        ESP_LOGI(TAG, "红色追逐 8s");
        red_chase(8000);
    }
}