#ifndef TEST_FAKE_LED_STRIP_H
#define TEST_FAKE_LED_STRIP_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct fake_led_strip *led_strip_handle_t;

typedef struct {
    int strip_gpio_num;
    uint32_t max_leds;
    int led_pixel_format;
    int led_model;
    struct {
        bool invert_out;
    } flags;
} led_strip_config_t;

typedef struct {
    int clk_src;
    uint32_t resolution_hz;
    uint32_t mem_block_symbols;
    struct {
        bool with_dma;
    } flags;
} led_strip_rmt_config_t;

#define LED_PIXEL_FORMAT_GRB 1
#define LED_MODEL_WS2812 1
#define RMT_CLK_SRC_DEFAULT 1

esp_err_t led_strip_new_rmt_device(const led_strip_config_t *strip_config,
                                   const led_strip_rmt_config_t *rmt_config,
                                   led_strip_handle_t *ret_strip);
esp_err_t led_strip_clear(led_strip_handle_t strip);
esp_err_t led_strip_refresh(led_strip_handle_t strip);
esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index,
                              uint32_t red, uint32_t green, uint32_t blue);

#endif
