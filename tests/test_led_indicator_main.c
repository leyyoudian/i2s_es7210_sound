#include <stdint.h>
#include <stdio.h>

#include "led_indicator.h"
#include "led_strip.h"

#define TEST_LED_COUNT 16U
#define MAX_REFRESHES 32U

struct fake_led_strip {
    uint32_t mask;
};

static struct fake_led_strip s_strip;
static uint32_t s_refresh_history[MAX_REFRESHES];
static unsigned s_refresh_count;

esp_err_t led_strip_new_rmt_device(const led_strip_config_t *strip_config,
                                   const led_strip_rmt_config_t *rmt_config,
                                   led_strip_handle_t *ret_strip)
{
    (void)rmt_config;
    if (!strip_config || !ret_strip || strip_config->max_leds != TEST_LED_COUNT) {
        return ESP_FAIL;
    }
    s_strip.mask = 0;
    *ret_strip = &s_strip;
    return ESP_OK;
}

esp_err_t led_strip_clear(led_strip_handle_t strip)
{
    if (!strip) return ESP_FAIL;
    strip->mask = 0;
    return ESP_OK;
}

esp_err_t led_strip_refresh(led_strip_handle_t strip)
{
    if (!strip || s_refresh_count >= MAX_REFRESHES) return ESP_FAIL;
    s_refresh_history[s_refresh_count++] = strip->mask;
    return ESP_OK;
}

esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index,
                              uint32_t red, uint32_t green, uint32_t blue)
{
    (void)red;
    (void)green;
    (void)blue;
    if (!strip || index >= TEST_LED_COUNT) return ESP_FAIL;
    strip->mask |= 1U << index;
    return ESP_OK;
}

static void reset_history(void)
{
    s_refresh_count = 0;
}

static int expect_refresh_pair(uint32_t expected_pixel_mask, const char *name)
{
    if (s_refresh_count != 2U) {
        fprintf(stderr, "%s: expected 2 refreshes, got %u\n", name, s_refresh_count);
        return 1;
    }
    if (s_refresh_history[0] != 0U ||
        s_refresh_history[1] != expected_pixel_mask) {
        fprintf(stderr, "%s: expected [0, 0x%03x], got [0x%03x, 0x%03x]\n",
                name, expected_pixel_mask,
                s_refresh_history[0], s_refresh_history[1]);
        return 1;
    }
    if ((s_refresh_history[1] & (s_refresh_history[1] - 1U)) != 0U) {
        fprintf(stderr, "%s: non-zero refresh contains multiple pixels\n", name);
        return 1;
    }
    reset_history();
    return 0;
}

static int expect_no_refresh(const char *name)
{
    if (s_refresh_count != 0U) {
        fprintf(stderr, "%s: expected no refreshes, got %u\n",
                name, s_refresh_count);
        return 1;
    }
    return 0;
}

static int expect_clear_once(const char *name)
{
    if (s_refresh_count != 1U) {
        fprintf(stderr, "%s: expected 1 refresh, got %u\n",
                name, s_refresh_count);
        return 1;
    }
    if (s_refresh_history[0] != 0U) {
        fprintf(stderr, "%s: expected clear refresh, got 0x%03x\n",
                name, s_refresh_history[0]);
        return 1;
    }
    reset_history();
    return 0;
}

int main(void)
{
    led_init();
    reset_history();

    /* 首次点亮直接落位 */
    led_update_direction(22.5f, true);
    if (expect_refresh_pair(1U << 15, "initial direction")) return 1;

    led_update_direction(23.0f, true);
    if (expect_no_refresh("same direction is not resent")) return 2;

    /* 22.5 -> 337.5 (45°): 滑动动画逐帧经过最短圆弧, 精确落位 */
    led_update_direction(337.5f, true);
    if (expect_refresh_pair(1U << 0, "sweep intermediate pixel")) return 3;
    led_update_direction(337.5f, true);
    if (expect_refresh_pair(1U << 1, "sweep lands on target")) return 8;
    led_update_direction(337.5f, true);
    if (expect_no_refresh("landed target is not resent")) return 4;

    /* 337.5 -> 202.5 (135°): 逐帧滑动, 每帧恰好单灯, 最终落位 */
    for (int i = 0; i < 10; ++i) {
        led_update_direction(202.5f, true);
        if (s_refresh_count == 0U) continue;   /* 未跨入新灯位则不刷新 */
        if (s_refresh_count != 2U || s_refresh_history[0] != 0U) {
            fprintf(stderr, "sweep frame refresh pattern wrong\n");
            return 9;
        }
        uint32_t m = s_refresh_history[1];
        if (m == 0U || (m & (m - 1U)) != 0U) {
            fprintf(stderr, "sweep frame must light exactly one pixel\n");
            return 9;
        }
        reset_history();
    }
    if (s_strip.mask != (1U << 7)) {
        fprintf(stderr, "wide sweep did not land on pixel 7: mask=0x%03x\n",
                s_strip.mask);
        return 5;
    }
    reset_history();

    led_update_direction(202.5f, false);
    if (expect_clear_once("inactive clears lit pixel")) return 6;

    led_update_direction(202.5f, false);
    if (expect_no_refresh("inactive state is not repeatedly cleared")) return 7;

    return 0;
}
