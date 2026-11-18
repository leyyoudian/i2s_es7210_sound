#include <stdint.h>
#include <stdio.h>

#include "led_indicator.h"
#include "led_strip.h"
#include "doa_display_gate.h"
#include "direction_mapping.h"

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
    return led_strip_refresh(strip);
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
    if (!strip || index >= TEST_LED_COUNT) return ESP_FAIL;
    if (red || green || blue) strip->mask |= 1U << index;
    else strip->mask &= ~(1U << index);
    return ESP_OK;
}

static void reset_history(void)
{
    s_refresh_count = 0;
}

static int expect_refresh_pair(uint32_t expected_pixel_mask, const char *name)
{
    if (s_refresh_count != 1U) {
        fprintf(stderr, "%s: expected 1 refresh, got %u\n", name, s_refresh_count);
        return 1;
    }
    if (s_refresh_history[0] != expected_pixel_mask) {
        fprintf(stderr, "%s: expected 0x%03x, got 0x%03x\n",
                name, expected_pixel_mask, s_refresh_history[0]);
        return 1;
    }
    if ((s_refresh_history[0] & (s_refresh_history[0] - 1U)) != 0U) {
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

    /* Confirmed positions latch directly, without black/intermediate frames. */
    led_update_direction(337.5f, true);
    if (expect_refresh_pair(1U << 1, "direct target")) return 3;
    led_update_direction(337.5f, true);
    if (expect_no_refresh("landed target is not resent")) return 4;

    /* A large accepted jump must never illuminate intermediate directions. */
    led_update_direction(202.5f, true);
    if (expect_refresh_pair(1U << 7, "wide direct target")) return 8;
    for (int i = 0; i < 10; ++i) {
        led_update_direction(202.5f, true);
        if (expect_no_refresh("stable target")) return 9;
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

    /* Integration: the gate's pixel, not its live display angle, drives LEDs.
     * Sweep both edges of every sector, including the 0/360 wrap. */
    for (int pixel = 0; pixel < DIRECTION_LED_COUNT; ++pixel) {
        for (int side = -1; side <= 1; side += 2) {
            doa_display_gate_t gate;
            float shown;
            float center = direction_led_angle_of_index(pixel);
            doa_display_gate_init(&gate);
            led_all_off();
            reset_history();
            doa_display_gate_update(&gate, true, true, true, .01f, center, &shown);
            led_update_direction(direction_led_angle_of_index(gate.held_pixel), true);
            if (expect_refresh_pair(1U << pixel, "sector initial")) return 10;
            for (int frame = 0; frame < 20; ++frame) {
                float a = center + side * (frame % 2 ? 13.5f : 10.0f);
                a = a < 0 ? a + 360.0f : (a >= 360 ? a - 360.0f : a);
                doa_display_gate_update(&gate, true, true, true, .01f, a, &shown);
                led_update_direction(direction_led_angle_of_index(gate.held_pixel), true);
                if (expect_no_refresh("boundary jitter bypassed hysteresis")) return 11;
            }
        }
    }

    return 0;
}
