/* WS2812 sound direction indicator. */

#include "led_indicator.h"

#include "direction_mapping.h"
#include "esp_err.h"
#include "esp_log.h"
#include "led_strip.h"

#include <math.h>
#include <string.h>
#ifdef ESP_PLATFORM
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#endif

static const char *TAG = "led";
static led_strip_handle_t s_led_strip;
static int s_rendered_pixel = -1;
#ifdef ESP_PLATFORM
static portMUX_TYPE s_scene_lock = portMUX_INITIALIZER_UNLOCKED;
static led_effect_scene_t s_latest_scene;
static uint32_t s_scene_ms;
static bool s_effects_started;
static bool s_force_off;

static void effects_task(void *arg)
{
    (void)arg;
    led_effects_t effects;
    led_effects_init(&effects);
    led_effect_rgb_t previous[DIRECTION_LED_COUNT] = {{0}};
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        led_effect_scene_t scene;
        uint32_t input_ms;
        bool force_off;
        portENTER_CRITICAL(&s_scene_lock);
        scene = s_latest_scene;
        input_ms = s_scene_ms;
        force_off = s_force_off;
        s_force_off = false;
        portEXIT_CRITICAL(&s_scene_lock);
        if (force_off) led_effects_init(&effects);
        if ((uint32_t)(now - input_ms) > LED_EFFECT_STALE_MS) scene.count = 0;
        led_effect_rgb_t pixels[DIRECTION_LED_COUNT];
        led_effects_render(&effects, &scene, now, pixels);
        if (memcmp(previous, pixels, sizeof(pixels))) {
            esp_err_t err = ESP_OK;
            for (int i = 0; i < DIRECTION_LED_COUNT && err == ESP_OK; ++i) {
                err = led_strip_set_pixel(s_led_strip, i, pixels[i].r, pixels[i].g, pixels[i].b);
            }
            if (err == ESP_OK) err = led_strip_refresh(s_led_strip);
            /* 失败时保留 previous，下一帧重试；不能假装已渲染成功。 */
            if (err == ESP_OK) memcpy(previous, pixels, sizeof(pixels));
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LED_EFFECT_FRAME_MS));
    }
}
#endif

void led_start_effects(void)
{
#ifdef ESP_PLATFORM
    if (!s_led_strip || s_effects_started) return;
    memset(&s_latest_scene, 0, sizeof(s_latest_scene));
    s_scene_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_effects_started = xTaskCreatePinnedToCore(effects_task, "led_effects", 4096,
                                               NULL, 2, NULL, 0) == pdPASS;
    if (!s_effects_started) ESP_LOGE(TAG, "LED effects task allocation failed");
#endif
}

void led_submit_scene(const led_effect_scene_t *scene, uint32_t now_ms)
{
#ifdef ESP_PLATFORM
    if (!scene || !s_effects_started) return;
    portENTER_CRITICAL(&s_scene_lock);
    s_latest_scene = *scene;
    if (s_latest_scene.count > LED_EFFECT_MAX_SOURCES) s_latest_scene.count = LED_EFFECT_MAX_SOURCES;
    s_scene_ms = now_ms;
    portEXIT_CRITICAL(&s_scene_lock);
#else
    (void)scene; (void)now_ms;
#endif
}
static esp_err_t clear_strip(void)
{
    /* led_strip_clear already transmits a frame in the RMT backend. */
    return led_strip_clear(s_led_strip);
}

static esp_err_t render_single_pixel(int pixel)
{
    /* Update the buffer, then latch ONCE: no intervening all-black frame. */
    esp_err_t err = ESP_OK;
    for (int i = 0; i < DIRECTION_LED_COUNT && err == ESP_OK; ++i) {
        err = led_strip_set_pixel(s_led_strip, i, 0, 0, 0);
    }
    if (err == ESP_OK) {
        err = led_strip_set_pixel(s_led_strip, pixel,
                                  LED_DIRECTION_BRIGHTNESS, 0, 0);
    }
    if (err == ESP_OK) {
        err = led_strip_refresh(s_led_strip);
    }
    return err;
}

void led_init(void)
{
    const led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = DIRECTION_LED_COUNT,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
#if defined(ESP_PLATFORM) && SOC_RMT_SUPPORT_DMA && LED_STRIP_USE_DMA
        .mem_block_symbols = LED_STRIP_DMA_SYMBOLS,
        .flags.with_dma = true,
#else
        .mem_block_symbols = 0,
        .flags.with_dma = false,
#endif
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config,
                                              &s_led_strip);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 init failed on GPIO %d: %s",
                 LED_STRIP_GPIO, esp_err_to_name(err));
        s_led_strip = NULL;
        return;
    }

    led_all_off();
    ESP_LOGI(TAG, "WS2812 initialized (GPIO %d, %d pixels, DMA=%d, symbols=%u)",
             LED_STRIP_GPIO, DIRECTION_LED_COUNT, (int)rmt_config.flags.with_dma,
             (unsigned)rmt_config.mem_block_symbols);
}

void led_all_off(void)
{
#ifdef ESP_PLATFORM
    if (s_effects_started) {
        /* 动态模式由唯一渲染任务操作RMT；下一个20ms刷新周期强制清屏，
         * 不能从音频/其它任务并发操作同一条灯带。 */
        portENTER_CRITICAL(&s_scene_lock);
        s_latest_scene.count = 0;
        s_force_off = true;
        portEXIT_CRITICAL(&s_scene_lock);
        return;
    }
#endif
    if (s_led_strip == NULL) {
        return;
    }

    esp_err_t err = clear_strip();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear WS2812 strip: %s", esp_err_to_name(err));
        return;
    }
    s_rendered_pixel = -1;
}

void led_update_direction(float angle_deg, bool source_active)
{
#ifdef ESP_PLATFORM
    if (s_effects_started) {
        led_effect_scene_t scene = {.count = source_active && isfinite(angle_deg) ? 1U : 0U};
        scene.sources[0] = (led_effect_source_t){angle_deg, .5f};
        led_submit_scene(&scene, (uint32_t)(esp_timer_get_time() / 1000));
        return;
    }
#endif
    if (s_led_strip == NULL) {
        return;
    }

    if (!source_active) {
        if (s_rendered_pixel >= 0) {
            led_all_off();
        }
        return;
    }

    if (!isfinite(angle_deg)) return;
    const int pixel = direction_led_nearest_index(angle_deg);
    if (pixel == s_rendered_pixel) return;
    esp_err_t err = render_single_pixel(pixel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to update WS2812 pixel %d: %s",
                 pixel, esp_err_to_name(err));
        return;
    }
    s_rendered_pixel = pixel;
}
/* 音色识别模式指示: 检测到目标 -> 全环点亮, 否则熄灭 (状态变化才刷新) */
void led_update_detection(bool detected)
{
    if (s_led_strip == NULL) {
        return;
    }

    static bool s_shown = false;
    if (detected == s_shown) {
        return;
    }
    s_shown = detected;

    esp_err_t err = ESP_OK;
    if (detected) {
        for (int i = 0; i < DIRECTION_LED_COUNT && err == ESP_OK; ++i) {
            err = led_strip_set_pixel(s_led_strip, i,
                                      LED_DIRECTION_BRIGHTNESS, 0, 0);
        }
        if (err == ESP_OK) {
            err = led_strip_refresh(s_led_strip);
        }
    } else {
        err = clear_strip();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to update detection LEDs: %s", esp_err_to_name(err));
    }
}
