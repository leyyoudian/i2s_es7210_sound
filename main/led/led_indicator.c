/* WS2812 sound direction indicator. */

#include "led_indicator.h"

#include "direction_mapping.h"
#include "esp_err.h"
#include "esp_log.h"
#include "led_strip.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "led";
static led_strip_handle_t s_led_strip;
static int s_rendered_pixel = -1;
/* 滑动动画 (受控跑马灯): 目标灯位变化时沿最短圆弧逐帧滑动,
 * 固定角速度, 精确落在目标灯位. 180° 约 170ms 滑完. */
#define LED_SWEEP_STEP_DEG 25.0f
static float s_lit_angle = 0.0f;    /* 当前动画位置角 */
static float s_target_angle = 0.0f; /* 动画目标角 */
static bool  s_animating = false;
/* 大角度跳变直接落位: 矢量 EMA 会把一次性大跳变抹成跨灯位逐颗扫掠,
 * 看起来像跑马灯从旧方向"跑"到新方向. 跳变角超过阈值时直接重置
 * 平滑状态到新方向, 只有小于阈值的连续变化才做平滑. */
static esp_err_t clear_strip(void)
{
    esp_err_t err = led_strip_clear(s_led_strip);
    if (err == ESP_OK) {
        err = led_strip_refresh(s_led_strip);
    }
    return err;
}

static esp_err_t render_single_pixel(int pixel)
{
    esp_err_t err = led_strip_clear(s_led_strip);
    if (err == ESP_OK) {
        err = led_strip_refresh(s_led_strip);
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
        .mem_block_symbols = 0,
        .flags.with_dma = false,
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
    ESP_LOGI(TAG, "WS2812 initialized (GPIO %d, %d pixels)",
             LED_STRIP_GPIO, DIRECTION_LED_COUNT);
}

void led_all_off(void)
{
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
    if (s_led_strip == NULL) {
        return;
    }

    if (!source_active) {
        if (s_rendered_pixel >= 0) {
            s_animating = false;
            led_all_off();
        }
        return;
    }

    float target = angle_deg;
    while (target < 0.0f) target += 360.0f;
    while (target >= 360.0f) target -= 360.0f;
    const int target_pixel = direction_led_nearest_index(target);

    if (!s_animating) {
        if (target_pixel == s_rendered_pixel) {
            return;
        }
        if (s_rendered_pixel < 0) {
            /* 首次点亮: 直接落位, 不做动画 */
            s_lit_angle = target;
            s_target_angle = target;
            if (render_single_pixel(target_pixel) == ESP_OK) {
                s_rendered_pixel = target_pixel;
            }
            return;
        }
        /* 目标灯位变化: 启动滑动动画 (受控跑马灯) */
        s_lit_angle = direction_led_angle_of_index(s_rendered_pixel);
        s_target_angle = target;
        s_animating = true;
    } else {
        /* 动画途中目标又变了: 就地改道 */
        s_target_angle = target;
    }

    /* 沿最短圆弧向目标滑 LED_SWEEP_STEP_DEG/帧, 到位后精确落位 */
    float delta = s_target_angle - s_lit_angle;
    while (delta > 180.0f) delta -= 360.0f;
    while (delta <= -180.0f) delta += 360.0f;
    if (fabsf(delta) <= LED_SWEEP_STEP_DEG) {
        s_lit_angle = s_target_angle;
        s_animating = false;
        if (target_pixel != s_rendered_pixel) {
            if (render_single_pixel(target_pixel) == ESP_OK) {
                s_rendered_pixel = target_pixel;
            }
        }
        return;
    }
    s_lit_angle += (delta > 0.0f ? LED_SWEEP_STEP_DEG : -LED_SWEEP_STEP_DEG);
    while (s_lit_angle < 0.0f) s_lit_angle += 360.0f;
    while (s_lit_angle >= 360.0f) s_lit_angle -= 360.0f;

    const int anim_pixel = direction_led_nearest_index(s_lit_angle);
    if (anim_pixel == s_rendered_pixel) {
        return;   /* 尚未滑入新灯位, 不刷新 */
    }

    esp_err_t err = render_single_pixel(anim_pixel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to update WS2812 pixel %d: %s",
                 anim_pixel, esp_err_to_name(err));
        return;
    }
    s_rendered_pixel = anim_pixel;
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
