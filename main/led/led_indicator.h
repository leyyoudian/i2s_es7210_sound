#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdbool.h>
#include "led_effects.h"

#define LED_STRIP_GPIO            (1)     /* WS2812 灯环数据脚 */
#define LED_DIRECTION_BRIGHTNESS  (60)    /* 方向灯亮度 0-255 (太亮可降到 10-30) */

void led_init(void);
/* 在 DOA 模式启动 Core 0 独立 50 Hz 渲染任务。I2S 处理只复制最新场景，
 * 不等待 RMT 发送；不让呼吸动画、刷新或日志增加音频采集延迟。 */
void led_start_effects(void);
void led_submit_scene(const led_effect_scene_t *scene, uint32_t now_ms);
void led_update_direction(float angle_deg, bool source_active);
void led_update_detection(bool detected);
void led_all_off(void);

#endif /* LED_INDICATOR_H */
