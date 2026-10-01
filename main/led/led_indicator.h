#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <stdbool.h>

#define LED_STRIP_GPIO            (1)     /* WS2812 灯环数据脚 */
#define LED_DIRECTION_BRIGHTNESS  (60)    /* 方向灯亮度 0-255 (太亮可降到 10-30) */

void led_init(void);
void led_update_direction(float angle_deg, bool source_active);
void led_update_detection(bool detected);
void led_all_off(void);

#endif /* LED_INDICATOR_H */