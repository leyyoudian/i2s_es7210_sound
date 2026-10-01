#ifndef DIRECTION_MAPPING_H
#define DIRECTION_MAPPING_H

#ifdef __cplusplus
extern "C" {
#endif

#define DIRECTION_LED_COUNT (16)

float direction_calibrate_doa(float raw_angle_deg);
int direction_led_nearest_index(float angle_deg);
float direction_led_angle_of_index(int index);

#ifdef __cplusplus
}
#endif

#endif /* DIRECTION_MAPPING_H */
