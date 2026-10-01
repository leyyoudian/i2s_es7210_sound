#include <stdio.h>

#include "direction_mapping.h"

typedef struct {
    float angle;
    int expected;
} mapping_case_t;

int main(void)
{
    static const mapping_case_t calibration_cases[] = {
        {0.0f, 180}, {90.0f, 90}, {180.0f, 0}, {270.0f, 270},
    };
    for (unsigned int i = 0;
         i < sizeof(calibration_cases) / sizeof(calibration_cases[0]); ++i) {
        const float actual = direction_calibrate_doa(calibration_cases[i].angle);
        float error = actual - (float)calibration_cases[i].expected;
        if (error < 0.0f) error = -error;
        if (error > 0.1f) {
            fprintf(stderr, "board calibration %.1f: expected %d, got %.1f\n",
                    calibration_cases[i].angle, calibration_cases[i].expected,
                    actual);
            return 2;
        }
    }

    static const mapping_case_t cases[] = {
        {0.0f, 0}, {337.5f, 1}, {315.0f, 2}, {292.5f, 3}, {270.0f, 4},
        {247.5f, 5}, {225.0f, 6}, {202.5f, 7}, {180.0f, 8}, {157.5f, 9},
        {135.0f, 10}, {112.5f, 11}, {90.0f, 12}, {67.5f, 13}, {45.0f, 14},
        {22.5f, 15},
        {-11.25f, 0}, {11.25f, 0}, {348.75f, 0}, {326.25f, 1},
        {382.5f, 15}, {270.0f, 4}, {180.0f, 8}, {90.0f, 12},
    };

    for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const int actual = direction_led_nearest_index(cases[i].angle);
        if (actual != cases[i].expected) {
            fprintf(stderr, "%.1f degrees: expected %d, got %d\n",
                    cases[i].angle, cases[i].expected, actual);
            return 1;
        }
    }

    /* 灯位->角度回查: 与最近邻映射互为反函数 */
    for (int i = 0; i < 16; ++i) {
        const float angle = direction_led_angle_of_index(i);
        if (direction_led_nearest_index(angle) != i) {
            fprintf(stderr, "pixel %d roundtrip failed at %.1f degrees\n",
                    i, angle);
            return 3;
        }
    }
    if (direction_led_angle_of_index(-1) != 0.0f ||
        direction_led_angle_of_index(16) != 0.0f) {
        fprintf(stderr, "out-of-range pixel index must fall back to 0 deg\n");
        return 4;
    }

    return 0;
}
