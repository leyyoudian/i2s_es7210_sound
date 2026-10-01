#ifndef BIRD_DECISION_H
#define BIRD_DECISION_H

#include <stdbool.h>

typedef struct {
    unsigned bird_count;
    unsigned noise_count;
    bool detected;
} bird_decision_t;

void bird_decision_init(bird_decision_t *state);
bool bird_decision_update(bird_decision_t *state,
                          float raw_bird, float raw_noise,
                          float smooth_bird, float smooth_noise);

#endif
