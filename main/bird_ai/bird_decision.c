#include "bird_decision.h"

#include <string.h>

#define CONFIRM_WINDOWS 2U
#define BIRD_THRESHOLD 0.85f
#define BIRD_SMOOTH_THRESHOLD 0.58f
#define BIRD_MARGIN 0.16f
#define BIRD_IMMEDIATE_EXIT_THRESHOLD 0.35f
#define STRONG_NOISE_THRESHOLD 0.85f

void bird_decision_init(bird_decision_t *state)
{
    if (state) memset(state, 0, sizeof(*state));
}

bool bird_decision_update(bird_decision_t *state,
                          float raw_bird, float raw_noise,
                          float smooth_bird, float smooth_noise)
{
    if (!state) return false;

    if (!state->detected) {
        const float raw_margin = raw_bird - raw_noise;
        const float smooth_margin = smooth_bird - smooth_noise;
        bool bird_like = raw_bird >= BIRD_THRESHOLD &&
                         smooth_bird >= BIRD_SMOOTH_THRESHOLD &&
                         raw_margin >= BIRD_MARGIN &&
                         smooth_margin >= 0.0f;
        state->bird_count = bird_like ? state->bird_count + 1U : 0U;
        if (state->bird_count >= CONFIRM_WINDOWS) {
            state->detected = true;
            state->bird_count = 0;
        }
    } else {
        if (raw_noise >= STRONG_NOISE_THRESHOLD ||
            raw_bird <= BIRD_IMMEDIATE_EXIT_THRESHOLD) {
            state->detected = false;
            state->noise_count = 0;
            state->bird_count = 0;
            return false;
        }
        bool noise_like = raw_noise >= 0.70f || smooth_noise >= 0.62f ||
                          smooth_bird <= 0.38f;
        state->noise_count = noise_like ? state->noise_count + 1U : 0U;
        if (state->noise_count >= CONFIRM_WINDOWS) {
            state->detected = false;
            state->noise_count = 0;
        }
    }
    return state->detected;
}
