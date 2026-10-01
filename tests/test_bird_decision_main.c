#include <stdbool.h>
#include <stdio.h>

#include "bird_decision.h"

int main(void)
{
    bird_decision_t state;
    bird_decision_init(&state);
    if (bird_decision_update(&state, 0.84f, 0.16f, 0.62f, 0.38f)) return 1;
    if (bird_decision_update(&state, 0.84f, 0.16f, 0.66f, 0.34f)) return 2;
    if (bird_decision_update(&state, 0.86f, 0.14f, 0.60f, 0.40f)) return 17;
    if (!bird_decision_update(&state, 0.88f, 0.12f, 0.62f, 0.38f)) return 18;
    if (bird_decision_update(&state, 0.25f, 0.75f, 0.45f, 0.55f)) return 3;
    if (bird_decision_update(&state, 0.20f, 0.80f, 0.35f, 0.65f)) return 4;

    bird_decision_init(&state);
    if (bird_decision_update(&state, 0.94f, 0.06f, 0.40f, 0.60f)) return 5;
    if (bird_decision_update(&state, 0.92f, 0.08f, 0.72f, 0.28f)) return 9;
    if (!bird_decision_update(&state, 0.91f, 0.09f, 0.80f, 0.20f)) return 11;

    bird_decision_init(&state);
    if (bird_decision_update(&state, 0.79f, 0.21f, 0.55f, 0.45f)) return 6;

    bird_decision_init(&state);
    if (bird_decision_update(&state, 0.85f, 0.15f, 0.57f, 0.43f)) return 13;
    if (bird_decision_update(&state, 0.85f, 0.15f, 0.58f, 0.42f)) return 14;
    if (!bird_decision_update(&state, 0.87f, 0.13f, 0.60f, 0.40f)) return 15;
    if (bird_decision_update(&state, 0.332f, 0.668f, 0.55f, 0.45f)) return 16;

    bird_decision_init(&state);
    if (bird_decision_update(&state, 0.94f, 0.06f, 0.80f, 0.20f)) return 7;
    if (!bird_decision_update(&state, 0.91f, 0.09f, 0.82f, 0.18f)) return 10;
    if (bird_decision_update(&state, 0.12f, 0.88f, 0.65f, 0.35f)) return 8;
    return 0;
}
