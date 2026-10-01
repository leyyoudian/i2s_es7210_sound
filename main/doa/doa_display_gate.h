#ifndef DOA_DISPLAY_GATE_H
#define DOA_DISPLAY_GATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    unsigned stale_weak_reject_frames;
    bool display_active;
    int held_pixel;
    int candidate_pixel;
    unsigned candidate_count;
    float display_angle;
} doa_display_gate_t;

void doa_display_gate_init(doa_display_gate_t *gate);
bool doa_display_gate_update(doa_display_gate_t *gate, bool tracker_updated,
                             bool source_active, bool localization_valid,
                             float doa_energy, float angle_deg,
                             float *display_angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* DOA_DISPLAY_GATE_H */
