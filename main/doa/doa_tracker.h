#ifndef DOA_TRACKER_H
#define DOA_TRACKER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    float x;
    float y;
    float held_angle;
    /* α-β 跟踪器速率状态: 单位弧度/帧, 沿当前方向矢量切线. */
    float angular_rate;
    float pending_x;
    float pending_y;
    float min_confidence;
    unsigned pending_count;
    unsigned pending_misses;
    unsigned transient_guard_frames;
    bool pending_high_confidence_jump;
    bool pending_opposite_jump;
    bool initialized;
} doa_tracker_t;

void doa_tracker_init(doa_tracker_t *tracker, float min_confidence);
bool doa_tracker_update_transient(doa_tracker_t *tracker, float angle_deg,
                                  float confidence, bool candidate_valid,
                                  float *held_angle);
bool doa_tracker_update(doa_tracker_t *tracker, float angle_deg, float confidence,
                        bool candidate_valid, float *held_angle);

#ifdef __cplusplus
}
#endif
#endif
