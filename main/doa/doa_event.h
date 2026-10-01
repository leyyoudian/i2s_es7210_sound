#ifndef DOA_EVENT_H
#define DOA_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

bool doa_event_use_fast_path(float frame_rms, float transient_rms,
                             float peak_ratio, float srp_score,
                             float srp_confidence);
bool doa_event_is_strong_snap(float frame_rms, float transient_rms,
                              float peak_ratio, float srp_score);

#ifdef __cplusplus
}
#endif

#endif /* DOA_EVENT_H */
