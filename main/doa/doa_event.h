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
/* A crest factor alone is not an onset: steady music has peaks every frame. */
bool doa_event_needs_instant_analysis(bool source_active, float frame_rms,
                                     float previous_frame_rms, float transient_rms,
                                     float previous_transient_rms, float noise_rms);
bool doa_event_can_lock_single_frame(bool has_direction, float band_rms,
                                     float noise_rms);
bool doa_event_has_weak_spatial_evidence(bool srp_valid, float band_rms,
                                         float noise_rms, float score,
                                         float zero_delay_score, float confidence,
                                         float mirror_ratio);

#ifdef __cplusplus
}
#endif

#endif /* DOA_EVENT_H */
