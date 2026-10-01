#include "doa_event.h"

#include <math.h>

#define STRONG_SNAP_FRAME_RMS_MIN   (0.0100f)
#define STRONG_SNAP_TRANSIENT_MIN   (0.0300f)
#define STRONG_SNAP_PEAK_RATIO_MIN  (2.25f)
#define STRONG_SNAP_SRP_SCORE_MIN   (0.0250f)

#define WEAK_TAP_FRAME_RMS_MIN      (0.0025f)
#define WEAK_TAP_TRANSIENT_MIN      (0.0100f)
#define WEAK_TAP_PEAK_RATIO_MIN     (1.60f)
#define WEAK_TAP_SRP_SCORE_MIN      (0.0400f)
#define WEAK_TAP_SRP_CONF_MIN       (0.1600f)

bool doa_event_is_strong_snap(float frame_rms, float transient_rms,
                              float peak_ratio, float srp_score)
{
    return isfinite(frame_rms) && isfinite(transient_rms) &&
           isfinite(peak_ratio) && isfinite(srp_score) &&
           transient_rms >= STRONG_SNAP_TRANSIENT_MIN &&
           frame_rms >= STRONG_SNAP_FRAME_RMS_MIN &&
           peak_ratio >= STRONG_SNAP_PEAK_RATIO_MIN &&
           srp_score >= STRONG_SNAP_SRP_SCORE_MIN;
}

bool doa_event_use_fast_path(float frame_rms, float transient_rms,
                             float peak_ratio, float srp_score,
                             float srp_confidence)
{
    if (doa_event_is_strong_snap(frame_rms, transient_rms,
                                 peak_ratio, srp_score)) {
        return true;
    }

    return isfinite(frame_rms) && isfinite(transient_rms) &&
           isfinite(peak_ratio) && isfinite(srp_score) &&
           isfinite(srp_confidence) &&
           frame_rms >= WEAK_TAP_FRAME_RMS_MIN &&
           transient_rms >= WEAK_TAP_TRANSIENT_MIN &&
           peak_ratio >= WEAK_TAP_PEAK_RATIO_MIN &&
           srp_score >= WEAK_TAP_SRP_SCORE_MIN &&
           srp_confidence >= WEAK_TAP_SRP_CONF_MIN;
}
