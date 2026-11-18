#include <stdio.h>

#include "doa_event.h"

int main(void)
{
    if (!doa_event_use_fast_path(0.0035f, 0.0120f, 1.70f, 0.130f, 0.180f)) {
        fprintf(stderr, "weak tap did not qualify for fast path\n");
        return 1;
    }

    if (doa_event_use_fast_path(0.0060f, 0.0066f, 1.10f, 0.200f, 0.220f)) {
        fprintf(stderr, "steady weak sound incorrectly qualified\n");
        return 2;
    }

    if (doa_event_use_fast_path(0.0045f, 0.0150f, 1.80f, 0.085f, 0.120f)) {
        fprintf(stderr, "low-quality reflection incorrectly qualified\n");
        return 3;
    }

    if (doa_event_use_fast_path(.02f, .08f, 3.0f, .10f, .02f)) {
        fprintf(stderr, "loud ambiguous snap bypassed confidence check\n");
        return 4;
    }
    /* Ongoing music can have a 2x block/frame crest without a new onset. */
    for (int i = 0; i < 100; ++i) {
        float frame = .005f + .001f * (float)(i % 2);
        if (doa_event_needs_instant_analysis(true, frame, .005f,
                                            .012f, .011f, .0008f)) return 5;
    }
    if (!doa_event_needs_instant_analysis(false, .002f, .001f,
                                          .005f, .002f, .0008f)) return 6;
    if (!doa_event_needs_instant_analysis(true, .020f, .004f,
                                          .060f, .006f, .0008f)) return 7;
    /* Measured weak rain outlier: SNR~2.1, high apparent confidence, one frame.
     * It must not select the first direction; continued evidence remains
     * eligible for the ordinary multi-frame tracking path. */
    if (doa_event_can_lock_single_frame(false,.0015f,.0007f)) return 8;
    if (!doa_event_can_lock_single_frame(true,.0015f,.0007f)) return 9;
    if (!doa_event_can_lock_single_frame(false,.0040f,.0007f)) return 10;
    if (!doa_event_has_weak_spatial_evidence(true,.001f,.0007f,.03f,.02f,.3f,.3f)) return 11;
    if (doa_event_has_weak_spatial_evidence(true,.001f,.0007f,.03f,.0298f,.3f,.3f)) return 12;
    if (doa_event_has_weak_spatial_evidence(true,.001f,.0007f,.03f,.02f,.3f,.8f)) return 13;
    return 0;
}
