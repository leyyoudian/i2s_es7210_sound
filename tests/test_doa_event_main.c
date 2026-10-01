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

    return 0;
}
