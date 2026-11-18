#pragma once
#include <stdint.h>
#include "driver/i2s_std.h"

/* Diagnostic build only. R starts loss-checked 4ch PCM, S stops it. */
void doa_pcm_stream_start(i2s_chan_handle_t rx, int sample_rate,
                          volatile uint32_t *overflows);
