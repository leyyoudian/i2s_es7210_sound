# Detection, LED, and Distance Cleanup Design

## Scope

This change addresses five confirmed behaviors without changing the v2 model,
microphone geometry, SRP-PHAT implementation, DOA calibration, or LED angle map.

## Bird Decision

Keep the existing two-window entry confirmation. Set the entry conditions to:

- raw bird confidence at least `0.68`
- smoothed bird confidence at least `0.58`
- raw bird-minus-noise margin at least `0.16`
- non-negative smoothed bird-minus-noise margin

While the state is bird, a raw bird confidence of `0.35` or lower exits bird
immediately. The existing strong-noise immediate exit remains. Other
noise-like results continue to use the existing consecutive-window exit path.
This rejects the logged `33.2%` result immediately while retaining hysteresis
for borderline model output.

## Web Angle

Display the direction as a normalized unsigned angle in `[0, 360)`. The canvas
pointer continues to consume the calibrated angle unchanged. For example,
`-90 degrees` is displayed as `270 degrees`.

## LED Output

Require three consecutive requests for a different LED before switching from
the current LED. When rendering a new or held LED, first transmit an all-off
frame, then transmit a frame containing only the selected pixel. This makes the
physical strip latch an explicit clear before the next direction and prevents
software state from leaving an old pixel active.

The LED output remains independent of bird classification. Direction mapping,
colors, brightness, and source-presence thresholds are unchanged.

## Distance Removal

Remove the RMS-based distance estimator completely:

- remove the estimator source and header from the firmware
- remove its initialization and audio-loop calculations
- remove distance arguments and JSON from the SSE server
- remove the distance row from the web page
- remove distance-specific tests and build registration

No replacement distance value is exposed. The removed calculation was an
uncalibrated loudness estimate and could not provide a trustworthy distance in
metres. Its CPU cost was negligible, so this cleanup is not expected to
materially change EI inference or DOA throughput.

## Verification

Automated tests must demonstrate that:

- the new midpoint bird thresholds still require two qualifying windows
- a bird state exits immediately at `33.2%` raw bird confidence
- the browser display normalizes negative angles to `[0, 360)`
- distance code, SSE fields, and dashboard elements are absent
- LED switching uses three-frame confirmation and an explicit clear refresh
- all existing direction, tracker, SRP-PHAT, and model-related tests still pass

