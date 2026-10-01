# DOA Single-Frame Outlier Rejection Design

## Scope

This change addresses only unstable direction output and brief flashes of an
unrelated direction LED. It does not change SRP-PHAT scoring, microphone or LED
geometry, bird classification, web display, source activity thresholds, or LED
pixel mapping.

## Root Cause

The tracker currently commits any direction movement greater than 45 degrees
immediately when confidence is at least 0.35. A single strong reflection can
therefore replace a stable direction. The periodic log prints only every 16
audio frames, so the accepted outlier can occur between two printed rejected
frames. The tracker then holds that incorrect angle long enough for the LED's
existing switch confirmation to display the unrelated pixel briefly.

## Tracking Behavior

A high-confidence direction movement greater than 45 degrees must produce two
consecutive valid candidates before it replaces the held direction. The two
candidates must remain within the existing 20-degree pending spread. Until the
second candidate arrives, the tracker returns the previous held direction.

An invalid frame or a candidate outside the pending spread must not publish the
pending angle. Existing pending expiry behavior remains in effect. Existing
low-confidence confirmation rules remain unchanged.

This adds one 1024-sample frame, approximately 64 ms at 16 kHz, to a real
high-confidence large-angle transition. It avoids the earlier six-frame delay
for high-confidence opposite transitions.

## Verification

The host tracker test must prove that:

- one high-confidence candidate near the opposite direction does not change a
  stable held direction
- an invalid frame after that outlier still leaves the held direction unchanged
- two consecutive high-confidence candidates within 20 degrees commit the new
  direction on the second frame
- existing low-confidence, wraparound, and pending-expiry behavior still passes

All host tests and the ESP-IDF 5.4.2 firmware build must pass. Board testing
should use a stationary airborne source and then a deliberate large-angle move;
the stationary source must not produce a brief unrelated LED, while the real
move should switch after two consistent localization frames.
