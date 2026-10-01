# DOA Transient Fast Path Design

## Scope

Restore single-frame localization for finger snaps and similar short impulses
without removing the two-frame protection for ordinary large-angle direction
changes. Bird classification, SRP-PHAT scoring, source thresholds, LED mapping,
and web rendering remain unchanged.

## Root Cause

The audio task detects a transient with energy, peak-ratio, and SRP gates, but
currently passes that result to the tracker only as confidence `0.36`. The
tracker therefore cannot distinguish a valid one-frame impulse from an ordinary
one-frame reflection and applies the ordinary two-frame rule to both.

## Transient Path

Add a separate `doa_tracker_update_transient` entry point. A valid transient
candidate immediately seeds or replaces the held direction and starts a
six-frame reflection guard, approximately 384 ms at 16 kHz with 1024-sample
frames.

While the guard is active:

- another transient candidate is rejected
- an ordinary candidate more than 45 degrees from the held direction is rejected
- an ordinary candidate within 45 degrees may continue through normal smoothing
- every audio frame, including an invalid frame, consumes one guard frame

After the guard expires, ordinary large-angle changes still require two
consistent high-confidence frames. The existing low-confidence confirmation
rules remain unchanged.

## Integration

The audio task calls the transient entry point only when both `transient_doa`
and `localization_valid` are true. All other frames use the existing ordinary
tracker update. This preserves one shared held direction for the LED and web
output.

## Verification

Host tests must prove that a valid transient updates immediately, six following
opposite-direction candidates cannot change the held angle, and ordinary
large-angle movement returns to the existing two-frame confirmation after the
guard. The existing test proving that an ordinary single-frame outlier is
rejected must remain green. All host tests and the ESP-IDF 5.4.2 firmware build
must pass.
