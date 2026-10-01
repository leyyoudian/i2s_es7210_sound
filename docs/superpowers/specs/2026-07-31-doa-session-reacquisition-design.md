# DOA Session Reacquisition Design

## Scope

This change fixes stale direction reuse and transient-reflection jumps without
changing microphone geometry, SRP-PHAT scoring, direction calibration, LED
mapping, source thresholds, or the v2 model.

## Direction Session State

Add a small direction-session controller around the existing DOA tracker. A
direction is fresh only after the current sound-source session has produced an
accepted localization result. The LED may use an angle only while that fresh
flag is set.

When the source becomes inactive, reset the tracker immediately and clear the
fresh flag. When a source becomes active again, the previous held angle must
not be displayed; the normal tracker initialization must accept new frames
before the LED can turn on.

At 16 kHz with 1024-sample localization frames, ten consecutive frames are
approximately 640 ms. If an active source produces ten consecutive frames
without a valid localization candidate, reset the tracker, clear direction
freshness, and turn off the LED. A valid candidate resets this miss counter,
including candidates that are still accumulating confirmation frames.

## Transient Handling

The first valid transient peak may reseed the tracker immediately so a finger
snap remains responsive. After accepting that peak, hold the direction for six
frames, approximately 384 ms. During this lock, do not feed later transient or
ordinary candidates into the tracker; this prevents the first table or wall
reflection from replacing the direct-path angle.

Source inactivity cancels the transient lock.

## Ordinary Large Jumps

Remove the one-frame high-confidence large-jump path from the tracker. Every
ordinary movement greater than 45 degrees must pass the existing consecutive
candidate accumulation rules. Existing angle-dependent confirmation counts,
pending spread limits, and low-confidence rejection remain unchanged.

## Bird Entry Threshold

Raise only the raw bird entry threshold from `0.68` to `0.85`. Keep the
smoothed threshold at `0.58`, raw margin at `0.16`, two-window confirmation,
and all exit behavior unchanged. This is a temporary false-positive reduction;
it does not alter or retrain the v2 model.

## Verification

Host tests must prove that:

- source inactivity invalidates an old angle and two new valid frames acquire
  the new direction without displaying the old direction
- ten invalid localization frames expire a previously fresh direction
- a transient peak updates immediately and six following reflection frames
  cannot change the angle
- a single high-confidence ordinary opposite jump is rejected
- raw bird confidence below `0.85` cannot enter bird state, while two
  qualifying windows can
- all existing SRP-PHAT, mapping, web, LED, and decision tests still pass

The ESP-IDF 5.4.2 firmware must build with Ninja concurrency limited to two.
