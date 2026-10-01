# DOA Session Reacquisition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent stale directions and transient reflections from driving LEDs while reducing non-bird false positives with a raw `0.85` entry threshold.

**Architecture:** Keep SRP-PHAT and the existing vector tracker intact, but place a deterministic session controller between localization candidates and the LED. The controller owns freshness, source-session reset, ten-frame expiry, and six-frame transient locking; the audio task only supplies frame observations and consumes the controller's held angle and validity.

**Tech Stack:** ESP-IDF C, existing SRP-PHAT and DOA tracker, pytest, MSVC host-side C tests, ESP-IDF 5.4.2/Ninja.

## Global Constraints

- Ten 1024-sample frames are the approximately 0.6-second stale timeout.
- Six frames are the approximately 350 ms transient reflection lock.
- A source-off event invalidates the previous direction immediately.
- A new source cannot display an old direction before new-frame confirmation.
- A transient first peak may reseed immediately; locked reflections cannot update the tracker.
- Ordinary jumps greater than 45 degrees never update from one frame.
- Set only raw bird entry threshold to `0.85`; keep `0.58`, `0.16`, two-window confirmation, and exit rules.
- Do not modify SRP-PHAT, microphone layout, direction calibration, LED mapping, or source thresholds.

---

### Task 1: Bird Gate and Ordinary Jump Confirmation

**Files:**
- Modify: `tests/test_bird_decision_main.c`
- Modify: `tests/test_doa_tracker_main.c`
- Modify: `main/bird_decision.c`
- Modify: `main/doa_tracker.c`
- Modify: `main/doa_tracker.h`

**Interfaces:**
- Preserve: `bird_decision_update(...)` and `doa_tracker_update(...)`
- Add: `bool doa_tracker_reseed(doa_tracker_t *, float angle_deg, float confidence, float *held_angle)`

- [ ] Write boundary tests showing `0.84` cannot enter bird, `0.85` still needs two qualifying windows, and one `0.36` large-jump frame cannot move an initialized tracker.
- [ ] Run `python -m pytest -q tests/test_bird_decision.py tests/test_doa_tracker.py` and verify failure against current behavior.
- [ ] Set `BIRD_THRESHOLD` to `0.85f`, remove the `HIGH_CONFIDENCE_JUMP` one-frame branch, and route every jump over 45 degrees through `accumulate_pending`.
- [ ] Add `doa_tracker_reseed` that validates inputs/confidence, replaces the vector and held angle, clears pending state, and marks the tracker initialized.
- [ ] Re-run both focused tests and verify they pass.
- [ ] Commit with `git commit -m "require confirmed doa jumps"`.

### Task 2: Direction Session Controller

**Files:**
- Create: `main/doa_session.h`
- Create: `main/doa_session.c`
- Create: `tests/test_doa_session_main.c`
- Create: `tests/test_doa_session.py`

**Interfaces:**
- Produce: `void doa_session_init(doa_session_t *, float min_confidence)`
- Produce: `bool doa_session_update(doa_session_t *, float angle_deg, float confidence, bool source_active, bool candidate_valid, bool transient_candidate, float *held_angle)`
- Produce: `bool doa_session_direction_valid(const doa_session_t *)`

- [ ] Write a host test proving source-off reset and two-frame reacquisition at the opposite direction, ten invalid-frame expiry, immediate transient reseed, and six locked reflection frames.
- [ ] Run `python -m pytest -q tests/test_doa_session.py` and verify compile failure because the controller does not yet exist.
- [ ] Implement `doa_session_t` with a nested `doa_tracker_t`, `invalid_frames`, `transient_lock_frames`, and `fresh`.
- [ ] On source-off, reset the nested tracker and freshness. On ten invalid frames, do the same. On a valid candidate, clear the miss count. On a transient while unlocked, reseed and set a six-frame lock. While locked, hold the current angle without updating.
- [ ] Re-run the focused session test and verify it passes.
- [ ] Commit with `git commit -m "expire stale doa sessions"`.

### Task 3: Audio Task Integration

**Files:**
- Modify: `main/CMakeLists.txt`
- Modify: `main/i2s_es7210_record_example.c`

**Interfaces:**
- Consume: the `doa_session_*` API from Task 2
- Preserve: existing LED, Wi-Fi, EI inference, and localization log interfaces

- [ ] Add `doa_session.c` to the component and replace the global tracker with `doa_session_t`.
- [ ] Pass `source_active`, `localization_valid`, and `transient_doa` to `doa_session_update`; remove the direct transient `0.36` tracker shortcut from the audio task.
- [ ] Gate `led_update_direction` with `doa_session_direction_valid`, so an old held angle cannot relight after source-off or expiry.
- [ ] Add `fresh`, `miss`, and `lock` fields to the periodic DOA log for board verification, then initialize the session with minimum confidence `0.08`.
- [ ] Run `python -m pytest -q tests` and verify all host tests pass.
- [ ] Commit with `git commit -m "gate leds on fresh doa sessions"`.

### Task 4: Firmware Verification

**Files:**
- Verify only: all files changed in Tasks 1-3

**Interfaces:**
- Consume: completed implementation
- Produce: host and firmware build evidence

- [ ] Run `python -m pytest -q tests` sequentially and verify zero failures.
- [ ] Run `git diff --check 0c08acb..HEAD` and verify no whitespace errors.
- [ ] Load `D:\\espidf\\v5.4.2\\esp-idf\\export.ps1`, then run `ninja -C build -j 2 all` and verify `NINJA_EXIT=0`.
- [ ] Run `git status --short` and review `git diff --stat 0c08acb..HEAD` for approved scope only.
