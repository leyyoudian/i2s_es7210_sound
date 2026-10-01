#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Host-side simulation of the DOA display pipeline (tracker -> gate -> LED).

Mirrors main/doa/doa_tracker.c, main/doa/doa_display_gate.c,
main/led/led_indicator.c and main/doa/direction_mapping.c so the three
reported symptoms can be reproduced and fixes validated without a compiler.
"""
import math

M_PI = math.pi


def circ_dist(a, b):
    d = abs(a - b)
    return 360.0 - d if d > 180.0 else d


def wrap_deg(d):
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def norm360(a):
    a %= 360.0
    return a + 360.0 if a < 0 else a


# ---------------------------------------------------------------- direction mapping
LED_ANGLES = [0.0, 337.5, 315.0, 292.5, 270.0, 247.5, 225.0, 202.5,
              180.0, 157.5, 135.0, 112.5, 90.0, 67.5, 45.0, 22.5]


def led_nearest(angle):
    angle = norm360(angle)
    best, best_d = 0, 360.0
    for i, la in enumerate(LED_ANGLES):
        d = abs(angle - la)
        if d > 180.0:
            d = 360.0 - d
        if d < best_d:
            best_d, best = d, i
    return best


def led_angle_of_index(index):
    if index < 0 or index >= len(LED_ANGLES):
        return 0.0
    return LED_ANGLES[index]


def calibrate_doa(raw):
    return norm360(180.0 - raw)


# ---------------------------------------------------------------- tracker
REQUIRED_STABLE_FRAMES = 2
LARGE_JUMP_DEG = 60.0
PENDING_SPREAD_DEG = 20.0
HIGH_CONFIDENCE_JUMP = 0.35
HIGH_CONFIDENCE_JUMP_FRAMES = 2
TRANSIENT_GUARD_FRAMES = 6
TRANSIENT_OPPOSITE_JUMP_DEG = 120.0
TRANSIENT_OPPOSITE_MIN_CONF = 0.60
TRANSIENT_INITIAL_LOCK_MIN_CONF = 0.35
MAX_PENDING_MISSES = 3
OPPOSITE_JUMP_DEG = 120.0
OPPOSITE_STABLE_FRAMES = 5
OPPOSITE_MIN_CONFIDENCE = 0.18
MEDIUM_JUMP_DEG = 60.0
WIDE_JUMP_DEG = 90.0
MEDIUM_CONFIDENCE = 0.25
WIDE_CONFIDENCE = 0.35


class Tracker:
    def __init__(self, min_confidence):
        self.min_confidence = min_confidence
        self.x = self.y = 0.0
        self.held_angle = 0.0
        self.angular_rate = 0.0
        self.pending_x = self.pending_y = 0.0
        self.pending_count = 0
        self.pending_misses = 0
        self.transient_guard_frames = 0
        self.pending_high_confidence_jump = False
        self.pending_opposite_jump = False
        self.initialized = False

    def _alpha_beta(self, angle_deg, alpha):
        measured = angle_deg % 360.0
        pred_deg = self.held_angle + self.angular_rate * 180.0 / M_PI
        residual_deg = wrap_deg(measured - pred_deg)
        beta = alpha * alpha / (2.0 - alpha)
        self.angular_rate += beta * residual_deg * (M_PI / 180.0)
        smoothed = pred_deg + alpha * residual_deg
        smoothed %= 360.0
        rad = math.radians(smoothed)
        self.x, self.y = math.cos(rad), math.sin(rad)
        self.held_angle = smoothed

    def _accumulate(self, angle_deg, opposite_jump, high_conf_jump, required):
        rad = math.radians(angle_deg)
        x, y = math.cos(rad), math.sin(rad)
        if self.pending_count > 0:
            pending_angle = math.degrees(math.atan2(self.pending_y, self.pending_x)) % 360.0
            if (self.pending_opposite_jump != opposite_jump or
                    self.pending_high_confidence_jump != high_conf_jump or
                    circ_dist(angle_deg, pending_angle) > PENDING_SPREAD_DEG):
                self.pending_count = 0
        if self.pending_count == 0:
            self.pending_x, self.pending_y = x, y
            self.pending_count = 1
            self.pending_opposite_jump = opposite_jump
            self.pending_high_confidence_jump = high_conf_jump
        else:
            self.pending_x += x
            self.pending_y += y
            self.pending_count += 1
        self.pending_misses = 0
        return self.pending_count >= required

    def update_transient(self, angle_deg, confidence, candidate_valid):
        if self.transient_guard_frames > 0:
            self.transient_guard_frames -= 1
            return False, self.held_angle
        if (not candidate_valid or not math.isfinite(angle_deg) or
                not math.isfinite(confidence) or confidence < self.min_confidence):
            return False, self.held_angle
        normalized = angle_deg % 360.0
        if not self.initialized and confidence < TRANSIENT_INITIAL_LOCK_MIN_CONF:
            return False, self.held_angle
        if self.initialized:
            movement = circ_dist(normalized, self.held_angle)
            if (movement > TRANSIENT_OPPOSITE_JUMP_DEG and
                    confidence < TRANSIENT_OPPOSITE_MIN_CONF):
                return False, self.held_angle
        rad = math.radians(normalized)
        self.x, self.y = math.cos(rad), math.sin(rad)
        self.held_angle = normalized
        self.angular_rate = 0.0
        self.pending_count = 0
        self.pending_misses = 0
        self.transient_guard_frames = TRANSIENT_GUARD_FRAMES
        self.pending_high_confidence_jump = False
        self.pending_opposite_jump = False
        self.initialized = True
        return True, self.held_angle

    def required_jump_frames(self, movement, confidence):
        if movement > OPPOSITE_JUMP_DEG:
            return OPPOSITE_STABLE_FRAMES if confidence >= OPPOSITE_MIN_CONFIDENCE else 0
        if movement > WIDE_JUMP_DEG:
            return 3 if confidence >= WIDE_CONFIDENCE else 5
        if movement > MEDIUM_JUMP_DEG:
            return 3 if confidence >= MEDIUM_CONFIDENCE else 4
        return REQUIRED_STABLE_FRAMES if confidence >= MEDIUM_CONFIDENCE else 3

    def update(self, angle_deg, confidence, candidate_valid):
        transient_guard_active = self.transient_guard_frames > 0
        if transient_guard_active:
            self.transient_guard_frames -= 1
        if (not candidate_valid or not math.isfinite(angle_deg) or
                not math.isfinite(confidence) or confidence < self.min_confidence):
            if self.pending_count > 0:
                if self.pending_high_confidence_jump:
                    self.pending_count = 0
                    self.pending_misses = 0
                    self.pending_high_confidence_jump = False
                    self.pending_opposite_jump = False
                else:
                    self.pending_misses += 1
                    if self.pending_misses > MAX_PENDING_MISSES:
                        self.pending_count = 0
                        self.pending_misses = 0
                        self.pending_high_confidence_jump = False
                        self.pending_opposite_jump = False
            return False, self.held_angle

        if not self.initialized:
            if not self._accumulate(angle_deg, False, False, REQUIRED_STABLE_FRAMES):
                return False, self.held_angle
            self.x, self.y = self.pending_x, self.pending_y
            self.angular_rate = 0.0
            self.pending_count = 0
            self.pending_high_confidence_jump = False
            self.initialized = True
        else:
            movement = circ_dist(angle_deg, self.held_angle)
            if transient_guard_active and movement > LARGE_JUMP_DEG:
                self.pending_count = 0
                self.pending_misses = 0
                self.pending_high_confidence_jump = False
                self.pending_opposite_jump = False
                return False, self.held_angle
            direct_jump = False
            if movement > LARGE_JUMP_DEG:
                opposite_jump = movement > OPPOSITE_JUMP_DEG
                if opposite_jump:
                    required = self.required_jump_frames(movement, confidence)
                elif confidence >= HIGH_CONFIDENCE_JUMP:
                    required = HIGH_CONFIDENCE_JUMP_FRAMES
                else:
                    required = self.required_jump_frames(movement, confidence)
                if required == 0:
                    return False, self.held_angle
                high_conf_jump = (not opposite_jump and
                                  confidence >= HIGH_CONFIDENCE_JUMP)
                if not self._accumulate(angle_deg, opposite_jump, high_conf_jump, required):
                    return False, self.held_angle
                cnt = float(self.pending_count)
                self.x = self.pending_x / cnt
                self.y = self.pending_y / cnt
                self.pending_count = 0
                self.pending_misses = 0
                self.pending_high_confidence_jump = False
                self.pending_opposite_jump = False
                self.angular_rate = 0.0
                direct_jump = True
            else:
                self.pending_count = 0
                self.pending_misses = 0
                self.pending_high_confidence_jump = False
                self.pending_opposite_jump = False
            if movement > LARGE_JUMP_DEG:
                self.angular_rate = 0.0
            if not direct_jump:
                if movement <= 15.0:
                    alpha = 0.15
                elif movement <= 30.0:
                    alpha = 0.28
                else:
                    alpha = 0.45 if confidence >= MEDIUM_CONFIDENCE else 0.35
                self._alpha_beta(angle_deg, alpha)

        held = math.degrees(math.atan2(self.y, self.x))
        if held < 0:
            held += 360.0
        self.held_angle = held
        return True, self.held_angle


# ---------------------------------------------------------------- display gate
STALE_WEAK_REJECT_FRAMES = 15
WEAK_REJECT_ENERGY = 0.0015
CANDIDATE_CONFIRM_FRAMES = 2
SWITCH_HYSTERESIS_DEG = 14.0
DISPLAY_DEAD_ZONE_DEG = 5.0


class DisplayGate:
    def __init__(self):
        self.stale_weak_reject_frames = 0
        self.display_active = False
        self.held_pixel = -1
        self.candidate_pixel = -1
        self.candidate_count = 0
        self.display_angle = 0.0

    @staticmethod
    def _gated_angle(live, current):
        return live if circ_dist(live, current) >= DISPLAY_DEAD_ZONE_DEG else current

    def update(self, tracker_updated, source_active, localization_valid,
               doa_energy, angle_deg):
        if tracker_updated and source_active and localization_valid and math.isfinite(angle_deg):
            force = (not self.display_active) or self.held_pixel < 0
            if (not force and self.held_pixel >= 0 and
                    circ_dist(angle_deg, led_angle_of_index(self.held_pixel)) <
                    SWITCH_HYSTERESIS_DEG):
                pixel = self.held_pixel
            else:
                pixel = led_nearest(angle_deg)
            self.stale_weak_reject_frames = 0
            if force or pixel == self.held_pixel:
                self.held_pixel = pixel
                self.candidate_pixel = -1
                self.candidate_count = 0
                self.display_angle = angle_deg if force else \
                    self._gated_angle(angle_deg, self.display_angle)
                self.display_active = True
                return True, self.display_angle
            if pixel != self.candidate_pixel:
                self.candidate_pixel = pixel
                self.candidate_count = 1
            elif self.candidate_count < CANDIDATE_CONFIRM_FRAMES:
                self.candidate_count += 1
            if self.candidate_count >= CANDIDATE_CONFIRM_FRAMES:
                self.held_pixel = pixel
                self.candidate_pixel = -1
                self.candidate_count = 0
                self.display_angle = angle_deg
            return self.display_active, self.display_angle

        if not source_active:
            self.stale_weak_reject_frames = 0
            self.candidate_pixel = -1
            self.candidate_count = 0
            self.display_active = False
            return False, self.display_angle

        weak_rejected = (not localization_valid) and (
            (not math.isfinite(doa_energy)) or doa_energy < WEAK_REJECT_ENERGY)
        if weak_rejected:
            self.stale_weak_reject_frames += 1
            if self.stale_weak_reject_frames >= STALE_WEAK_REJECT_FRAMES:
                self.display_active = False
                self.candidate_pixel = -1
                self.candidate_count = 0
        else:
            self.stale_weak_reject_frames = 0
        return self.display_active, self.display_angle


# ---------------------------------------------------------------- LED indicator
LED_DISPLAY_EMA_ALPHA = 0.30
LED_DISPLAY_SNAP_DEG = 45.0


class LedIndicator:
    def __init__(self):
        self.rendered_pixel = -1
        self.disp_x = 0.0
        self.disp_y = 0.0
        self.disp_valid = False
        self.lit_history = []          # pixels actually rendered, in order

    def update(self, angle_deg, source_active):
        if not source_active:
            if self.rendered_pixel >= 0:
                self.rendered_pixel = -1
                self.lit_history.append(None)   # all off
            return
        pixel = led_nearest(angle_deg)
        if pixel == self.rendered_pixel:
            return
        rad = math.radians(angle_deg)
        mx, my = math.cos(rad), math.sin(rad)
        snap = not self.disp_valid
        if not snap:
            cur_angle = math.degrees(math.atan2(self.disp_y, self.disp_x)) % 360.0
            snap = circ_dist(angle_deg, cur_angle) >= LED_DISPLAY_SNAP_DEG
        if snap:
            self.disp_x, self.disp_y = mx, my
            self.disp_valid = True
        else:
            a = LED_DISPLAY_EMA_ALPHA
            self.disp_x = (1.0 - a) * self.disp_x + a * mx
            self.disp_y = (1.0 - a) * self.disp_y + a * my
            inv = 1.0 / math.sqrt(self.disp_x ** 2 + self.disp_y ** 2 + 1e-9)
            self.disp_x *= inv
            self.disp_y *= inv
        smooth_angle = math.degrees(math.atan2(self.disp_y, self.disp_x)) % 360.0
        smooth_pixel = led_nearest(smooth_angle)
        if smooth_pixel == self.rendered_pixel:
            return
        self.rendered_pixel = smooth_pixel
        self.lit_history.append(smooth_pixel)

    def pixels(self):
        return self.lit_history


# ---------------------------------------------------------------- pipeline
class Pipeline:
    """tracker -> gate -> led, matching audio_task wiring."""

    def __init__(self):
        self.tracker = Tracker(0.025)
        self.gate = DisplayGate()
        self.led = LedIndicator()

    def frame(self, angle_raw, confidence, localization_valid, source_active,
              transient=False, doa_energy=0.010):
        if transient and localization_valid:
            tracker_updated, angle = self.tracker.update_transient(
                angle_raw, confidence, True)
        else:
            tracker_updated, angle = self.tracker.update(
                angle_raw, confidence, localization_valid)
        display_active, display_angle = self.gate.update(
            tracker_updated, source_active, localization_valid,
            doa_energy, angle)
        self.led.update(display_angle, display_active and self.tracker.initialized)
        return angle, display_angle


# ---------------------------------------------------------------- scenarios
def sim_stationary_jitter(angle_center=0.0, jitter=10.0, frames=200, seed=7):
    """A source sitting still; per-frame estimate noise ±jitter degrees."""
    import random
    rnd = random.Random(seed)
    p = Pipeline()
    for _ in range(frames):
        a = norm360(angle_center + rnd.uniform(-jitter, jitter))
        p.frame(a, 0.35, True, True)
    return p


def sim_source_switch(from_a=0.0, to_b=180.0, switch_frame=60, frames=120, seed=3):
    import random
    rnd = random.Random(seed)
    p = Pipeline()
    for i in range(frames):
        if i < switch_frame:
            a = from_a
            active = True
        elif i < switch_frame + 2:
            a = from_a      # source briefly absent
            active = True   # keep display alive (smoothing/decay)
        else:
            a = to_b
            active = True
        a = norm360(a + rnd.uniform(-4, 4))
        p.frame(a, 0.35, True, active)
    return p


def sim_snap_mirror(held_angle=0.0, mirror_angle=180.0, snap_conf=0.45):
    """Snap frame lands on the mirror peak while a source is lit at held_angle."""
    p = Pipeline()
    for _ in range(30):
        p.frame(held_angle, 0.35, True, True)
    pre = list(p.led.pixels())
    # snap frame via fast path, then a couple of quiet frames
    p.frame(mirror_angle, snap_conf, True, True, transient=True)
    for _ in range(10):
        p.frame(held_angle, 0.30, True, True)
    return p, pre


if __name__ == "__main__":
    print("=== scenario 1: stationary source, ±10° estimate jitter ===")
    p = sim_stationary_jitter()
    px = p.led.pixels()
    print("LED pixel switches:", len(px), "unique:", sorted(set(x for x in px if x is not None)))
    print("last 30 lit sequence:", [x for x in px[-30:]])

    print("\n=== scenario 2: source switch 0 -> 180 (marquee check) ===")
    p = sim_source_switch()
    px = [x for x in p.led.pixels()]
    print("lit sequence after switch:", px[-40:])

    print("\n=== scenario 3: snap lands on mirror peak (conf 0.45 fast path) ===")
    p, pre = sim_snap_mirror()
    print("lit before snap:", pre[-4:], " after snap:", p.led.pixels()[-8:])

    print("\n=== scenario 4: flip-flop between two sources 58/292 (from serial log) ===")
    p = Pipeline()
    seq = []
    for i in range(240):
        a = 58.0 if (i // 8) % 2 == 0 else 292.0
        p.frame(a, 0.40 if a == 58.0 else 0.28, True, True)
        seq.append(p.led.rendered_pixel)
    # count how many intermediate pixels get lit during transitions
    print("unique pixels lit:", sorted(set(seq)))
    transitions = 0
    for i in range(1, len(seq)):
        if seq[i] != seq[i - 1]:
            transitions += 1
    print("total LED changes over 240 frames:", transitions)
