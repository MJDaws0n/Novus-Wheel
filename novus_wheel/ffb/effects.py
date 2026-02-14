"""Force Feedback effect model for Novus Wheel.

Tracks per-slot DirectInput / HID PID effects and computes the aggregate
force to send to the motor.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# HID PID Effect Type IDs
ET_CONSTANT = 0x01
ET_RAMP = 0x02
ET_SQUARE = 0x03
ET_SINE = 0x04
ET_TRIANGLE = 0x05
ET_SAW_UP = 0x06
ET_SAW_DOWN = 0x07
ET_SPRING = 0x08
ET_DAMPER = 0x09
ET_INERTIA = 0x0A
ET_FRICTION = 0x0B


def _sign(x: float) -> float:
    return -1.0 if x < 0.0 else (1.0 if x > 0.0 else 0.0)


@dataclass
class EffectSlot:
    """A single FFB effect instance (one DirectInput effect block)."""

    effect_type: int = 0
    playing: bool = False
    gain: float = 1.0  # Per-effect gain [0..1]

    # --- Constant Force ---
    magnitude: float = 0.0  # [-1..+1]
    # Some games encode constant-force sign in the effect's direction field rather than magnitude.
    # Default to +X direction.
    direction_sign: float = 1.0  # +1 or -1
    # If we ever observe a negative magnitude for this block, treat magnitude as signed and
    # do not additionally apply direction_sign (avoids double-inversion).
    const_magnitude_is_signed: bool = False

    # --- Condition (spring / damper / friction / inertia) ---
    center: float = 0.0  # [-1..+1]
    pos_coeff: float = 0.0  # [0..1]
    neg_coeff: float = 0.0  # [0..1]
    # Some titles send signed condition coefficients (negative for restoring/damping).
    # We keep both representations and choose a mode per effect block.
    pos_coeff_signed: float = 0.0  # [-1..+1]
    neg_coeff_signed: float = 0.0  # [-1..+1]
    cond_use_signed_coeff: bool = False
    pos_sat: float = 1.0  # [0..1]
    neg_sat: float = 1.0  # [0..1]
    dead_band: float = 0.0  # [0..1]


def _compute_condition_force(
    slot: EffectSlot,
    metric: float,
    *,
    center: float | None = None,
    use_signed_coeff: bool | None = None,
) -> float:
    """Evaluate a condition effect against a metric (position or velocity)."""
    c = slot.center if center is None else float(center)
    offset = metric - c
    if abs(offset) <= slot.dead_band:
        return 0.0

    signed_mode = slot.cond_use_signed_coeff if use_signed_coeff is None else bool(use_signed_coeff)

    if offset > 0.0:
        adj = offset - slot.dead_band
        if signed_mode:
            f = float(slot.pos_coeff_signed) * float(adj)
        else:
            # Default convention: coeff is positive magnitude and we apply a leading minus.
            f = -float(slot.pos_coeff) * float(adj)
        return clamp(f, -slot.pos_sat, slot.pos_sat)
    else:
        adj = offset + slot.dead_band
        if signed_mode:
            f = float(slot.neg_coeff_signed) * float(adj)
        else:
            f = -float(slot.neg_coeff) * float(adj)
        return clamp(f, -slot.neg_sat, slot.neg_sat)


def _slot_force(slot: EffectSlot, *, pos: float, vel: float, friction_ramp: float) -> float:
    """Compute the force contribution of a single effect slot."""
    t = slot.effect_type

    if t == ET_CONSTANT:
        m = float(slot.magnitude)
        if not slot.const_magnitude_is_signed:
            m *= float(slot.direction_sign)
        return m

    if t == ET_SPRING:
        return _compute_condition_force(slot, pos)

    if t == ET_DAMPER:
        # For velocity-based conditions, CenterPointOffset should generally be treated as 0;
        # some stacks/games send non-zero center which would create a constant force at rest.
        return _compute_condition_force(slot, vel, center=0.0)

    if t == ET_FRICTION:
        # Some titles send friction even when "stationary". Encoder velocity estimates can
        # jitter around 0, which would produce a constant torque and slowly spin the wheel.
        # Keep a deadzone and smoothly ramp friction in as speed increases.
        if abs(vel) <= slot.dead_band:
            return 0.0
        coeff = slot.pos_coeff if vel > 0 else slot.neg_coeff
        sat = slot.pos_sat if vel > 0 else slot.neg_sat
        # Soft-start friction over a small velocity range above deadband.
        # vel can be outside [-1,1] in this project (see app.py normalization), so this is
        # intentionally unitless and tuned via EffectState.
        vel_ramp = float(friction_ramp or 0.0)
        if vel_ramp > 0.0:
            ramp = clamp((abs(float(vel)) - float(slot.dead_band)) / vel_ramp, 0.0, 1.0)
        else:
            ramp = 1.0
        # Use a continuous sign near zero to avoid bang-bang direction flips from
        # tiny velocity jitter.
        eps = max(1e-6, float(getattr(slot, "dead_band", 0.0) or 0.0), 0.02)
        s = float(vel) / (abs(float(vel)) + float(eps))
        return clamp(-float(s) * coeff * float(ramp), -sat, sat)

    if t == ET_INERTIA:
        # Approximate inertia as light damping.
        return _compute_condition_force(slot, vel, center=0.0) * 0.5

    # Periodic effects (sine, square, etc.) and ramp are uncommon in
    # sim-racing titles; treat as zero for now.
    return 0.0


class EffectState:
    """Tracks all active FFB effects and computes the aggregate force."""

    def __init__(self) -> None:
        self.slots: dict[int, EffectSlot] = {}
        self.device_gain: float = 1.0
        self.last_update: float = 0.0
        # If no FFB updates arrive for this many seconds, zero all forces.
        self.stale_timeout_s: float = 2.0

        # Velocity conditioning: helps prevent damper/friction from generating constant
        # torque at rest due to encoder noise.
        self.vel_lpf_hz: float = float(os.environ.get("NOVUS_WHEEL_FFB_VEL_LPF_HZ", "25.0") or 25.0)
        self.vel_zero_epsilon: float = float(os.environ.get("NOVUS_WHEEL_FFB_VEL_ZERO_EPS", "0.02") or 0.02)
        # Soft-start range for ET_FRICTION above deadband (in the same units as vel input).
        self.friction_ramp: float = float(os.environ.get("NOVUS_WHEEL_FFB_FRICTION_RAMP", "0.15") or 0.15)

        self._vel_filt: float = 0.0
        self._vel_t: float | None = None

        # Debug/diagnostics: last raw packet summary received from the backend.
        self._debug_seq: int = 0
        self._debug_last: str = ""
        self._debug_time: float = 0.0

    def record_packet_debug(self, msg: str) -> None:
        """Record a human-readable summary of the last FFB packet received."""

        self._debug_seq += 1
        self._debug_last = str(msg)
        self._debug_time = time.monotonic()

    def get_packet_debug(self) -> tuple[int, str, float]:
        """Return (seq, message, time_monotonic)."""

        return int(self._debug_seq), str(self._debug_last), float(self._debug_time)

    def get_or_create(self, block_id: int) -> EffectSlot:
        if block_id not in self.slots:
            self.slots[block_id] = EffectSlot()
        return self.slots[block_id]

    def remove(self, block_id: int) -> None:
        self.slots.pop(block_id, None)

    def start(self, block_id: int, solo: bool = False) -> None:
        if solo:
            for s in self.slots.values():
                s.playing = False
        slot = self.slots.get(block_id)
        if slot is not None:
            slot.playing = True

    def stop(self, block_id: int) -> None:
        slot = self.slots.get(block_id)
        if slot is not None:
            slot.playing = False

    def stop_all(self) -> None:
        for s in self.slots.values():
            s.playing = False

    def reset(self) -> None:
        self.slots.clear()

    def touch(self) -> None:
        """Mark that we received an FFB packet (keeps staleness timer alive)."""
        self.last_update = time.monotonic()

    @property
    def is_stale(self) -> bool:
        if self.stale_timeout_s <= 0.0:
            return False
        if self.last_update == 0.0:
            return True
        return (time.monotonic() - self.last_update) > self.stale_timeout_s

    def compute_force(self, *, pos: float, vel: float, invert_directional: bool = False) -> float:
        """Sum all playing effects and return total force in [-1, +1].

        Parameters
        ----------
        pos : float
            Normalized wheel position in [-1, +1].
        vel : float
            Normalized wheel velocity.
        invert_directional : bool
            If *True*, flip the sign of **directional** effects only
            (constant force, ramp, periodic).  Condition effects (spring,
            damper, friction, inertia) are left untouched because they
            compute their own sign from position/velocity physics and are
            already correct.
        """
        if self.is_stale:
            return 0.0

        # Filter and de-noise velocity before feeding it to damping/friction effects.
        now = time.monotonic()
        vel_in = float(vel)
        if self.vel_lpf_hz <= 0.0:
            vel_use = vel_in
        else:
            if self._vel_t is None:
                self._vel_t = float(now)
                self._vel_filt = float(vel_in)
            else:
                dt = max(0.0, float(now) - float(self._vel_t))
                self._vel_t = float(now)
                # alpha = 1 - exp(-2*pi*f*dt)
                alpha = 1.0 - math.exp(-2.0 * math.pi * float(self.vel_lpf_hz) * dt) if dt > 0.0 else 1.0
                self._vel_filt = (1.0 - alpha) * float(self._vel_filt) + alpha * float(vel_in)
            vel_use = float(self._vel_filt)

        if abs(float(vel_use)) < float(self.vel_zero_epsilon):
            vel_use = 0.0

        total = 0.0
        # Snapshot to avoid RuntimeError if callback mutates dict concurrently.
        try:
            snapshot = list(self.slots.values())
        except RuntimeError:
            return 0.0

        _CONDITION_TYPES = {ET_SPRING, ET_DAMPER, ET_FRICTION, ET_INERTIA}
        dir_sign = -1.0 if invert_directional else 1.0

        for slot in snapshot:
            if not slot.playing:
                continue

            f = _slot_force(slot, pos=pos, vel=float(vel_use), friction_ramp=float(self.friction_ramp))
            # Flip condition effects (damper/spring/friction/inertia) only;
            # directional effects (constant force from game) are already correct.
            if slot.effect_type in _CONDITION_TYPES:
                f *= dir_sign
            total += f * slot.gain
        total *= self.device_gain
        return clamp(total, -1.0, 1.0)
