from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ODriveSafetyConfig:
    """ODrive configuration for Novus Wheel.

    Includes safety limits (current/velocity) and encoder parameters.
    Defaults match scripts/example_spin.py and scripts/odrive_movement.py.
    """

    axis: int = 0
    # Motor pole pairs (only applied when override_motor_pole_pairs=True)
    pole_pairs: int = 7
    override_motor_pole_pairs: bool = True

    current_limit_a: float = 16.0
    calibration_current_a: float = 6.0
    dc_max_negative_current_a: float = -5.0
    vel_limit_turn_s: float = 30.0
    vel_ramp_rate_turn_s2: float = 2.0

    # Incremental encoder config
    # ODrive expects CPR as counts/rev after quadrature (4x PPR).
    encoder_cpr: int = 4000
    motor_pulley_teeth: int = 30
    encoder_pulley_teeth: int = 20
    override_encoder_cpr: bool = True
    override_encoder_mode: bool = True
    encoder_use_index: bool = False
    encoder_bandwidth: float = 1000.0


@dataclass(frozen=True)
class WheelKinematics:
    """How to map encoder turns into a steering axis."""

    # Lock-to-lock in *wheel* turns. Example: 2.0 turns = 720 degrees.
    lock_to_lock_turns: float = 2.0
    # Optional gear ratio: wheel turns per motor turn.
    # If encoder is on the motor but wheel has reduction, set >1.
    wheel_turns_per_motor_turn: float = 1.0


@dataclass(frozen=True)
class FfbTuning:
    """Tuning for converting game FFB to motor torque."""

    # Maximum torque command in Nm that we will send in torque-control mode.
    # This is a *software clamp* in addition to current_limit_a.
    max_torque_nm: float = 2.0

    # Damping added on top of game effects (Nm per (turn/s))
    extra_damping_nm_per_turn_s: float = 0.0

    # Low-pass filter for torque command (0 disables).
    torque_lpf_hz: float = 60.0


@dataclass(frozen=True)
class AppConfig:
    odrive: ODriveSafetyConfig = ODriveSafetyConfig()
    wheel: WheelKinematics = WheelKinematics()
    ffb: FfbTuning = FfbTuning()

    # Control loop rate.
    loop_hz: float = 500.0

    # Calibration storage.
    calibration_path: str = "novus_wheel_calibration.json"
