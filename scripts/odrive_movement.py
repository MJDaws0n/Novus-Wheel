#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    CONTROL_MODE_VELOCITY_CONTROL,
    ENCODER_MODE_INCREMENTAL,
    INPUT_MODE_VEL_RAMP,
    MOTOR_TYPE_PMSM_CURRENT_CONTROL,
)
from odrive.utils import dump_errors


@dataclass(frozen=True)
class SensorlessConfig:
    pole_pairs: int = 7
    current_limit: float = 15.0
    calibration_current: float = 6.0
    dc_max_negative_current: float = -5.0
    vel_limit: float = 50.0
    vel_ramp_rate: float = 2.0
    min_sensorless_vel_turn_s: float = 0.2


@dataclass(frozen=True)
class Axis0Config:
    # Motor pole pairs.
    pole_pairs: int = 7
    # Default back to the old behavior: apply this value to the device.
    override_motor_pole_pairs: bool = True
    current_limit: float = 15.0
    calibration_current: float = 6.0
    dc_max_negative_current: float = -5.0
    vel_limit: float = 50.0
    vel_ramp_rate: float = 2.0
    prefer_encoder: bool = True
    fallback_to_sensorless: bool = True
    min_sensorless_vel_turn_s: float = 0.2

    # Incremental encoder config (A/B, optional Z index)
    # ODrive expects `cpr` as *counts per revolution* after quadrature (i.e. 4xPPR).
    # For a "1000 P/R" incremental encoder (1000 pulses per rev), use 4000 CPR.
    # If you drive the encoder with a belt/pulley reduction, ODrive needs the *effective*
    # counts per motor revolution. Set pulley teeth to compute it automatically.
    encoder_cpr: int = 4000
    motor_pulley_teeth: int = 30
    encoder_pulley_teeth: int = 20

    # Default back to the old behavior: apply these values to the device.
    override_encoder_cpr: bool = True
    override_encoder_mode: bool = True
    encoder_use_index: bool = False
    encoder_bandwidth: float = 1000.0
    # On FW 0.5.x this exists and is in electrical radians of scan distance.
    # Leave default unless you know you need to change it.
    encoder_calib_scan_distance: float | None = None


class ODriveAxis0:
    """Axis0 runner that prefers encoder closed-loop, with sensorless fallback.

    This is meant for ODrive FW 0.5.x + odrive 0.6.x.
    - If the encoder is working/configured, it runs FULL_CALIBRATION_SEQUENCE and
      uses normal encoder-based CLOSED_LOOP_CONTROL.
    - If encoder calibration fails, it can fall back to sensorless (motor-calibrate
      only) so you can still spin the motor.

    Use `dump_encoder_diagnostics()` if you believe the encoder is wired but the
    ODrive reports NO_RESPONSE / not-ready.
    """

    def __init__(
        self,
        *,
        timeout: float = 10.0,
        config: Axis0Config = Axis0Config(),
        verbose: bool = True,
    ):
        self._verbose = verbose
        self.config = config

        self.odr = odrive.find_any(timeout=timeout)
        self.axis = self.odr.axis0
        self._mode: str | None = None

        # Snapshot device config before we potentially override anything.
        self._device_pole_pairs: int | None = None
        self._device_encoder_cpr: int | None = None
        self._read_device_config_snapshot()

        self._clear_errors()
        self._configure_system()
        self._configure_motor_controller_common()

        if self.config.prefer_encoder:
            try:
                self._configure_for_encoder()
                self.calibrate_encoder()
                self._mode = "encoder"
                return
            except Exception as exc:
                self._log(f"Encoder path failed: {exc}")
                self.dump_encoder_diagnostics()
                if not self.config.fallback_to_sensorless:
                    raise
                self._log("Falling back to sensorless mode...")

        self._configure_for_sensorless()
        self.calibrate_motor_only()
        self._mode = "sensorless"

    @property
    def mode(self) -> str:
        return self._mode or "unknown"

    def _log(self, msg: str) -> None:
        if self._verbose:
            print(msg)

    def _clear_errors(self) -> None:
        try:
            self.odr.clear_errors()
            time.sleep(0.2)
        except Exception:
            pass

    def _read_device_config_snapshot(self) -> None:
        try:
            self._device_pole_pairs = int(getattr(self.axis.motor.config, "pole_pairs"))
        except Exception:
            self._device_pole_pairs = None
        try:
            self._device_encoder_cpr = int(getattr(self.axis.encoder.config, "cpr"))
        except Exception:
            self._device_encoder_cpr = None

    def _effective_pole_pairs(self) -> int:
        # Prefer live device config unless the user explicitly overrides.
        if bool(self.config.override_motor_pole_pairs):
            return int(self.config.pole_pairs)
        if self._device_pole_pairs not in (None, 0):
            return int(self._device_pole_pairs)
        return int(self.config.pole_pairs)

    def _effective_encoder_cpr(self) -> int:
        # Compute effective CPR for belt-driven encoder (motor_teeth/encoder_teeth).
        base = float(self.config.encoder_cpr)
        try:
            mt = int(getattr(self.config, "motor_pulley_teeth"))
            et = int(getattr(self.config, "encoder_pulley_teeth"))
            if mt > 0 and et > 0:
                base = base * (float(mt) / float(et))
        except Exception:
            pass

        eff = int(round(base))

        # Prefer live device config only if the user has disabled override.
        if not bool(self.config.override_encoder_cpr) and self._device_encoder_cpr not in (None, 0):
            return int(self._device_encoder_cpr)
        return eff

    def _configure_system(self) -> None:
        # Prevent instant DC_BUS_OVER_REGEN_CURRENT after config resets.
        try:
            self.odr.config.dc_max_negative_current = float(self.config.dc_max_negative_current)
        except Exception:
            pass

    def _configure_motor_controller_common(self) -> None:
        m = self.axis.motor.config
        m.motor_type = MOTOR_TYPE_PMSM_CURRENT_CONTROL
        # Do not overwrite pole_pairs unless explicitly requested.
        if bool(self.config.override_motor_pole_pairs):
            m.pole_pairs = int(self.config.pole_pairs)
        m.current_lim = float(self.config.current_limit)
        m.calibration_current = float(self.config.calibration_current)

        c = self.axis.controller.config
        c.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.input_mode = INPUT_MODE_VEL_RAMP
        c.vel_limit = float(self.config.vel_limit)
        c.vel_ramp_rate = float(self.config.vel_ramp_rate)

    def _configure_for_encoder(self) -> None:
        self.axis.config.enable_sensorless_mode = False
        # Force a real calibration pass if we are trying to use the encoder.
        try:
            self.axis.motor.config.pre_calibrated = False
        except Exception:
            pass
        try:
            self.axis.encoder.config.pre_calibrated = False
        except Exception:
            pass

        # Explicitly configure incremental encoder parameters ONLY if requested.
        # If your encoder is specified as PPR (lines), set encoder_cpr = 4 * PPR.
        if bool(self.config.override_encoder_mode):
            try:
                self.axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL
            except Exception:
                pass
        if bool(self.config.override_encoder_cpr):
            try:
                self.axis.encoder.config.cpr = int(self._effective_encoder_cpr())
            except Exception:
                pass
        try:
            self.axis.encoder.config.use_index = bool(self.config.encoder_use_index)
        except Exception:
            pass
        try:
            self.axis.encoder.config.bandwidth = float(self.config.encoder_bandwidth)
        except Exception:
            pass
        if self.config.encoder_calib_scan_distance is not None:
            try:
                self.axis.encoder.config.calib_scan_distance = float(self.config.encoder_calib_scan_distance)
            except Exception:
                pass

        self._print_preflight_config()

        # Quick sanity check: if CPR is 0, FULL_CALIBRATION_SEQUENCE will never work.
        try:
            cpr = getattr(self.axis.encoder.config, "cpr", None)
            mode = getattr(self.axis.encoder.config, "mode", None)
            if cpr in (None, 0):
                self._log(f"Encoder config warning: encoder.config.cpr={cpr} (mode={mode}).")
        except Exception:
            pass

    def _configure_for_sensorless(self) -> None:
        self.axis.config.enable_sensorless_mode = True
        try:
            self.axis.motor.config.pre_calibrated = False
        except Exception:
            pass

    def _print_preflight_config(self) -> None:
        """Print the live config values that matter for encoder calibration.

        This helps catch the common situation where a script accidentally overwrites
        saved ODrive config with incorrect defaults (leading to CPR_POLEPAIRS_MISMATCH).
        """
        try:
            live_pp = getattr(self.axis.motor.config, "pole_pairs")
        except Exception:
            live_pp = None
        try:
            live_cpr = getattr(self.axis.encoder.config, "cpr")
        except Exception:
            live_cpr = None
        try:
            live_mode = getattr(self.axis.encoder.config, "mode")
        except Exception:
            live_mode = None

        self._log(
            "Preflight config: "
            f"motor.pole_pairs={live_pp} (override={int(bool(self.config.override_motor_pole_pairs))}) "
            f"encoder.cpr={live_cpr} (override={int(bool(self.config.override_encoder_cpr))}) "
            f"encoder.mode={live_mode} (override={int(bool(self.config.override_encoder_mode))})"
        )

    def dump_encoder_diagnostics(self, *, watch_seconds: float = 0.0) -> None:
        """Print encoder status/config to help debug 'it is plugged in' issues."""
        enc = self.axis.encoder
        self._log("--- Encoder diagnostics (axis0/enc0) ---")
        self._log(f"axis.current_state={self.axis.current_state} axis.error={self.axis.error}")
        self._log(f"motor.error={self.axis.motor.error} encoder.error={enc.error}")

        # Config fields vary slightly across firmware; use getattr defensively.
        cfg = enc.config
        for name in [
            "mode",
            "cpr",
            "use_index",
            "pre_calibrated",
            "bandwidth",
            "idx_search_speed",
            "idx_offset",
            # Present for SPI absolute encoders on many firmwares.
            "abs_spi_cs_gpio_pin",
            "abs_spi_mode",
            "abs_spi_rate",
        ]:
            try:
                self._log(f"encoder.config.{name}={getattr(cfg, name)}")
            except Exception:
                pass

        for name in [
            "is_ready",
            "pos_estimate",
            "vel_estimate",
            "shadow_count",
            "count_in_cpr",
        ]:
            try:
                self._log(f"encoder.{name}={getattr(enc, name)}")
            except Exception:
                pass

        if watch_seconds and watch_seconds > 0:
            try:
                a = int(getattr(enc, "shadow_count"))
                time.sleep(float(watch_seconds))
                b = int(getattr(enc, "shadow_count"))
                self._log(f"shadow_count delta over {watch_seconds}s: {b - a}")
            except Exception:
                pass

        dump_errors(self.odr)

    def watch_encoder_counts(self, *, seconds: float = 5.0, interval: float = 0.2) -> None:
        """Print encoder count deltas over time.

        Run this with the motor in IDLE and rotate the shaft by hand.
        If the encoder is wired/configured correctly, counts should change.
        """
        enc = self.axis.encoder
        t_end = time.time() + max(0.0, float(seconds))
        try:
            last = int(getattr(enc, "shadow_count"))
        except Exception:
            self._log("Encoder does not expose shadow_count on this firmware")
            return

        self._log(f"Watching encoder.shadow_count for {seconds}s...")
        while time.time() < t_end:
            try:
                now = int(getattr(enc, "shadow_count"))
                delta = now - last
                self._log(f"shadow_count={now} (delta {delta})")
                last = now
            except Exception:
                break
            time.sleep(max(0.02, float(interval)))

    def measure_encoder_cpr_interactive(self) -> int:
        """Estimate incremental encoder CPR by manual rotation.

        Steps:
        1) Ensure axis is in IDLE.
        2) Rotate the motor/encoder shaft exactly 1 mechanical revolution by hand.
        3) Press Enter.

        Returns the absolute delta in `shadow_count`, which is the CPR value ODrive expects.
        """
        self.stop()
        enc = self.axis.encoder
        try:
            start = int(getattr(enc, "shadow_count"))
        except Exception:
            raise RuntimeError("Encoder does not expose shadow_count on this firmware")

        print("Rotate the shaft exactly 1 mechanical revolution, then press Enter...")
        input()

        try:
            end = int(getattr(enc, "shadow_count"))
        except Exception:
            raise RuntimeError("Encoder does not expose shadow_count on this firmware")

        cpr = abs(end - start)
        print(f"Measured CPR (counts/rev) ≈ {cpr}")
        return cpr

    def calibrate_encoder(self, *, timeout_s: float = 60.0) -> None:
        self._log("Full calibration (motor + encoder)...")
        self._clear_errors()

        self.axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        entered = False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            st = self.axis.current_state
            if st in (
                AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
                AXIS_STATE_MOTOR_CALIBRATION,
                AXIS_STATE_ENCODER_INDEX_SEARCH,
                AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
            ):
                entered = True

            if st == AXIS_STATE_IDLE and entered:
                # Ensure no lingering faults and encoder is ready.
                if self.axis.error != 0 or self.axis.motor.error != 0 or self.axis.encoder.error != 0:
                    dump_errors(self.odr)
                    raise RuntimeError(
                        f"Calibration ended with faults (axis_err={self.axis.error} motor_err={self.axis.motor.error} enc_err={self.axis.encoder.error})"
                    )
                try:
                    if not bool(getattr(self.axis.encoder, "is_ready")):
                        self.dump_encoder_diagnostics()
                        raise RuntimeError("Encoder not ready after calibration")
                except Exception:
                    # If is_ready isn't available, fall back to error==0.
                    pass
                self._clear_errors()
                return

            if st == AXIS_STATE_IDLE and not entered and (time.time() - t0) > 2.0:
                self.dump_encoder_diagnostics()
                raise RuntimeError("FULL_CALIBRATION_SEQUENCE did not start (axis stayed IDLE)")

            time.sleep(0.1)

        self.dump_encoder_diagnostics()
        raise TimeoutError("Full calibration timed out")

    def calibrate_motor_only(self, *, timeout_s: float = 45.0) -> None:
        self._log("Motor calibration (sensorless fallback)...")
        self._clear_errors()

        try:
            self.axis.motor.config.pre_calibrated = False
        except Exception:
            pass
        self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        entered = False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            st = self.axis.current_state
            if st == AXIS_STATE_MOTOR_CALIBRATION:
                entered = True
            if st == AXIS_STATE_IDLE and entered:
                self._clear_errors()
                return
            if st == AXIS_STATE_IDLE and not entered and (time.time() - t0) > 2.0:
                dump_errors(self.odr)
                raise RuntimeError("MOTOR_CALIBRATION did not start (axis stayed IDLE)")
            time.sleep(0.1)

        dump_errors(self.odr)
        raise TimeoutError("Motor calibration timed out")

    def estimate_pole_pairs_from_offset_calib(self, *, timeout_s: float = 30.0) -> float | None:
        """Try to estimate motor pole pairs from the encoder offset calibration scan.

        This is specifically to debug EncoderError.CPR_POLEPAIRS_MISMATCH.

        Theory (ODrive FW 0.5.x): during encoder offset calibration the motor is rotated
        by `encoder.config.calib_scan_distance` electrical radians.

        Electrical radians -> mechanical revolutions: mech_revs = scan / (2*pi*pole_pairs)
        Encoder counts observed: counts = cpr * mech_revs

        Rearranged (using observed counts and configured cpr):
            pole_pairs_est ≈ cpr * scan / (2*pi*counts)

        Returns estimated pole pairs (float) or None if insufficient info.
        """
        enc = self.axis.encoder
        try:
            scan = float(getattr(enc.config, "calib_scan_distance"))
        except Exception:
            self._log("Cannot read encoder.config.calib_scan_distance; cannot estimate pole_pairs")
            return None

        try:
            start = int(getattr(enc, "shadow_count"))
        except Exception:
            self._log("Encoder does not expose shadow_count; cannot estimate pole_pairs")
            return None

        self._log(f"Estimating pole pairs using offset calibration scan (scan={scan} electrical rad)...")
        self._clear_errors()

        # Offset calibration requires motor calibration to be valid.
        try:
            self.axis.motor.config.pre_calibrated = True
        except Exception:
            pass

        self.axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        entered = False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            st = self.axis.current_state
            if st == AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
                entered = True
            if st == AXIS_STATE_IDLE and entered:
                break
            if st == AXIS_STATE_IDLE and not entered and (time.time() - t0) > 2.0:
                break
            time.sleep(0.02)

        try:
            end = int(getattr(enc, "shadow_count"))
        except Exception:
            return None

        delta = abs(end - start)
        if delta <= 0:
            self._log("No encoder count change observed during scan; check motor actually moved and encoder wiring.")
            return None

        cpr = float(self._effective_encoder_cpr())
        pole_pairs_est = (cpr * scan) / (2.0 * math.pi * float(delta))

        self._log(f"Observed encoder delta={delta} counts over scan")
        self._log(f"Estimated pole_pairs ≈ {pole_pairs_est:.3f} (using cpr={int(cpr)} and scan={scan})")
        return pole_pairs_est

    def _enter_closed_loop(self, *, settle_s: float = 0.8) -> None:
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(max(0.0, float(settle_s)))
        if self.axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL or self.axis.error != 0:
            dump_errors(self.odr)
            raise RuntimeError(
                f"Failed to enter CLOSED_LOOP_CONTROL (state={self.axis.current_state} err={self.axis.error})"
            )

    def stop(self) -> None:
        try:
            self.axis.controller.input_vel = 0
        except Exception:
            pass
        try:
            self.axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

    def coast(self, seconds: float = 1.0) -> None:
        self.stop()
        time.sleep(max(0.0, float(seconds)))

    def apply_velocity(self, vel_turn_s: float, duration_s: float, *, coast_before: float | None = None) -> None:
        if coast_before is not None and coast_before > 0:
            self.coast(coast_before)

        vel = float(vel_turn_s)
        if self.mode == "sensorless":
            # In sensorless, very low speeds are unreliable.
            if abs(vel) < float(self.config.min_sensorless_vel_turn_s):
                vel = math.copysign(float(self.config.min_sensorless_vel_turn_s), vel if vel != 0 else 1.0)
            ramp_vel = vel * (2.0 * math.pi * float(self._effective_pole_pairs()))
            try:
                self.axis.config.sensorless_ramp.vel = float(ramp_vel)
            except Exception:
                pass

        self._enter_closed_loop()

        self.axis.controller.input_vel = vel
        t0 = time.time()
        last_print = 0.0
        while time.time() - t0 < float(duration_s):
            if self.axis.error != 0 or self.axis.motor.error != 0 or self.axis.encoder.error != 0:
                self.dump_encoder_diagnostics()
                raise RuntimeError(
                    f"Fault during run (axis_err={self.axis.error} motor_err={self.axis.motor.error} enc_err={self.axis.encoder.error})"
                )

            # Telemetry: continuously print motor velocity (turns/s).
            # Uses encoder.vel_estimate when available; otherwise derives from sensorless_estimator.
            now = time.time()
            if now - last_print >= 0.05:  # 20 Hz
                last_print = now
                enc_vel = None
                try:
                    enc_vel = float(getattr(self.axis.encoder, "vel_estimate"))
                except Exception:
                    enc_vel = None

                motor_vel = enc_vel
                if motor_vel is None or math.isnan(motor_vel):
                    try:
                        # sensorless_estimator.vel_estimate is typically electrical rad/s on FW 0.5.x
                        sle = float(getattr(self.axis.sensorless_estimator, "vel_estimate"))
                        motor_vel = sle / (2.0 * math.pi * float(self._effective_pole_pairs()))
                    except Exception:
                        motor_vel = float("nan")

                try:
                    vel_cmd = float(getattr(self.axis.controller, "input_vel"))
                except Exception:
                    vel_cmd = float(vel)

                vel_cmd_rpm = float(vel_cmd) * 60.0
                motor_vel_rpm = float(motor_vel) * 60.0
                line = (
                    f"\rvel_cmd={vel_cmd:+.3f} turn/s ({vel_cmd_rpm:+.1f} rpm)  "
                    f"motor_vel={float(motor_vel):+.3f} turn/s ({motor_vel_rpm:+.1f} rpm)"
                )
                print(line, end="", flush=True)

            time.sleep(0.02)
        self.axis.controller.input_vel = 0.0
        print("")

    def close(self) -> None:
        self.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


class ODriveAxis0Sensorless:
    """Minimal axis0 sensorless runner for ODrive FW 0.5.x.

    Constructor connects + configures + motor-calibrates.

    Notes:
    - This is sensorless closed-loop. Without an encoder, very low speeds are unreliable.
      We clamp to `min_sensorless_vel_turn_s` for stability.
    - `apply_velocity()` uses INPUT_MODE_VEL_RAMP so reversals are smooth.
    """

    def __init__(
        self,
        *,
        timeout: float = 10.0,
        config: SensorlessConfig = SensorlessConfig(),
        verbose: bool = True,
    ):
        self._verbose = verbose
        self.config = config

        self.odr = odrive.find_any(timeout=timeout)
        self.axis = self.odr.axis0

        self._clear_errors()
        self._configure_system()
        self._configure_axis_motor_controller()
        self.calibrate()

    def _log(self, msg: str) -> None:
        if self._verbose:
            print(msg)

    def _clear_errors(self) -> None:
        try:
            self.odr.clear_errors()
            time.sleep(0.2)
        except Exception:
            pass

    def _configure_system(self) -> None:
        # Prevent instant DC_BUS_OVER_REGEN_CURRENT after config resets.
        try:
            self.odr.config.dc_max_negative_current = float(self.config.dc_max_negative_current)
        except Exception:
            pass

    def _configure_axis_motor_controller(self) -> None:
        m = self.axis.motor.config
        m.motor_type = MOTOR_TYPE_PMSM_CURRENT_CONTROL
        m.pole_pairs = int(self.config.pole_pairs)
        m.current_lim = float(self.config.current_limit)
        m.calibration_current = float(self.config.calibration_current)
        m.pre_calibrated = False

        c = self.axis.controller.config
        c.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        c.input_mode = INPUT_MODE_VEL_RAMP
        c.vel_limit = float(self.config.vel_limit)
        c.vel_ramp_rate = float(self.config.vel_ramp_rate)

        # Sensorless enabled; direction comes from sensorless_ramp.vel sign.
        self.axis.config.enable_sensorless_mode = True

    def stop(self) -> None:
        try:
            self.axis.controller.input_vel = 0
        except Exception:
            pass
        try:
            self.axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

    def coast(self, seconds: float = 1.0) -> None:
        # Drop to IDLE so rotor coasts (less regen/braking).
        self.stop()
        time.sleep(max(0.0, float(seconds)))

    def calibrate(self, *, timeout_s: float = 45.0) -> None:
        self._log("Motor calibration...")
        self._clear_errors()

        self.axis.motor.config.pre_calibrated = False
        self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        entered = False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            st = self.axis.current_state
            if st == AXIS_STATE_MOTOR_CALIBRATION:
                entered = True
            if st == AXIS_STATE_IDLE and entered:
                self._clear_errors()
                return
            if st == AXIS_STATE_IDLE and not entered and (time.time() - t0) > 2.0:
                dump_errors(self.odr)
                raise RuntimeError("MOTOR_CALIBRATION did not start (axis stayed IDLE)")
            time.sleep(0.1)

        dump_errors(self.odr)
        raise TimeoutError("Motor calibration timed out")

    def _prepare_sensorless_direction_and_min_speed(self, vel_turn_s: float) -> float:
        vel = float(vel_turn_s)
        if abs(vel) < float(self.config.min_sensorless_vel_turn_s):
            vel = math.copysign(float(self.config.min_sensorless_vel_turn_s), vel if vel != 0 else 1.0)

        # axis.config.sensorless_ramp.vel is electrical rad/s.
        ramp_vel = vel * (2.0 * math.pi * float(self.config.pole_pairs))
        try:
            self.axis.config.sensorless_ramp.vel = float(ramp_vel)
        except Exception:
            pass

        return vel

    def _enter_closed_loop(self, *, settle_s: float = 0.8) -> None:
        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(max(0.0, float(settle_s)))
        if self.axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL or self.axis.error != 0:
            dump_errors(self.odr)
            raise RuntimeError(f"Failed to enter CLOSED_LOOP_CONTROL (state={self.axis.current_state} err={self.axis.error})")

    def apply_velocity(
        self,
        vel_turn_s: float,
        duration_s: float,
        *,
        coast_before: float | None = None,
        reverse_kick_vel: float | None = None,
        reverse_kick_seconds: float = 0.5,
    ) -> None:
        """Apply a velocity for a duration.

        If vel is negative and very small, sensorless can fail to reverse; use reverse_kick_vel.
        """
        if coast_before is not None and coast_before > 0:
            self.coast(coast_before)

        eff_vel = self._prepare_sensorless_direction_and_min_speed(vel_turn_s)
        self._enter_closed_loop()

        # Optional kick for reliable reversal at tiny reverse speeds
        if reverse_kick_vel is not None and eff_vel != 0 and math.copysign(1.0, reverse_kick_vel) != math.copysign(1.0, eff_vel):
            reverse_kick_vel = -reverse_kick_vel
        if reverse_kick_vel is not None and abs(eff_vel) < abs(float(reverse_kick_vel)):
            self._log(f"Reverse kick: {math.copysign(abs(reverse_kick_vel), eff_vel)} for {reverse_kick_seconds}s")
            self._run_for(math.copysign(abs(float(reverse_kick_vel)), eff_vel), reverse_kick_seconds)
            remaining = float(duration_s) - float(reverse_kick_seconds)
            if remaining > 0:
                self._run_for(eff_vel, remaining)
        else:
            self._run_for(eff_vel, duration_s)

    def _run_for(self, vel_turn_s: float, duration_s: float) -> None:
        self.axis.controller.input_vel = float(vel_turn_s)
        t0 = time.time()
        while time.time() - t0 < float(duration_s):
            if self.axis.error != 0 or self.axis.motor.error != 0:
                dump_errors(self.odr)
                raise RuntimeError(f"Fault during run (axis_err={self.axis.error} motor_err={self.axis.motor.error})")
            time.sleep(0.02)
        self.axis.controller.input_vel = 0.0

    def close(self) -> None:
        self.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
