from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Any

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
    AXIS_STATE_IDLE,
    CONTROL_MODE_TORQUE_CONTROL,
    ENCODER_MODE_INCREMENTAL,
    INPUT_MODE_PASSTHROUGH,
    MOTOR_TYPE_PMSM_CURRENT_CONTROL,
)
from odrive.utils import dump_errors

from .config import ODriveSafetyConfig
from .errors import CriticalWheelError


def _windows_usb_troubleshooting_hint() -> str:
    return (
        "Windows USB hint: The ODrive Python API uses libusb. If you see '[UsbDiscoverer] Failed to open USB device: -5', "
        "it typically means the ODrive interface is not using a WinUSB driver or access is denied.\n"
        "- Bind the ODrive 'Native Interface' to WinUSB (Zadig).\n"
        "- Close ODrive GUI/other processes using the device.\n"
        "- Try a different USB port/cable; avoid hubs.\n"
        "- Try running as Administrator."
    )


def _safe_get(obj: Any, attr: str, default: Any = None) -> Any:
    try:
        return getattr(obj, attr)
    except Exception:
        return default


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _fmt_hex(v: int) -> str:
    try:
        return f"0x{int(v) & 0xFFFFFFFF:08X}"
    except Exception:
        return str(v)


@dataclass
class ODriveTelemetry:
    pos_turns: float
    vel_turn_s: float
    axis_error: int
    motor_error: int
    encoder_error: int


class ODriveWheelAxis:
    """ODrive axis wrapper for a force-feedback wheel.

    - Runs encoder-based full calibration on startup.
    - Enters torque control for closed-loop output.
    - Provides safe shutdown.

    Critical errors raise CriticalWheelError.
    """

    def __init__(self, *, config: ODriveSafetyConfig, timeout_s: float = 30.0, verbose: bool = True):
        self._verbose = verbose
        self.cfg = config

        try:
            self.odrv = odrive.find_any(timeout=timeout_s)
        except Exception as exc:
            msg = f"Failed to find/connect to ODrive (timeout={timeout_s}s): {exc}"
            if sys.platform.startswith("win"):
                msg = msg + "\n" + _windows_usb_troubleshooting_hint()
            raise CriticalWheelError(msg) from exc
        self.axis = _safe_get(self.odrv, f"axis{int(config.axis)}")
        if self.axis is None:
            raise CriticalWheelError(f"ODrive has no axis{config.axis}")

        self._log("Clearing errors...")
        try:
            self.odrv.clear_errors()
        except Exception:
            pass
        time.sleep(0.2)

        self._apply_safety_limits()
        self._configure_encoder_incremental_if_possible()
        self._full_calibration()
        self._enter_torque_control_closed_loop()

    def _log(self, msg: str) -> None:
        if self._verbose:
            print(msg)

    def _apply_safety_limits(self) -> None:
        try:
            self.odrv.config.dc_max_negative_current = float(self.cfg.dc_max_negative_current_a)
        except Exception:
            pass

        try:
            m = self.axis.motor.config
            m.motor_type = MOTOR_TYPE_PMSM_CURRENT_CONTROL
            if bool(self.cfg.override_motor_pole_pairs):
                m.pole_pairs = int(self.cfg.pole_pairs)
            m.current_lim = float(self.cfg.current_limit_a)
            m.calibration_current = float(self.cfg.calibration_current_a)
        except Exception:
            pass

        try:
            c = self.axis.controller.config
            c.vel_limit = float(self.cfg.vel_limit_turn_s)
            c.vel_ramp_rate = float(self.cfg.vel_ramp_rate_turn_s2)
        except Exception:
            pass

    def _configure_encoder_incremental_if_possible(self) -> None:
        enc_cfg = self.axis.encoder.config

        if bool(self.cfg.override_encoder_mode):
            try:
                enc_cfg.mode = ENCODER_MODE_INCREMENTAL
            except Exception:
                pass

        if bool(self.cfg.override_encoder_cpr):
            try:
                enc_cfg.cpr = int(self._effective_encoder_cpr())
            except Exception:
                pass

        try:
            enc_cfg.use_index = bool(self.cfg.encoder_use_index)
        except Exception:
            pass

        try:
            enc_cfg.bandwidth = float(self.cfg.encoder_bandwidth)
        except Exception:
            pass

        self._print_preflight_config()

    def _effective_encoder_cpr(self) -> int:
        base = float(self.cfg.encoder_cpr)
        mt = int(self.cfg.motor_pulley_teeth)
        et = int(self.cfg.encoder_pulley_teeth)
        if mt > 0 and et > 0:
            base = base * (float(mt) / float(et))
        return int(round(base))

    def _print_preflight_config(self) -> None:
        try:
            live_pp = _safe_get(self.axis.motor.config, "pole_pairs", None)
        except Exception:
            live_pp = None
        try:
            live_cpr = _safe_get(self.axis.encoder.config, "cpr", None)
        except Exception:
            live_cpr = None
        try:
            live_mode = _safe_get(self.axis.encoder.config, "mode", None)
        except Exception:
            live_mode = None

        self._log(
            "Preflight config: "
            f"motor.pole_pairs={live_pp} (override={int(bool(self.cfg.override_motor_pole_pairs))}) "
            f"encoder.cpr={live_cpr} (override={int(bool(self.cfg.override_encoder_cpr))}, effective={self._effective_encoder_cpr()}) "
            f"encoder.mode={live_mode} (override={int(bool(self.cfg.override_encoder_mode))}) "
            f"use_index={int(bool(self.cfg.encoder_use_index))}"
        )

    def _full_calibration(self, timeout_s: float = 60.0) -> None:
        self._log("Full calibration (motor + encoder)...")
        try:
            self.odrv.clear_errors()
        except Exception:
            pass

        self.axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        entered = False
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            st = int(_safe_get(self.axis, "current_state", 0) or 0)
            if st != AXIS_STATE_IDLE:
                entered = True

            if st == AXIS_STATE_IDLE and entered:
                axis_err = int(_safe_get(self.axis, "error", 0) or 0)
                motor_err = int(_safe_get(self.axis.motor, "error", 0) or 0)
                enc_err = int(_safe_get(self.axis.encoder, "error", 0) or 0)
                if axis_err or motor_err or enc_err:
                    dump_errors(self.odrv)
                    raise CriticalWheelError(
                        "Calibration ended with faults "
                        f"(axis_err={_fmt_hex(axis_err)} motor_err={_fmt_hex(motor_err)} enc_err={_fmt_hex(enc_err)}). "
                        "If you see CPR_POLEPAIRS_MISMATCH, set the correct --encoder-cpr and pulley teeth ratio."
                    )

                enc_ready = _safe_get(self.axis.encoder, "is_ready", None)
                if enc_ready is not None and not bool(enc_ready):
                    dump_errors(self.odrv)
                    raise CriticalWheelError("Encoder not ready after calibration")
                return

            time.sleep(0.1)

        dump_errors(self.odrv)
        raise CriticalWheelError("Full calibration timed out")

    def _enter_torque_control_closed_loop(self) -> None:
        self._log("Entering CLOSED_LOOP_CONTROL (torque mode)...")
        try:
            c = self.axis.controller.config
            c.control_mode = CONTROL_MODE_TORQUE_CONTROL
            c.input_mode = INPUT_MODE_PASSTHROUGH
        except Exception:
            pass

        self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.8)

        st = int(_safe_get(self.axis, "current_state", 0) or 0)
        axis_err = int(_safe_get(self.axis, "error", 0) or 0)
        if st != AXIS_STATE_CLOSED_LOOP_CONTROL or axis_err != 0:
            dump_errors(self.odrv)
            raise CriticalWheelError(f"Failed to enter CLOSED_LOOP_CONTROL (state={st} axis_err={axis_err})")

        # Ensure zero torque on entry.
        self.set_torque_nm(0.0)

    def read_telemetry(self) -> ODriveTelemetry:
        enc = self.axis.encoder
        pos = float(_safe_get(enc, "pos_estimate", float("nan")))
        vel = float(_safe_get(enc, "vel_estimate", float("nan")))

        axis_err = int(_safe_get(self.axis, "error", 0) or 0)
        motor_err = int(_safe_get(self.axis.motor, "error", 0) or 0)
        enc_err = int(_safe_get(enc, "error", 0) or 0)

        if any(e != 0 for e in (axis_err, motor_err, enc_err)):
            dump_errors(self.odrv)
            raise CriticalWheelError(f"ODrive fault (axis_err={axis_err} motor_err={motor_err} enc_err={enc_err})")

        # Overspeed guard (software): if we're beyond vel_limit, stop.
        if not math.isnan(vel) and abs(vel) > float(self.cfg.vel_limit_turn_s) * 1.25:
            raise CriticalWheelError(f"Overspeed: encoder.vel_estimate={vel:.3f} turn/s")

        if math.isnan(pos) or math.isnan(vel):
            raise CriticalWheelError("Invalid encoder readings (NaN)")

        return ODriveTelemetry(
            pos_turns=pos,
            vel_turn_s=vel,
            axis_error=axis_err,
            motor_error=motor_err,
            encoder_error=enc_err,
        )

    def set_torque_nm(self, torque_nm: float) -> None:
        # Clamp is handled at a higher level too; keep a sane bound here.
        cmd = float(torque_nm)
        if not math.isfinite(cmd):
            cmd = 0.0

        try:
            self.axis.controller.input_torque = cmd
        except Exception as exc:
            raise CriticalWheelError(f"Failed to set torque: {exc}")

    def disable_output(self) -> None:
        try:
            self.set_torque_nm(0.0)
        except Exception:
            pass
        try:
            self.axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass

    def close(self) -> None:
        self.disable_output()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False
