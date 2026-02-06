from __future__ import annotations

import argparse
import math
import os
import sys
import time

from .backend.base import VirtualWheelBackend, WheelAxisState
from .backend.macos_sdl import SdlVirtualWheelBackend
from .calibration import Calibration, load_calibration, save_calibration
from .config import AppConfig, FfbTuning, ODriveSafetyConfig, WheelKinematics
from .errors import BackendUnavailableError, CriticalWheelError
from .ffb.effects import clamp
from .odrive_wheel import ODriveWheelAxis


def _pick_backend(name: str, *, vjoy_id: int) -> VirtualWheelBackend:
    if name == "auto":
        if sys.platform == "win32":
            from .backend.windows_vjoy import VJoyWheelBackend

            return VJoyWheelBackend(device_id=vjoy_id)
        return SdlVirtualWheelBackend()
    if name == "vjoy":
        from .backend.windows_vjoy import VJoyWheelBackend

        return VJoyWheelBackend(device_id=vjoy_id)
    if name == "sdl":
        return SdlVirtualWheelBackend()
    raise ValueError(f"Unknown backend: {name}")


def _norm_pos_from_motor_turns(
    *, motor_pos_turns: float, center_motor_turns: float, kin: WheelKinematics
) -> float:
    wheel_turns = (motor_pos_turns - center_motor_turns) * float(kin.wheel_turns_per_motor_turn)
    half = float(kin.lock_to_lock_turns) / 2.0
    if half <= 0:
        return 0.0
    return clamp(wheel_turns / half, -1.0, 1.0)


def _norm_vel_from_motor_turn_s(*, motor_vel_turn_s: float, kin: WheelKinematics) -> float:
    # Normalize velocity by half-turn per second (arbitrary) to get into a useful range.
    # Games typically scale damper internally; we keep it bounded.
    wheel_vel = float(motor_vel_turn_s) * float(kin.wheel_turns_per_motor_turn)
    return clamp(wheel_vel / 2.0, -3.0, 3.0)


class _OnePoleLPF:
    def __init__(self, *, hz: float, initial: float = 0.0):
        self._hz = float(hz)
        self._y = float(initial)
        self._t = None

    def reset(self, value: float = 0.0) -> None:
        self._y = float(value)
        self._t = None

    def update(self, x: float, now: float) -> float:
        if self._hz <= 0:
            self._y = float(x)
            return self._y
        if self._t is None:
            self._t = float(now)
            self._y = float(x)
            return self._y
        dt = max(0.0, float(now) - float(self._t))
        self._t = float(now)
        # alpha = 1 - exp(-2*pi*f*dt)
        alpha = 1.0 - math.exp(-2.0 * math.pi * self._hz * dt) if dt > 0 else 1.0
        self._y = (1.0 - alpha) * self._y + alpha * float(x)
        return self._y


def run(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Novus Wheel: ODrive-backed virtual wheel + FFB")
    ap.add_argument("--backend", choices=["auto", "vjoy", "sdl"], default="auto")
    ap.add_argument("--vjoy-id", type=int, default=1, help="vJoy device id (Windows)")

    ap.add_argument("--axis", type=int, default=0, choices=[0, 1], help="ODrive axis index")
    ap.add_argument("--loop-hz", type=float, default=500.0)

    ap.add_argument("--lock-to-lock-turns", type=float, default=2.0)
    ap.add_argument("--wheel-turns-per-motor-turn", type=float, default=1.0)

    ap.add_argument("--current-limit-a", type=float, default=16.0)
    ap.add_argument("--vel-limit-turn-s", type=float, default=30.0)

    ap.add_argument("--pole-pairs", type=int, default=7)
    ap.add_argument("--no-override-motor-pole-pairs", action="store_true", help="Do not overwrite motor pole_pairs")

    ap.add_argument("--encoder-cpr", type=int, default=4000, help="Encoder counts/rev after quadrature (4x PPR)")
    ap.add_argument("--motor-pulley-teeth", type=int, default=30)
    ap.add_argument("--encoder-pulley-teeth", type=int, default=20)
    ap.add_argument("--no-override-encoder-cpr", action="store_true", help="Do not overwrite encoder CPR")
    ap.add_argument("--no-override-encoder-mode", action="store_true", help="Do not overwrite encoder mode")
    ap.add_argument("--use-index", action="store_true", help="Use encoder Z index (ABZ encoders)")

    ap.add_argument("--max-torque-nm", type=float, default=2.0)
    ap.add_argument("--torque-lpf-hz", type=float, default=60.0)
    ap.add_argument("--extra-damping", type=float, default=0.0, help="Extra damping Nm per (turn/s)")

    ap.add_argument("--calib", default="novus_wheel_calibration.json", help="Calibration file path")
    ap.add_argument("--recalibrate", action="store_true")

    args = ap.parse_args(argv)

    cfg = AppConfig(
        odrive=ODriveSafetyConfig(
            axis=int(args.axis),
            current_limit_a=float(args.current_limit_a),
            vel_limit_turn_s=float(args.vel_limit_turn_s),
            pole_pairs=int(args.pole_pairs),
            override_motor_pole_pairs=not bool(args.no_override_motor_pole_pairs),
            encoder_cpr=int(args.encoder_cpr),
            motor_pulley_teeth=int(args.motor_pulley_teeth),
            encoder_pulley_teeth=int(args.encoder_pulley_teeth),
            override_encoder_cpr=not bool(args.no_override_encoder_cpr),
            override_encoder_mode=not bool(args.no_override_encoder_mode),
            encoder_use_index=bool(args.use_index),
        ),
        wheel=WheelKinematics(
            lock_to_lock_turns=float(args.lock_to_lock_turns),
            wheel_turns_per_motor_turn=float(args.wheel_turns_per_motor_turn),
        ),
        ffb=FfbTuning(
            max_torque_nm=float(args.max_torque_nm),
            torque_lpf_hz=float(args.torque_lpf_hz),
            extra_damping_nm_per_turn_s=float(args.extra_damping),
        ),
        loop_hz=float(args.loop_hz),
        calibration_path=str(args.calib),
    )

    backend = _pick_backend(str(args.backend), vjoy_id=int(args.vjoy_id))

    # Open backend first so games can discover device as soon as calibration completes.
    backend.open()

    try:
        with ODriveWheelAxis(config=cfg.odrive, timeout_s=30.0, verbose=True) as od:
            calib = None if bool(args.recalibrate) else load_calibration(cfg.calibration_path)
            if calib is None:
                print("Calibration: center the wheel, then press Enter...")
                input()
                tel = od.read_telemetry()
                calib = Calibration(center_motor_turns=float(tel.pos_turns))
                save_calibration(cfg.calibration_path, calib)
                print(f"Saved calibration to {cfg.calibration_path}")

            # Main loop
            period = 1.0 / max(1.0, float(cfg.loop_hz))
            torque_lpf = _OnePoleLPF(hz=float(cfg.ffb.torque_lpf_hz), initial=0.0)

            print("Novus Wheel running. Ctrl-C to stop.")

            last = time.perf_counter()
            while True:
                now = time.perf_counter()
                dt = now - last
                if dt < period:
                    time.sleep(max(0.0, period - dt))
                    now = time.perf_counter()
                last = now

                tel = od.read_telemetry()

                pos_n = _norm_pos_from_motor_turns(
                    motor_pos_turns=float(tel.pos_turns),
                    center_motor_turns=float(calib.center_motor_turns),
                    kin=cfg.wheel,
                )
                vel_n = _norm_vel_from_motor_turn_s(motor_vel_turn_s=float(tel.vel_turn_s), kin=cfg.wheel)

                backend.update_axis(WheelAxisState(steering=float(pos_n)))

                # FFB aggregation:
                # - On Windows vJoy, read parsed effects from backend (spring/damper supported).
                force_n = 0.0
                eff = getattr(backend, "effects", None)
                if eff is not None and hasattr(eff, "compute_force"):
                    force_n = float(eff.compute_force(pos=float(pos_n), vel=float(vel_n)))
                else:
                    cmd = backend.poll_ffb()
                    if cmd is not None:
                        force_n = clamp(float(cmd.force), -1.0, 1.0)

                # Extra damping based on real wheel velocity (Nm per turn/s).
                extra = -float(cfg.ffb.extra_damping_nm_per_turn_s) * float(tel.vel_turn_s)

                torque_cmd = float(force_n) * float(cfg.ffb.max_torque_nm) + float(extra)
                torque_cmd = clamp(torque_cmd, -float(cfg.ffb.max_torque_nm), float(cfg.ffb.max_torque_nm))

                torque_cmd = torque_lpf.update(torque_cmd, now)
                od.set_torque_nm(float(torque_cmd))

    except KeyboardInterrupt:
        print("\nStopping...")
        return 0
    except BackendUnavailableError as exc:
        print(f"Backend error: {exc}")
        return 2
    except CriticalWheelError as exc:
        print(f"CRITICAL: {exc}")
        return 3
    except Exception as exc:
        print(f"Unexpected error: {exc}")
        return 4
    finally:
        try:
            backend.close()
        except Exception:
            pass


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
