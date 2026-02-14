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
    if name == "none":
        from .backend.none_backend import NoneBackend

        return NoneBackend()
    raise ValueError(f"Unknown backend: {name}")


def _sign(x: float) -> float:
    return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)


def _internal_ffb_force(
    *,
    pos: float,
    vel: float,
    spring_k: float,
    damper_b: float,
    friction: float,
    endstop_k: float,
    endstop_start: float,
) -> float:
    """Simple standalone FFB model (no game input).

    Returns normalized force in [-1, +1].
    """

    pos = clamp(float(pos), -1.0, 1.0)
    vel = float(vel)

    f = 0.0
    f += -float(spring_k) * pos
    f += -float(damper_b) * vel
    if float(friction) != 0.0:
        f += -float(friction) * _sign(vel)

    # Soft endstops near lock. endstop_start is a normalized position (0..1).
    es = clamp(float(endstop_start), 0.0, 1.0)
    if float(endstop_k) > 0.0 and abs(pos) > es:
        # Penetration depth into endstop region.
        d = (abs(pos) - es) / max(1e-6, (1.0 - es))
        f += -float(endstop_k) * d * _sign(pos)

    return clamp(f, -1.0, 1.0)


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
    ap.add_argument("--backend", choices=["auto", "vjoy", "sdl", "none"], default="auto")
    ap.add_argument("--vjoy-id", type=int, default=1, help="vJoy device id (Windows)")

    ap.add_argument(
        "--invert-steering",
        action="store_true",
        help="Invert steering direction (flip left/right). Also flips FFB sign to match.",
    )

    ap.add_argument(
        "--invert-ffb",
        action="store_true",
        help="Invert FFB direction without inverting the steering axis (useful if the game/driver reports FFB sign opposite).",
    )

    ap.add_argument(
        "--ffb",
        choices=["game", "internal", "none"],
        default="game",
        help="FFB source: game (vJoy/SDL), internal (standalone), none (zero force)",
    )
    ap.add_argument(
        "--game-ffb-gain",
        type=float,
        default=1.0,
        help="Multiply game-provided FFB force before converting to torque (use <1.0 if game FFB is too strong)",
    )
    ap.add_argument(
        "--ffb-spring-k",
        type=float,
        default=0.0,
        help="Internal FFB spring gain (also used as an optional overlay on game FFB)",
    )
    ap.add_argument(
        "--ffb-damper-b",
        type=float,
        default=0.0,
        help="Internal FFB damper gain (also used as an optional overlay on game FFB)",
    )
    ap.add_argument(
        "--ffb-friction",
        type=float,
        default=0.0,
        help="Internal FFB Coulomb friction (also used as an optional overlay on game FFB)",
    )
    ap.add_argument(
        "--ffb-endstop-k",
        type=float,
        default=0.0,
        help="Internal FFB endstop gain (also used as an optional overlay on game FFB)",
    )
    ap.add_argument(
        "--ffb-endstop-start",
        type=float,
        default=0.95,
        help="Internal FFB endstop start position (0..1)",
    )

    # Game FFB stability tuning (vJoy EffectState). These are intentionally CLI flags
    # so you can tune per-game without editing code.
    ap.add_argument(
        "--game-ffb-vel-lpf-hz",
        type=float,
        default=25.0,
        help="Low-pass filter cutoff (Hz) for velocity used by game damper/friction (0 disables)",
    )
    ap.add_argument(
        "--game-ffb-vel-zero-eps",
        type=float,
        default=0.02,
        help="Treat |vel| below this as zero when evaluating game damper/friction (reduces idle oscillation)",
    )
    ap.add_argument(
        "--game-ffb-friction-ramp",
        type=float,
        default=0.15,
        help="Velocity range above deadband to ramp friction in (reduces bang-bang at low speed)",
    )

    ap.add_argument("--status-hz", type=float, default=10.0, help="Console status print rate")

    ap.add_argument(
        "--no-vel-sign-autofix",
        action="store_true",
        help="Disable auto-fix for encoder velocity sign (uses ODrive vel_estimate as-is).",
    )

    ap.add_argument("--axis", type=int, default=0, choices=[0, 1], help="ODrive axis index")
    ap.add_argument("--loop-hz", type=float, default=500.0)

    ap.add_argument("--lock-to-lock-turns", type=float, default=2.0)
    ap.add_argument(
        "--wheel-turns-per-motor-turn",
        type=float,
        default=1.0,
        help="Wheel turns per motor turn (gear ratio). 1.0 for direct drive; >1 if wheel turns more than motor.",
    )
    ap.add_argument(
        "--steering-scale",
        type=float,
        default=1.0,
        help="Scale the steering axis output (and FFB position input). Use <1.0 if the game turns too much for a given real angle.",
    )

    ap.add_argument("--current-limit-a", type=float, default=16.0)
    ap.add_argument(
        "--dc-max-negative-current-a",
        type=float,
        default=-5.0,
        help="ODrive dc_max_negative_current (regen limit). Too small can cause asymmetric feel.",
    )
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
    ap.add_argument(
        "--torque-slew-rate-nm-per-s",
        type=float,
        default=0.0,
        help="Optional torque command slew-rate limit (0 disables). Helps prevent runaway if FFB sign/config is wrong.",
    )
    ap.add_argument("--extra-damping", type=float, default=0.0, help="Extra damping Nm per (turn/s)")

    ap.add_argument("--calib", default="novus_wheel_calibration.json", help="Calibration file path")
    ap.add_argument("--recalibrate", action="store_true")
    ap.add_argument("--invert-motor", action="store_true", help="Invert physical torque direction (use if wheel runs away regardless of settings)")
    ap.add_argument("--dry-run", action="store_true", help="Calculate FFB but do not apply torque to motor (debug mode)")
    ap.add_argument(
        "--no-motor-ffb",
        action="store_true",
        help="Do not apply any FFB torque to the motor (torque command forced to 0). Steering axis still works.",
    )
    ap.add_argument(
        "--print-game-ffb-only",
        action="store_true",
        help="Print only the raw FFB packets the game sends (vJoy backend) and suppress normal status output.",
    )

    args = ap.parse_args(argv)

    cfg = AppConfig(
        odrive=ODriveSafetyConfig(
            axis=int(args.axis),
            current_limit_a=float(args.current_limit_a),
            dc_max_negative_current_a=float(args.dc_max_negative_current_a),
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

    # Only open a backend when we actually want to expose a device.
    # (backend=none is a no-op anyway, but keep intent explicit.)
    backend.open()

    # Apply game-FFB stability tuning to the vJoy effect model (if present).
    eff_state = getattr(backend, "effect_state", None)
    if eff_state is not None:
        try:
            if hasattr(eff_state, "vel_lpf_hz"):
                eff_state.vel_lpf_hz = float(args.game_ffb_vel_lpf_hz)
            if hasattr(eff_state, "vel_zero_epsilon"):
                eff_state.vel_zero_epsilon = float(args.game_ffb_vel_zero_eps)
            if hasattr(eff_state, "friction_ramp"):
                eff_state.friction_ramp = float(args.game_ffb_friction_ramp)
        except Exception:
            pass

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

            # Safety check for calibration drift (common with incremental encoders)
            tel_start = od.read_telemetry()
            calib_diff = abs(float(tel_start.pos_turns) - float(calib.center_motor_turns))
            if calib_diff > 0.5:
                print(f"\n[WARNING] Calibration Mismatch Detected!")
                print(f"Current Position: {tel_start.pos_turns:.2f} turns")
                print(f"Stored Center:    {calib.center_motor_turns:.2f} turns")
                print(f"Difference:       {calib_diff:.2f} turns")
                print("This large offset will cause violence/runaway FFB immediately on startup.")
                print("SOLUTION: Run with --recalibrate to fix the center position.\n")

            # Main loop
            period = 1.0 / max(1.0, float(cfg.loop_hz))
            torque_lpf = _OnePoleLPF(hz=float(cfg.ffb.torque_lpf_hz), initial=0.0)
            last_torque_cmd = 0.0

            # Velocity sign auto-fix: if ODrive vel_estimate sign is flipped relative to
            # position changes, velocity-based effects (damper/friction) will be inverted.
            last_wheel_turns: float | None = None
            vel_sign_ema = 1.0  # +1 => consistent, -1 => inverted
            vel_sign_alpha = 0.05
            vel_sign_min_turn_s = 0.02

            print("Novus Wheel running. Ctrl-C to stop.")

            last = time.perf_counter()
            status_period = 1.0 / max(0.1, float(args.status_hz))
            next_status = last
            last_dbg_seq = 0
            while True:
                now = time.perf_counter()
                dt = now - last
                if dt < period:
                    time.sleep(max(0.0, period - dt))
                    now = time.perf_counter()
                last = now

                tel = od.read_telemetry()

                # Compute both unclamped and clamped normalized position.
                wheel_turns = (float(tel.pos_turns) - float(calib.center_motor_turns)) * float(
                    cfg.wheel.wheel_turns_per_motor_turn
                )

                # Best-effort wheel velocity estimate from position difference.
                wheel_vel_turn_s_diff = 0.0
                if last_wheel_turns is not None and float(dt) > 0.0:
                    wheel_vel_turn_s_diff = (float(wheel_turns) - float(last_wheel_turns)) / float(dt)
                last_wheel_turns = float(wheel_turns)
                half = float(cfg.wheel.lock_to_lock_turns) / 2.0
                pos_unclamped = 0.0 if half <= 0.0 else (float(wheel_turns) / float(half))
                pos_n = clamp(float(pos_unclamped), -1.0, 1.0)
                # Wheel velocity from ODrive estimate (converted through gear ratio).
                wheel_vel_turn_s_odrive = float(tel.vel_turn_s) * float(cfg.wheel.wheel_turns_per_motor_turn)

                # Auto-detect if ODrive velocity sign is inverted relative to position delta.
                if not bool(getattr(args, "no_vel_sign_autofix", False)):
                    if abs(float(wheel_vel_turn_s_odrive)) > float(vel_sign_min_turn_s) and abs(
                        float(wheel_vel_turn_s_diff)
                    ) > float(vel_sign_min_turn_s):
                        same = 1.0 if (float(wheel_vel_turn_s_odrive) * float(wheel_vel_turn_s_diff)) >= 0.0 else -1.0
                        vel_sign_ema = (1.0 - float(vel_sign_alpha)) * float(vel_sign_ema) + float(vel_sign_alpha) * float(same)

                vel_sign = -1.0 if float(vel_sign_ema) < 0.0 else 1.0
                wheel_vel_turn_s = float(wheel_vel_turn_s_odrive) * float(vel_sign)
                # Normalize velocity by half-turn per second to get into a useful range.
                vel_n = clamp(float(wheel_vel_turn_s) / 2.0, -3.0, 3.0)

                # Physical coordinates (match what we send to ODrive after sign conversion).
                pos_phys_unclamped = float(pos_unclamped)
                pos_phys = float(pos_n)
                vel_phys = float(vel_n)

                steering_sign = -1.0 if bool(args.invert_steering) else 1.0
                # Values in the game's coordinate system (matches what we expose on the virtual axis)
                pos_game_unclamped = steering_sign * float(pos_unclamped)
                # Apply user scale before clamping to vJoy range.
                pos_game_unclamped = float(pos_game_unclamped) * float(args.steering_scale)
                pos_game = clamp(float(pos_game_unclamped), -1.0, 1.0)
                vel_game = steering_sign * float(vel_n)

                backend.update_axis(WheelAxisState(steering=float(pos_game)))

                # --- Raw game FFB monitor mode (no motor output) ---
                # Prints only what the game is sending via vJoy, independent of wheel movement.
                if bool(getattr(args, "print_game_ffb_only", False)):
                    eff_state = getattr(backend, "effect_state", None)
                    if eff_state is not None and hasattr(eff_state, "get_packet_debug"):
                        try:
                            seq, msg, _tmono = eff_state.get_packet_debug()
                        except Exception:
                            seq, msg = 0, ""
                        if int(seq) != int(last_dbg_seq) and str(msg).strip():
                            last_dbg_seq = int(seq)
                            print(str(msg))

                    # Force motor torque to 0 in this mode.
                    try:
                        od.set_torque_nm(0.0)
                    except Exception:
                        pass
                    continue

                # --- FFB Force Computation ---
                force_n = 0.0
                game_ffb_raw = None
                if str(args.ffb) == "internal":
                    force_n = _internal_ffb_force(
                        pos=float(pos_game),
                        vel=float(vel_game),
                        spring_k=float(args.ffb_spring_k),
                        damper_b=float(args.ffb_damper_b),
                        friction=float(args.ffb_friction),
                        endstop_k=float(args.ffb_endstop_k),
                        endstop_start=float(args.ffb_endstop_start),
                    )
                elif str(args.ffb) == "game":
                    # --invert-ffb flips only directional effects (constant force from game).
                    # Condition effects (damper, spring, friction) derive correct sign from
                    # position/velocity physics and must NOT be flipped.
                    need_dir_invert = bool(getattr(args, "invert_ffb", False))
                    if float(steering_sign) < 0.0:
                        need_dir_invert = not need_dir_invert

                    # Prefer the per-slot effect state (vJoy backend).
                    eff_state = getattr(backend, "effect_state", None)
                    if eff_state is not None and hasattr(eff_state, "compute_force"):
                        force_n = float(eff_state.compute_force(
                            pos=float(pos_game),
                            vel=float(vel_game),
                            invert_directional=bool(need_dir_invert),
                        ))
                        game_ffb_raw = eff_state
                    else:
                        cmd = backend.poll_ffb()
                        if cmd is not None:
                            force_n = clamp(float(cmd.force), -1.0, 1.0)
                            game_ffb_raw = cmd
                        # Legacy path: apply uniform flip.
                        force_n = float(steering_sign) * float(force_n)
                        if bool(getattr(args, "invert_ffb", False)):
                            force_n = -float(force_n)

                    force_n = clamp(float(force_n) * float(getattr(args, "game_ffb_gain", 1.0)), -1.0, 1.0)
                # else ffb=="none": force_n stays 0.0

                # Hard endstop override: if we are beyond the lock-to-lock range,
                # add a restoring force + damping to pull the wheel back.
                #
                # NOTE: This can feel like a one-sided “runaway” if center calibration is off
                # or lock-to-lock is misconfigured, because one direction may hit the endstop
                # region early. Use a small tolerance so tiny overshoots don't slam torque.
                over = abs(float(pos_phys_unclamped)) - 1.0
                endstop_tol = 0.02
                endstop_active = over > float(endstop_tol)
                if endstop_active:
                    inward = -_sign(float(pos_phys_unclamped))
                    pull = clamp(float(over - endstop_tol) * 8.0, 0.0, 1.0)
                    brake = clamp(-0.3 * float(vel_phys), -1.0, 1.0)
                    force_n = clamp(float(force_n) + float(inward) * float(pull) + float(brake), -1.0, 1.0)

                # Extra damping based on real wheel velocity (Nm per turn/s).
                # NOTE: `wheel_turns_per_motor_turn` impacts both kinematics and torque.
                # If wheel turns faster than motor (ratio > 1), the wheel sees *less* torque
                # than the motor produces (ignoring losses). For consistent “wheel torque” feel
                # across gearing, we interpret `max_torque_nm` as a *wheel-side* torque limit and
                # convert to motor-side torque by multiplying by the ratio.
                ratio = float(cfg.wheel.wheel_turns_per_motor_turn)
                ratio_abs = abs(ratio) if abs(ratio) > 1e-6 else 1.0

                wheel_vel_turn_s = float(tel.vel_turn_s) * float(cfg.wheel.wheel_turns_per_motor_turn)
                extra_wheel = -float(cfg.ffb.extra_damping_nm_per_turn_s) * float(wheel_vel_turn_s)

                wheel_torque_cmd = float(force_n) * float(cfg.ffb.max_torque_nm) + float(extra_wheel)
                wheel_torque_cmd = clamp(
                    float(wheel_torque_cmd),
                    -float(cfg.ffb.max_torque_nm),
                    float(cfg.ffb.max_torque_nm),
                )

                torque_cmd = float(wheel_torque_cmd) * float(ratio_abs)
                torque_cmd = clamp(
                    float(torque_cmd),
                    -float(cfg.ffb.max_torque_nm) * float(ratio_abs),
                    float(cfg.ffb.max_torque_nm) * float(ratio_abs),
                )

                # Optional slew-rate limiter (Nm/s). This is a safety net for unstable FFB.
                slew = float(getattr(args, "torque_slew_rate_nm_per_s", 0.0) or 0.0)
                if slew > 0.0:
                    max_delta = float(slew) / max(1.0, float(cfg.loop_hz))
                    torque_cmd = clamp(
                        float(torque_cmd),
                        float(last_torque_cmd) - float(max_delta),
                        float(last_torque_cmd) + float(max_delta),
                    )
                    last_torque_cmd = float(torque_cmd)

                torque_cmd = torque_lpf.update(torque_cmd, now)

                if bool(args.invert_motor):
                    torque_cmd = -torque_cmd

                if bool(args.dry_run) or bool(getattr(args, "no_motor_ffb", False)):
                    od.set_torque_nm(0.0)
                else:
                    od.set_torque_nm(float(torque_cmd))

                if now >= next_status:
                    next_status = now + status_period
                    print(
                        "pos={:+.3f} vel={:+.3f} posRaw={:+.3f} over={:+.3f} endstop={} posGame={:+.3f} ffb={:+.3f} torqueNm={:+.3f} gameFFB={}".format(
                            float(pos_n),
                            float(vel_n),
                            float(pos_phys_unclamped),
                            float(over),
                            int(bool(endstop_active)),
                            float(pos_game),
                            float(force_n),
                            float(torque_cmd),
                            game_ffb_raw,
                        )
                    )

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
