#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import odrive


def _safe_get(obj: Any, attr: str, default: Any = None) -> Any:
    try:
        return getattr(obj, attr)
    except Exception:
        return default


def _get_axis(odrv: Any, axis_index: int) -> Any:
    axis_name = f"axis{axis_index}"
    axis = _safe_get(odrv, axis_name)
    if axis is None:
        raise ValueError(f"ODrive has no {axis_name}")
    return axis


def _fmt_hex(value: Any) -> str:
    try:
        return f"0x{int(value) & 0xFFFFFFFF:08X}"
    except Exception:
        return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Live encoder readout for ODrive. Continuously prints encoder position (turns) "
            "and velocity (turns/s)."
        )
    )
    parser.add_argument("--axis", type=int, default=0, choices=[0, 1], help="Axis index (0 or 1)")
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Update rate in Hz (e.g. 10, 50, 100)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="USB discovery timeout seconds for odrive.find_any()",
    )
    parser.add_argument(
        "--reconnect",
        action="store_true",
        help="Keep trying to reconnect if ODrive disconnects",
    )
    parser.add_argument(
        "--newline",
        action="store_true",
        help="Print one line per sample (default updates a single line)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Also show raw counts (shadow_count, count_in_cpr) when available",
    )
    args = parser.parse_args()

    if args.rate <= 0:
        raise SystemExit("--rate must be > 0")

    period_s = 1.0 / args.rate

    while True:
        try:
            print(f"Finding ODrive (timeout={args.timeout}s)...")
            odrv = odrive.find_any(timeout=args.timeout)
            axis = _get_axis(odrv, args.axis)
            enc = axis.encoder

            fw = _safe_get(odrv, "fw_version", None)
            serial = _safe_get(odrv, "serial_number", None)
            if serial is None:
                serial = _safe_get(odrv, "serial", None)

            print(
                "Connected." + (f" serial={serial}" if serial is not None else "") + (f" fw={fw}" if fw is not None else "")
            )
            print(
                "Ctrl-C to stop. Reading: encoder.pos_estimate (turns), encoder.vel_estimate (turns/s)."
            )

            last_print = 0.0
            while True:
                now = time.time()
                # Keep time.sleep stable-ish if printing slows down.
                if now - last_print < period_s:
                    time.sleep(max(0.0, period_s - (now - last_print)))
                last_print = time.time()

                pos = _safe_get(enc, "pos_estimate", float("nan"))
                vel = _safe_get(enc, "vel_estimate", float("nan"))

                axis_state = _safe_get(axis, "current_state", None)
                axis_err = _safe_get(axis, "error", None)
                enc_err = _safe_get(enc, "error", None)
                enc_ready = _safe_get(enc, "is_ready", None)

                parts = [
                    f"pos={float(pos):+.6f} turns",
                    f"vel={float(vel):+.4f} turns/s",
                ]

                if enc_ready is not None:
                    parts.append(f"enc_ready={int(bool(enc_ready))}")
                if axis_state is not None:
                    parts.append(f"axis_state={axis_state}")
                if enc_err is not None:
                    parts.append(f"enc_err={_fmt_hex(enc_err)}")
                if axis_err is not None:
                    parts.append(f"axis_err={_fmt_hex(axis_err)}")

                if args.raw:
                    shadow = _safe_get(enc, "shadow_count", None)
                    in_cpr = _safe_get(enc, "count_in_cpr", None)
                    cpr = _safe_get(enc.config, "cpr", None)
                    if shadow is not None:
                        parts.append(f"shadow_count={int(shadow)}")
                    if in_cpr is not None:
                        parts.append(f"count_in_cpr={int(in_cpr)}")
                    if cpr is not None:
                        parts.append(f"cpr={int(cpr)}")

                line = "  ".join(parts)

                if args.newline:
                    print(line)
                else:
                    # Single-line updating readout.
                    pad = " " * max(0, 120 - len(line))
                    sys.stdout.write("\r" + line + pad)
                    sys.stdout.flush()

        except KeyboardInterrupt:
            print("\nStopped.")
            return 0
        except Exception as exc:
            print(f"\nODrive read failed: {exc}")
            if not args.reconnect:
                return 2
            print("Reconnecting in 1s...")
            time.sleep(1.0)


if __name__ == "__main__":
    raise SystemExit(main())
