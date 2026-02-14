#!/usr/bin/env python3

import argparse

from odrive_movement import Axis0Config, ODriveAxis0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--diag", action="store_true", help="Print encoder diagnostics and watch counts, then exit")
    ap.add_argument("--measure-cpr", action="store_true", help="Interactively measure encoder CPR (rotate 1 rev by hand)")
    ap.add_argument("--force-sensorless", action="store_true", help="Skip encoder attempt and run sensorless")
    ap.add_argument("--encoder-cpr", type=int, default=4000, help="Incremental encoder counts/rev (CPR, typically 4*PPR)")
    ap.add_argument("--use-index", action="store_true", help="Use encoder Z index (ABZ encoders only)")
    args = ap.parse_args()

    cfg = Axis0Config(
        pole_pairs=7,
        current_limit=16.0,
        calibration_current=6.0,
        dc_max_negative_current=-5.0,
        vel_limit=30.0,
        vel_ramp_rate=2.0,
        prefer_encoder=not args.force_sensorless,
        fallback_to_sensorless=True,
        min_sensorless_vel_turn_s=0.2,
        encoder_cpr=args.encoder_cpr,
        encoder_use_index=args.use_index,
    )

    with ODriveAxis0(config=cfg, verbose=True) as od:
        print(f"Using mode: {od.mode}")

        if args.measure_cpr:
            # This works even if encoder calibration failed; it's just reading counts.
            measured = od.measure_encoder_cpr_interactive()
            print(f"Suggestion: rerun with --encoder-cpr {measured}")
            return

        if args.diag:
            od.dump_encoder_diagnostics(watch_seconds=0.0)
            od.stop()
            od.watch_encoder_counts(seconds=5.0, interval=0.25)
            return

        # Movement demo
        od.apply_velocity(2, 5)
        od.apply_velocity(-2, 5)
        od.apply_velocity(2, 5)
        od.apply_velocity(-2, 5)


if __name__ == "__main__":
    main()
