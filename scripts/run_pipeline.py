"""Run IMU + GNSS fusion on a KITTI drive; print errors and write a plot and a Rerun recording."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

from gnss_denied.datasets.kitti_raw import KittiRawDrive
from gnss_denied.frontend.visual_odom import VoNoise, load_odometry
from gnss_denied.pipeline import FusionStep, load_imu_profile, run_fusion, window_errors
from gnss_denied.simulator.gnss_degrader import (
    DegradationMode,
    GnssDegrader,
    load_degrader_config,
)
from gnss_denied.simulator.imu_degrader import corrupt_imu
from gnss_denied.viz.plots import plot_blackout


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/default_sim.yaml"))
    parser.add_argument("--imu-profile", default="mems", help="a key of imu_profiles in the config")
    parser.add_argument("--vo", default="none", help="a cached VO backend, for example orb")
    parser.add_argument("--plot", type=Path, help="default: assets/blackout_<profile>_<vo>.png")
    parser.add_argument("--rrd", type=Path, help="default: assets/blackout_<profile>_<vo>.rrd")
    parser.add_argument("--spawn", action="store_true", help="stream live to a Rerun viewer")
    parser.add_argument("--no-rerun", action="store_true", help="skip the Rerun recording")
    args = parser.parse_args()

    run_name = f"blackout_{args.imu_profile}_{args.vo}"
    plot_path = args.plot or Path(f"assets/{run_name}.png")
    rrd_path = args.rrd or Path(f"assets/{run_name}.rrd")

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    dataset = config["dataset"]
    drive = KittiRawDrive(dataset["root"], dataset["date"], dataset["drive"])
    truth = list(drive.ground_truth())
    degrader_config = load_degrader_config(args.config)
    gnss = list(GnssDegrader(degrader_config).degrade(truth))
    profile = load_imu_profile(args.config, args.imu_profile)
    imu = drive.imu() if profile.injected is None else corrupt_imu(drive.imu(), profile.injected)

    odometry = []
    if args.vo != "none":
        cache_name = f"vo_{dataset['date']}_{dataset['drive']}_{args.vo}.npz"
        noise = VoNoise(**config["visual_odometry"][args.vo])
        odometry = load_odometry(Path("data/cache") / cache_name, noise)

    on_step = None
    if not args.no_rerun:
        # Imported here so that plain runs do not need the Rerun SDK.
        from gnss_denied.viz.rerun_logger import RerunLogger

        logger = RerunLogger(spawn=args.spawn, save_path=None if args.spawn else rrd_path)

        def on_step(step: FusionStep) -> None:
            logger.log_step(step)

    result = run_fusion(
        imu,
        truth,
        gnss,
        profile.estimator,
        on_step=on_step,
        initial_sigmas=profile.initial_sigmas,
        odometry=odometry,
    )

    blackouts = [w for w in degrader_config.windows if w.mode is DegradationMode.BLACKOUT]
    if not blackouts:
        raise SystemExit("this report expects a blackout window in the config")
    start_s, end_s = blackouts[0].start_s, blackouts[0].end_s

    truth_by_time = result.truth_enu_by_time
    fused = [step.fused for step in result.steps]
    reckoned = [step.dead_reckoning for step in result.steps]
    rows = [
        window_errors("Fused, before blackout", fused, truth_by_time, 5.0, start_s - 0.5),
        window_errors("Fused, blackout (online)", fused, truth_by_time, start_s, end_s),
        window_errors("Fused, blackout (smoothed)", result.smoothed, truth_by_time, start_s, end_s),
        window_errors("Fused, after recovery", fused, truth_by_time, end_s + 1.0),
        window_errors("IMU only, blackout window", reckoned, truth_by_time, start_s, end_s),
        window_errors("IMU only, whole drive", reckoned, truth_by_time),
    ]

    print(f"IMU profile: {args.imu_profile}, VO: {args.vo}, keyframes: {result.num_keyframes}")
    print(f"{'':30} {'n':>4} {'RMS m':>9} {'max m':>9} {'end m':>9}")
    for row in rows:
        print(
            f"{row.name:30} {row.samples:4d} {row.horizontal_rms_m:9.2f} "
            f"{row.horizontal_max_m:9.2f} {row.horizontal_end_m:9.2f}"
        )
    print(f"final accel bias: {np.round(result.accel_bias_mps2, 4)}")
    print(f"final gyro bias:  {np.round(result.gyro_bias_radps, 6)}")

    title = f"KITTI {dataset['date']}_{dataset['drive']}, IMU: {args.imu_profile}, VO: {args.vo}"
    plot_blackout(result, (start_s, end_s), title, plot_path)
    print(f"plot: {plot_path}")
    if not args.no_rerun and not args.spawn:
        print(f"recording: {rrd_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
