"""Benchmark the three modes across seeds: a markdown table and a comparison plot.

Run `scripts/run_visual_odometry.py` for each VO backend first, so that its motion is cached.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

from gnss_denied.datasets.kitti_raw import KittiRawDrive
from gnss_denied.evaluation import TrajectoryMetrics, evaluate_result
from gnss_denied.frontend.visual_odom import VoNoise, load_odometry
from gnss_denied.pipeline import load_imu_profile, run_fusion
from gnss_denied.simulator.gnss_degrader import (
    DegradationMode,
    GnssDegrader,
    load_degrader_config,
)
from gnss_denied.simulator.imu_degrader import corrupt_imu
from gnss_denied.viz.plots import ModeSeries, plot_benchmark

MODES = {
    "GNSS+IMU (Blackout)": None,
    "GNSS+IMU+Classic_ORB": "orb",
    "GNSS+IMU+Deep_LightGlue": "aliked",
}
PROFILES = ("raw", "mems")
PLOTTED_PROFILE = "mems"


def _cell(values: list[float]) -> str:
    return f"{np.median(values):.2f} [{min(values):.2f}, {max(values):.2f}]"


def markdown_table(scores: dict[tuple[str, str], list[TrajectoryMetrics]]) -> str:
    lines = [
        "| IMU | Mode | ATE full drive (m) | ATE blackout (m) | End of blackout (m) "
        "| RPE 10 s, translation (m) | RPE 10 s, rotation (deg) |",
        "|---|---|---|---|---|---|---|",
    ]
    for (profile, mode), runs in scores.items():
        lines.append(
            f"| `{profile}` | {mode} "
            f"| {_cell([run[0].ate_full_m for run in runs])} "
            f"| {_cell([run[0].ate_blackout_m for run in runs])} "
            f"| {_cell([run[1] for run in runs])} "
            f"| {_cell([run[0].rpe_translation_blackout_m for run in runs])} "
            f"| {_cell([run[0].rpe_rotation_blackout_deg for run in runs])} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=Path("configs/default_sim.yaml"))
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--output-dir", type=Path, default=Path("assets"))
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    dataset = config["dataset"]
    drive = KittiRawDrive(dataset["root"], dataset["date"], dataset["drive"])
    truth = list(drive.ground_truth())
    raw_imu = list(drive.imu())
    degrader_config = load_degrader_config(args.config)
    window = next(w for w in degrader_config.windows if w.mode is DegradationMode.BLACKOUT)
    blackout = (window.start_s, window.end_s)

    odometry = {}
    for backend in filter(None, MODES.values()):
        cache = Path("data/cache") / f"vo_{dataset['date']}_{dataset['drive']}_{backend}.npz"
        odometry[backend] = load_odometry(cache, VoNoise(**config["visual_odometry"][backend]))

    scores: dict[tuple[str, str], list] = {}
    series: dict[str, ModeSeries] = {}
    for profile_name in PROFILES:
        profile = load_imu_profile(args.config, profile_name)
        for mode, backend in MODES.items():
            runs, errors, first_path = [], [], None
            for seed in range(args.seeds):
                gnss = list(GnssDegrader(replace(degrader_config, seed=seed)).degrade(truth))
                imu = raw_imu
                if profile.injected is not None:
                    imu = list(corrupt_imu(raw_imu, replace(profile.injected, seed=seed)))
                result = run_fusion(
                    imu,
                    truth,
                    gnss,
                    profile.estimator,
                    initial_sigmas=profile.initial_sigmas,
                    odometry=odometry[backend] if backend else (),
                )
                offsets = np.array(
                    [step.fused.position_enu_m - step.truth_enu_m for step in result.steps]
                )
                horizontal = np.linalg.norm(offsets[:, :2], axis=1)
                times = np.array([step.timestamp for step in result.steps])
                end_of_blackout = horizontal[times <= blackout[1]][-1]
                runs.append((evaluate_result(result, truth, blackout), float(end_of_blackout)))
                errors.append(horizontal)
                if first_path is None:
                    first_path = np.array([step.fused.position_enu_m for step in result.steps])
                    truth_path = np.array([step.truth_enu_m for step in result.steps])
            scores[(profile_name, mode)] = runs
            median_end = np.median([run[1] for run in runs])
            print(f"{profile_name} / {mode}: median end of blackout {median_end:.2f} m", flush=True)
            if profile_name == PLOTTED_PROFILE:
                series[mode] = ModeSeries(times, np.array(errors), first_path)

    table = markdown_table(scores)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "benchmark_results.md").write_text(table, encoding="utf-8")
    plot_benchmark(
        series,
        truth_path,
        blackout,
        f"KITTI {dataset['date']}_{dataset['drive']}, IMU: {PLOTTED_PROFILE}, {args.seeds} seeds",
        args.output_dir / "benchmark_results.png",
    )
    print(f"\nEach cell is the median [min, max] over {args.seeds} seeds.\n\n{table}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
