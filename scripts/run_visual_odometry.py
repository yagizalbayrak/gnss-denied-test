"""Run stereo VO over a KITTI drive, cache its motion and compare it with ground truth."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import gtsam
import numpy as np
import yaml

from gnss_denied.core.geodesy import LocalEnuFrame
from gnss_denied.core.types import GroundTruthSample
from gnss_denied.datasets.kitti_raw import KittiRawDrive, load_stereo_calibration
from gnss_denied.frontend.visual_odom import (
    FeatureBackend,
    FrameOdometry,
    OrbBackend,
    StereoVisualOdometry,
    VoNoise,
    load_odometry,
    save_odometry,
)


def make_backend(name: str) -> FeatureBackend:
    if name == "orb":
        return OrbBackend()
    if name == "aliked":
        # Imported here so that the classic baseline does not need PyTorch.
        from gnss_denied.frontend.deep_features import AlikedLightGlueBackend

        return AlikedLightGlueBackend()
    raise SystemExit(f"unknown backend: {name}")


class TruthInterpolator:
    """Ground truth pose in the local frame at any time, from the 100 Hz reference."""

    def __init__(self, truth: list[GroundTruthSample]) -> None:
        self._times = np.array([sample.timestamp for sample in truth])
        frame = LocalEnuFrame(truth[0].lat_deg, truth[0].lon_deg, truth[0].alt_m)
        self._positions = np.array(
            [frame.from_geodetic(s.lat_deg, s.lon_deg, s.alt_m) for s in truth]
        )
        self._rotations = [gtsam.Rot3.Ypr(s.yaw_rad, s.pitch_rad, s.roll_rad) for s in truth]

    def covers(self, timestamp: float) -> bool:
        """False near a recording hole, where interpolating over it would be fiction."""
        index = int(np.searchsorted(self._times, timestamp))
        if index == 0 or index == len(self._times):
            return False
        return self._times[index] - self._times[index - 1] < 0.05

    def pose(self, timestamp: float) -> gtsam.Pose3:
        index = int(np.searchsorted(self._times, timestamp))
        t0, t1 = self._times[index - 1], self._times[index]
        alpha = (timestamp - t0) / (t1 - t0)
        rotation0, rotation1 = self._rotations[index - 1], self._rotations[index]
        rotation = rotation0.compose(
            gtsam.Rot3.Expmap(alpha * gtsam.Rot3.Logmap(rotation0.between(rotation1)))
        )
        position = (1.0 - alpha) * self._positions[index - 1] + alpha * self._positions[index]
        return gtsam.Pose3(rotation, position)


def report(records: list[FrameOdometry], truth: TruthInterpolator, window=(30.0, 90.0)) -> None:
    constraints = [record.constraint for record in records if record.constraint is not None]
    print(f"frames: {len(records)}, tracking losses: {len(records) - len(constraints) - 1}")
    inliers = np.array([constraint.num_inliers for constraint in constraints])
    print(f"inliers: median {np.median(inliers):.0f}, min {inliers.min()}")

    rotation_errors, translation_errors = [], []
    for constraint in constraints:
        if not (truth.covers(constraint.timestamp_from) and truth.covers(constraint.timestamp_to)):
            continue
        true_motion = truth.pose(constraint.timestamp_from).between(
            truth.pose(constraint.timestamp_to)
        )
        w, x, y, z = constraint.rotation_wxyz
        rotation = gtsam.Rot3.Quaternion(w, x, y, z)
        rotation_errors.append(gtsam.Rot3.Logmap(true_motion.rotation().between(rotation)))
        translation_errors.append(constraint.translation_m - true_motion.translation())
    rotation_errors, translation_errors = np.array(rotation_errors), np.array(translation_errors)
    print("per-frame error against truth, x forward / y left / z up:")
    print(f"  rotation std (rad): {np.round(rotation_errors.std(axis=0), 5)}")
    print(f"  rotation 95% (rad): {np.round(np.percentile(abs(rotation_errors), 95, axis=0), 5)}")
    print(f"  translation mean (m): {np.round(translation_errors.mean(axis=0), 4)}")
    print(f"  translation std (m):  {np.round(translation_errors.std(axis=0), 4)}")
    print(f"  translation max (m):  {np.round(np.abs(translation_errors).max(axis=0), 3)}")

    # VO alone, chained from the true pose at the start of the blackout.
    start_s, end_s = window
    in_window = [c for c in constraints if start_s <= c.timestamp_from and c.timestamp_to <= end_s]
    pose = truth.pose(in_window[0].timestamp_from)
    path_m = 0.0
    for constraint in in_window:
        w, x, y, z = constraint.rotation_wxyz
        motion = gtsam.Pose3(gtsam.Rot3.Quaternion(w, x, y, z), constraint.translation_m)
        pose = pose.compose(motion)
        path_m += float(np.linalg.norm(constraint.translation_m))
    final = truth.pose(in_window[-1].timestamp_to)
    drift = np.linalg.norm((pose.translation() - final.translation())[:2])
    print(
        f"VO alone over [{start_s:.0f}, {end_s:.0f}] s: horizontal drift {drift:.2f} m "
        f"over {path_m:.0f} m ({100.0 * drift / path_m:.2f}%)"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/default_sim.yaml"))
    parser.add_argument("--backend", default="orb", choices=["orb", "aliked"])
    parser.add_argument(
        "--max-frames", type=int, help="speed test on the first N frames; nothing is cached"
    )
    parser.add_argument("--evaluate-only", action="store_true", help="re-score an existing cache")
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    dataset = config["dataset"]
    drive = KittiRawDrive(dataset["root"], dataset["date"], dataset["drive"])
    noise = VoNoise(**config["visual_odometry"][args.backend])
    cache_path = Path("data/cache") / f"vo_{dataset['date']}_{dataset['drive']}_{args.backend}.npz"

    if args.evaluate_only:
        records = load_odometry(cache_path, noise)
    else:
        calibration = load_stereo_calibration(dataset["root"], dataset["date"])
        odometry = StereoVisualOdometry(make_backend(args.backend), calibration, noise=noise)
        records = []
        started = time.perf_counter()
        for index, (frame, right_path) in enumerate(drive.stereo_pairs()):
            if args.max_frames is not None and index >= args.max_frames:
                break
            left = cv2.imread(str(frame.image_path), cv2.IMREAD_GRAYSCALE)
            right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
            records.append(odometry.process(frame.timestamp, left, right))
            if (index + 1) % 500 == 0:
                rate = (index + 1) / (time.perf_counter() - started)
                print(f"  {index + 1} frames, {rate:.1f} fps", flush=True)
        elapsed_s = time.perf_counter() - started
        print(f"{elapsed_s:.0f} s in total, {len(records) / elapsed_s:.1f} fps")
        if args.max_frames is not None:
            inliers = [r.constraint.num_inliers for r in records if r.constraint is not None]
            print(f"tracked {len(inliers)}/{len(records) - 1}, median inliers {np.median(inliers)}")
            return 0
        save_odometry(cache_path, records)
        print(f"cache: {cache_path}")

    report(records, TruthInterpolator(list(drive.ground_truth())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
