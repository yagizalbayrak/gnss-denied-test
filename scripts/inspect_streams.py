"""Phase 2 gate: print the merged IMU, camera and degraded GNSS streams and check their timing."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

import yaml

from gnss_denied.core.streams import merge_streams
from gnss_denied.core.types import CameraFrame, GnssMeasurement, ImuMeasurement
from gnss_denied.datasets.kitti_raw import KittiRawDrive
from gnss_denied.simulator.gnss_degrader import GnssDegrader, load_degrader_config


def describe(packet) -> str:
    if isinstance(packet, ImuMeasurement):
        ax, ay, az = packet.accel_mps2
        return f"IMU     accel=({ax:+7.3f}, {ay:+7.3f}, {az:+7.3f})"
    if isinstance(packet, CameraFrame):
        return f"CAMERA  id={packet.camera_id} {packet.image_path.name}"
    if isinstance(packet, GnssMeasurement):
        position = f"lat={packet.lat_deg:.7f} lon={packet.lon_deg:.7f} alt={packet.alt_m:.2f}"
        return f"GNSS    {packet.status.name:8} {position if packet.has_fix else 'no fix'}"
    raise TypeError(f"unknown packet: {packet!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("configs/default_sim.yaml"))
    parser.add_argument("--head", type=int, default=25, help="packets to print at the start")
    args = parser.parse_args()

    dataset = yaml.safe_load(args.config.read_text(encoding="utf-8"))["dataset"]
    drive = KittiRawDrive(dataset["root"], dataset["date"], dataset["drive"])
    degrader = GnssDegrader(load_degrader_config(args.config))
    merged = merge_streams(drive.imu(), drive.frames(), degrader.degrade(drive.ground_truth()))

    counts: Counter[str] = Counter()
    first_seen: dict[str, float] = {}
    last_seen: dict[str, float] = {}
    gnss_status: Counter[str] = Counter()
    largest_gap = {"IMU": 0.0, "CAMERA": 0.0, "GNSS": 0.0}
    previous_status = None
    previous_timestamp = float("-inf")
    out_of_order = 0

    for index, packet in enumerate(merged):
        kind = describe(packet).split()[0]
        if index < args.head:
            print(f"{packet.timestamp:10.4f}  {describe(packet)}")
        if packet.timestamp < previous_timestamp:
            out_of_order += 1
        previous_timestamp = packet.timestamp
        if kind in last_seen:
            largest_gap[kind] = max(largest_gap[kind], packet.timestamp - last_seen[kind])
        first_seen.setdefault(kind, packet.timestamp)
        last_seen[kind] = packet.timestamp
        counts[kind] += 1
        if isinstance(packet, GnssMeasurement):
            gnss_status[packet.status.name] += 1
            if previous_status is not None and packet.status is not previous_status:
                print(f"{packet.timestamp:10.4f}  {describe(packet)}   <-- status change")
            previous_status = packet.status

    print("\nstream    packets  rate_hz  max_gap_s")
    for kind in ("IMU", "CAMERA", "GNSS"):
        span = last_seen[kind] - first_seen[kind]
        rate = (counts[kind] - 1) / span
        print(f"{kind:8} {counts[kind]:8d} {rate:8.2f} {largest_gap[kind]:10.3f}")
    print(f"\nGNSS status: {dict(gnss_status)}")
    print(f"dropped OXTS samples: {drive.dropped_oxts_samples}")
    print(f"out-of-order packets: {out_of_order}")
    return 1 if out_of_order else 0


if __name__ == "__main__":
    raise SystemExit(main())
