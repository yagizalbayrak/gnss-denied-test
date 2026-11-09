"""Loader for KITTI Raw: 100 Hz OXTS from the `extract` set, 10 Hz images from the `sync` set.

`pykitti` is not used here because it assumes a single dataset flavour per drive, while the
100 Hz inertial stream and the rectified images live in different archives.

All timestamps are seconds relative to the earliest sample of the drive, so that every stream
shares one clock and degradation windows can be written as offsets from the start of the drive.
"""

from __future__ import annotations

from bisect import bisect_left
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from gnss_denied.core.types import CameraFrame, GroundTruthSample, ImuMeasurement
from gnss_denied.frontend.visual_odom import StereoCalibration

_OXTS_FIELD_COUNT = 30


def _read_calibration(path: Path) -> dict[str, np.ndarray]:
    """Parse `name: numbers` lines, skipping non-numeric ones such as `calib_time`."""
    entries = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        name, _, values = line.partition(":")
        try:
            entries[name] = np.array([float(value) for value in values.split()])
        except ValueError:
            continue
    return entries


def _rigid_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation.reshape(3, 3)
    transform[:3, 3] = translation
    return transform


def load_stereo_calibration(root: str | Path, date: str) -> StereoCalibration:
    """Read the rectified grayscale pair, with the body taken to be the OXTS IMU."""
    date_dir = Path(root) / date
    cameras = _read_calibration(date_dir / "calib_cam_to_cam.txt")
    velo_to_camera = _read_calibration(date_dir / "calib_velo_to_cam.txt")
    imu_to_velo = _read_calibration(date_dir / "calib_imu_to_velo.txt")

    T_rectified_camera = np.eye(4)
    T_rectified_camera[:3, :3] = cameras["R_rect_00"].reshape(3, 3)
    T_camera_velo = _rigid_transform(velo_to_camera["R"], velo_to_camera["T"])
    T_velo_imu = _rigid_transform(imu_to_velo["R"], imu_to_velo["T"])
    T_rectified_imu = T_rectified_camera @ T_camera_velo @ T_velo_imu

    left, right = cameras["P_rect_00"].reshape(3, 4), cameras["P_rect_01"].reshape(3, 4)
    return StereoCalibration(
        fx=float(left[0, 0]),
        fy=float(left[1, 1]),
        cx=float(left[0, 2]),
        cy=float(left[1, 2]),
        baseline_m=float((left[0, 3] - right[0, 3]) / left[0, 0]),
        T_body_camera=np.linalg.inv(T_rectified_imu),
    )


def _parse_timestamp_ns(line: str) -> int | None:
    text = line.strip()
    if not text:
        return None
    return int(np.datetime64(text, "ns").astype(np.int64))


def _read_timestamps_ns(path: Path) -> list[int | None]:
    return [_parse_timestamp_ns(line) for line in path.read_text(encoding="utf-8").splitlines()]


def _longest_increasing_indices(values: list[int]) -> list[int]:
    """Return the indices of a longest strictly increasing subsequence, in O(n log n)."""
    tail_values: list[int] = []
    tail_indices: list[int] = []
    predecessors: list[int | None] = []
    for index, value in enumerate(values):
        position = bisect_left(tail_values, value)
        predecessors.append(tail_indices[position - 1] if position > 0 else None)
        if position == len(tail_values):
            tail_values.append(value)
            tail_indices.append(index)
        else:
            tail_values[position] = value
            tail_indices[position] = index

    indices: list[int] = []
    current = tail_indices[-1] if tail_indices else None
    while current is not None:
        indices.append(current)
        current = predecessors[current]
    return indices[::-1]


@dataclass(frozen=True)
class OxtsSample:
    timestamp_ns: int
    values: tuple[float, ...]


class KittiRawDrive:
    """One drive on disk, laid out as `<root>/<date>/<date>_drive_<drive>_{extract,sync}/`."""

    def __init__(self, root: str | Path, date: str, drive: str) -> None:
        self._date_dir = Path(root) / date
        self._extract_dir = self._date_dir / f"{date}_drive_{drive}_extract"
        self._sync_dir = self._date_dir / f"{date}_drive_{drive}_sync"
        self.dropped_oxts_samples = 0
        self._oxts = self._load_oxts()
        if not self._oxts:
            raise ValueError(f"no usable OXTS samples in {self._extract_dir}")
        first_frame_ns = self._first_frame_timestamp_ns()
        first_oxts_ns = self._oxts[0].timestamp_ns
        self._t0_ns = (
            first_oxts_ns if first_frame_ns is None else min(first_oxts_ns, first_frame_ns)
        )

    def _load_oxts(self) -> list[OxtsSample]:
        """Read OXTS packets, dropping blank, truncated or mis-stamped ones.

        The unsynced recordings do contain such packets, and a backwards timestamp would produce
        a negative integration interval downstream. Comparing each stamp with the previous one is
        not enough: drive 0018 has bursts stamped 60 ms late, and that test would keep the bad
        burst and drop the good packets after it. Keeping the longest increasing run of stamps
        removes the fewest packets and handles forward and backward jumps alike.
        """
        oxts_dir = self._extract_dir / "oxts"
        if not oxts_dir.exists():
            raise FileNotFoundError(
                f"{oxts_dir} not found. Fetch the data with: python data/download_sample.py"
            )
        candidates: list[OxtsSample] = []
        timestamps = _read_timestamps_ns(oxts_dir / "timestamps.txt")
        for index, timestamp_ns in enumerate(timestamps):
            path = oxts_dir / "data" / f"{index:010d}.txt"
            usable = timestamp_ns is not None and path.exists()
            values = path.read_text(encoding="utf-8").split() if usable else []
            if len(values) == _OXTS_FIELD_COUNT:
                candidates.append(OxtsSample(timestamp_ns, tuple(float(value) for value in values)))
        kept = _longest_increasing_indices([sample.timestamp_ns for sample in candidates])
        self.dropped_oxts_samples = len(timestamps) - len(kept)
        return [candidates[index] for index in kept]

    def _first_frame_timestamp_ns(self) -> int | None:
        path = self._sync_dir / "image_00" / "timestamps.txt"
        if not path.exists():
            return None
        return next((stamp for stamp in _read_timestamps_ns(path) if stamp is not None), None)

    def _seconds(self, timestamp_ns: int) -> float:
        return (timestamp_ns - self._t0_ns) * 1e-9

    def imu(self) -> Iterator[ImuMeasurement]:
        """Accelerometer and gyro in the OXTS body frame (x forward, y left, z up)."""
        for sample in self._oxts:
            values = sample.values
            yield ImuMeasurement(self._seconds(sample.timestamp_ns), values[11:14], values[17:20])

    def ground_truth(self) -> Iterator[GroundTruthSample]:
        for sample in self._oxts:
            values = sample.values
            velocity_enu = (values[7], values[6], values[10])  # Stored as vn, ve, vf, vl, vu.
            yield GroundTruthSample(
                timestamp=self._seconds(sample.timestamp_ns),
                lat_deg=values[0],
                lon_deg=values[1],
                alt_m=values[2],
                roll_rad=values[3],
                pitch_rad=values[4],
                yaw_rad=values[5],
                velocity_enu_mps=velocity_enu,
            )

    def frames(self, camera_id: int = 0) -> Iterator[CameraFrame]:
        camera_dir = self._sync_dir / f"image_{camera_id:02d}"
        for index, timestamp_ns in enumerate(_read_timestamps_ns(camera_dir / "timestamps.txt")):
            path = camera_dir / "data" / f"{index:010d}.png"
            if timestamp_ns is not None and path.exists():
                yield CameraFrame(self._seconds(timestamp_ns), camera_id, path)

    def stereo_pairs(self) -> Iterator[tuple[CameraFrame, Path]]:
        """Yield each left frame with the path of the right image that was taken with it."""
        right_dir = self._sync_dir / "image_01" / "data"
        for frame in self.frames(0):
            right_path = right_dir / frame.image_path.name
            if right_path.exists():
                yield frame, right_path
