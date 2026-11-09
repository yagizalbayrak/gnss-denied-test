"""Immutable measurement and state types shared across the pipeline.

Conventions:
    * Timestamps are seconds in a single, monotonic dataset clock.
    * Local frame is ENU (x = East, y = North, z = Up), in metres.
    * Quaternions are (w, x, y, z), unit norm, and rotate body vectors into the local frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


def _frozen_array(value: object, shape: tuple[int, ...], name: str) -> FloatArray:
    """Copy `value` into a read-only float64 array and enforce its shape."""
    array = np.array(value, dtype=np.float64)
    if array.shape != shape:
        raise ValueError(f"{name} must have shape {shape}, got {array.shape}")
    array.setflags(write=False)
    return array


def _frozen_covariance(value: object, name: str) -> FloatArray:
    """Like `_frozen_array`, and also reject a non-symmetric 3x3 covariance."""
    array = _frozen_array(value, (3, 3), name)
    if not np.allclose(array, array.T):
        raise ValueError(f"{name} must be symmetric")
    return array


class GnssStatus(Enum):
    """Validity of a GNSS sample, as reported by the receiver or the degrader."""

    VALID = "valid"
    DEGRADED = "degraded"
    DENIED = "denied"


@dataclass(frozen=True)
class ImuMeasurement:
    """One IMU sample in the body frame."""

    timestamp: float
    accel_mps2: FloatArray
    gyro_radps: FloatArray

    def __post_init__(self) -> None:
        object.__setattr__(self, "accel_mps2", _frozen_array(self.accel_mps2, (3,), "accel_mps2"))
        object.__setattr__(self, "gyro_radps", _frozen_array(self.gyro_radps, (3,), "gyro_radps"))


@dataclass(frozen=True)
class GnssMeasurement:
    """One geodetic GNSS sample.

    `covariance_enu_m2` is the position covariance in the local ENU axes at the fix. Latitude,
    longitude and altitude are NaN when `status` is DENIED, because a denied epoch has no fix.
    """

    timestamp: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    covariance_enu_m2: FloatArray
    status: GnssStatus = GnssStatus.VALID

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "covariance_enu_m2",
            _frozen_covariance(self.covariance_enu_m2, "covariance_enu_m2"),
        )

    @property
    def has_fix(self) -> bool:
        return self.status is not GnssStatus.DENIED


@dataclass(frozen=True)
class GroundTruthSample:
    """Reference navigation state from the dataset, used to simulate GNSS and to score estimates.

    Attitude follows the OXTS convention: yaw is 0 facing East and positive counter-clockwise.
    """

    timestamp: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    velocity_enu_mps: FloatArray

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "velocity_enu_mps",
            _frozen_array(self.velocity_enu_mps, (3,), "velocity_enu_mps"),
        )


@dataclass(frozen=True)
class CameraFrame:
    """Reference to one image on disk; decoding is left to the consumer to keep streams light."""

    timestamp: float
    camera_id: int
    image_path: Path


@dataclass(frozen=True)
class VisualOdometryConstraint:
    """Relative motion between two camera frames, expressed in the frame at `timestamp_from`.

    Monocular front-ends recover translation only up to scale. They set `scale_known` to False,
    and `translation_m` is then a unit direction rather than a metric displacement.
    """

    timestamp_from: float
    timestamp_to: float
    rotation_wxyz: FloatArray
    translation_m: FloatArray
    covariance_6x6: FloatArray
    scale_known: bool = True
    num_inliers: int = 0

    def __post_init__(self) -> None:
        if self.timestamp_to <= self.timestamp_from:
            raise ValueError("timestamp_to must be later than timestamp_from")
        rotation = _frozen_array(self.rotation_wxyz, (4,), "rotation_wxyz")
        if not np.isclose(np.linalg.norm(rotation), 1.0):
            raise ValueError("rotation_wxyz must be a unit quaternion")
        object.__setattr__(self, "rotation_wxyz", rotation)
        object.__setattr__(
            self, "translation_m", _frozen_array(self.translation_m, (3,), "translation_m")
        )
        object.__setattr__(
            self, "covariance_6x6", _frozen_array(self.covariance_6x6, (6, 6), "covariance_6x6")
        )


@dataclass(frozen=True)
class StateOutput:
    """Estimated navigation state in the local ENU frame."""

    timestamp: float
    position_enu_m: FloatArray
    velocity_enu_mps: FloatArray
    orientation_wxyz: FloatArray
    position_covariance_m2: FloatArray

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position_enu_m", _frozen_array(self.position_enu_m, (3,), "position_enu_m")
        )
        object.__setattr__(
            self, "velocity_enu_mps", _frozen_array(self.velocity_enu_mps, (3,), "velocity_enu_mps")
        )
        orientation = _frozen_array(self.orientation_wxyz, (4,), "orientation_wxyz")
        if not np.isclose(np.linalg.norm(orientation), 1.0):
            raise ValueError("orientation_wxyz must be a unit quaternion")
        object.__setattr__(self, "orientation_wxyz", orientation)
        object.__setattr__(
            self,
            "position_covariance_m2",
            _frozen_covariance(self.position_covariance_m2, "position_covariance_m2"),
        )
