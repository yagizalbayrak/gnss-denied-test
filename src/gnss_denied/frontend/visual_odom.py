"""Stereo visual odometry: metric frame-to-frame motion from any feature extractor and matcher.

Each stereo pair gives 3D points through its disparity. Matching the previous left image to the
current one then turns those points into 3D-2D correspondences, and PnP with MAGSAC recovers the
motion. A monocular Essential Matrix would only give the direction of translation, and the graph
needs metres to bound drift when GNSS is gone.

Conventions: T_a_b maps coordinates in frame b to frame a. Cameras are rectified, x right, y down,
z forward. Constraints are reported in the body frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import cv2
import gtsam
import numpy as np
from numpy.typing import NDArray

from gnss_denied.core.types import FloatArray, VisualOdometryConstraint


@dataclass(frozen=True)
class StereoCalibration:
    """Rectified stereo intrinsics and the pose of the left camera in the body frame."""

    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float
    T_body_camera: FloatArray

    @property
    def intrinsics(self) -> FloatArray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass(frozen=True)
class VoNoise:
    """Per-frame 1-sigma error assumed for a constraint, on each axis of the body frame."""

    rotation_sigma_rad: float = 2e-3
    translation_sigma_m: float = 0.05

    def covariance(self) -> FloatArray:
        sigmas = np.r_[np.full(3, self.rotation_sigma_rad), np.full(3, self.translation_sigma_m)]
        return np.diag(sigmas**2)


@dataclass(frozen=True)
class VoParams:
    min_disparity_px: float = 3.0
    max_epipolar_error_px: float = 1.5
    reprojection_error_px: float = 1.5
    min_inliers: int = 20
    ransac_iterations: int = 500
    ransac_confidence: float = 0.999


@dataclass(frozen=True)
class FrameOdometry:
    """Outcome for one stereo frame. `constraint` is None for the first frame or a tracking loss."""

    timestamp: float
    constraint: VisualOdometryConstraint | None


@dataclass(frozen=True)
class Features:
    keypoints: NDArray[np.float32]  # N x 2, pixels
    descriptors: np.ndarray


class FeatureBackend(Protocol):
    """A feature extractor and matcher, so that classic and learned front-ends are interchangeable.

    `match` returns an M x 2 array of index pairs into (first, second).
    """

    name: str

    def extract(self, image: np.ndarray) -> Features: ...

    def match(self, first: Features, second: Features) -> NDArray[np.int64]: ...


class OrbBackend:
    """ORB with a ratio test and a mutual check: the classic baseline for the learned front-end."""

    name = "orb"

    def __init__(self, num_features: int = 3000, ratio: float = 0.8) -> None:
        self._orb = cv2.ORB_create(nfeatures=num_features, fastThreshold=10)
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self._ratio = ratio

    def extract(self, image: np.ndarray) -> Features:
        keypoints, descriptors = self._orb.detectAndCompute(image, None)
        points = np.array([kp.pt for kp in keypoints], dtype=np.float32).reshape(-1, 2)
        if descriptors is None:
            descriptors = np.empty((0, 32), dtype=np.uint8)
        return Features(points, descriptors)

    def match(self, first: Features, second: Features) -> NDArray[np.int64]:
        if len(first.keypoints) < 2 or len(second.keypoints) < 2:
            return np.empty((0, 2), dtype=np.int64)
        forward = self._ratio_matches(first.descriptors, second.descriptors)
        backward = self._ratio_matches(second.descriptors, first.descriptors)
        mutual = [(i, j) for i, j in forward.items() if backward.get(j) == i]
        return np.array(mutual, dtype=np.int64).reshape(-1, 2)

    def _ratio_matches(self, query: np.ndarray, train: np.ndarray) -> dict[int, int]:
        matches = {}
        for pair in self._matcher.knnMatch(query, train, k=2):
            if len(pair) == 2 and pair[0].distance < self._ratio * pair[1].distance:
                matches[pair[0].queryIdx] = pair[0].trainIdx
        return matches


@dataclass(frozen=True)
class _StereoFrame:
    timestamp: float
    features: Features
    points_camera_m: FloatArray  # 3D point per keypoint, NaN where stereo matching failed


class StereoVisualOdometry:
    """Frame-to-frame odometry. After a tracking loss, it restarts from the current frame."""

    def __init__(
        self,
        backend: FeatureBackend,
        calibration: StereoCalibration,
        params: VoParams | None = None,
        noise: VoNoise | None = None,
    ) -> None:
        self._backend = backend
        self._calibration = calibration
        self._params = params or VoParams()
        self._noise = noise or VoNoise()
        self._previous: _StereoFrame | None = None

    def process(self, timestamp: float, left: np.ndarray, right: np.ndarray) -> FrameOdometry:
        frame = self._triangulate(timestamp, left, right)
        constraint = None if self._previous is None else self._estimate(self._previous, frame)
        self._previous = frame
        return FrameOdometry(timestamp, constraint)

    def _triangulate(self, timestamp: float, left: np.ndarray, right: np.ndarray) -> _StereoFrame:
        features = self._backend.extract(left)
        right_features = self._backend.extract(right)
        matches = self._backend.match(features, right_features)
        points = np.full((len(features.keypoints), 3), np.nan)
        if len(matches) > 0:
            left_px = features.keypoints[matches[:, 0]]
            right_px = right_features.keypoints[matches[:, 1]]
            disparity = left_px[:, 0] - right_px[:, 0]
            on_epipolar = (
                np.abs(left_px[:, 1] - right_px[:, 1]) <= self._params.max_epipolar_error_px
            )
            valid = on_epipolar & (disparity >= self._params.min_disparity_px)
            calibration = self._calibration
            depth = calibration.fx * calibration.baseline_m / disparity[valid]
            points[matches[valid, 0]] = np.column_stack(
                [
                    (left_px[valid, 0] - calibration.cx) * depth / calibration.fx,
                    (left_px[valid, 1] - calibration.cy) * depth / calibration.fy,
                    depth,
                ]
            )
        return _StereoFrame(timestamp, features, points)

    def _estimate(
        self, previous: _StereoFrame, current: _StereoFrame
    ) -> VisualOdometryConstraint | None:
        params = self._params
        matches = self._backend.match(previous.features, current.features)
        if len(matches) == 0:
            return None
        object_points = previous.points_camera_m[matches[:, 0]]
        image_points = current.features.keypoints[matches[:, 1]]
        has_depth = np.isfinite(object_points[:, 2])
        if has_depth.sum() < params.min_inliers:
            return None

        object_points = object_points[has_depth].astype(np.float64)
        image_points = image_points[has_depth].astype(np.float64)
        intrinsics = self._calibration.intrinsics
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            intrinsics,
            None,
            iterationsCount=params.ransac_iterations,
            reprojectionError=params.reprojection_error_px,
            confidence=params.ransac_confidence,
            flags=cv2.USAC_MAGSAC,
        )
        if not success or inliers is None or len(inliers) < params.min_inliers:
            return None
        inliers = inliers.ravel()
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points[inliers], image_points[inliers], intrinsics, None, rvec, tvec
        )

        T_current_previous = np.eye(4)
        T_current_previous[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_current_previous[:3, 3] = tvec.ravel()
        T_body = self._calibration.T_body_camera
        T_previous_current = np.linalg.inv(T_current_previous)
        motion_body = T_body @ T_previous_current @ np.linalg.inv(T_body)
        return make_constraint(
            previous.timestamp,
            current.timestamp,
            motion_body[:3, :3],
            motion_body[:3, 3],
            len(inliers),
            self._noise,
        )


def make_constraint(
    timestamp_from: float,
    timestamp_to: float,
    rotation: FloatArray,
    translation_m: FloatArray,
    num_inliers: int,
    noise: VoNoise,
) -> VisualOdometryConstraint:
    quaternion = gtsam.Rot3(rotation).toQuaternion()
    wxyz = np.array([quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()])
    return VisualOdometryConstraint(
        timestamp_from=timestamp_from,
        timestamp_to=timestamp_to,
        rotation_wxyz=wxyz / np.linalg.norm(wxyz),
        translation_m=translation_m,
        covariance_6x6=noise.covariance(),
        scale_known=True,
        num_inliers=num_inliers,
    )


def save_odometry(path: str | Path, records: list[FrameOdometry]) -> None:
    """Cache the measured motion. Noise is not stored, so it can be re-tuned without re-running."""
    timestamps = np.array([record.timestamp for record in records])
    valid = np.array([record.constraint is not None for record in records])
    rotations = np.full((len(records), 4), np.nan)
    translations = np.full((len(records), 3), np.nan)
    previous = np.full(len(records), np.nan)
    inliers = np.zeros(len(records), dtype=np.int64)
    for index, record in enumerate(records):
        if record.constraint is not None:
            rotations[index] = record.constraint.rotation_wxyz
            translations[index] = record.constraint.translation_m
            previous[index] = record.constraint.timestamp_from
            inliers[index] = record.constraint.num_inliers
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        path,
        timestamps=timestamps,
        valid=valid,
        rotations=rotations,
        translations=translations,
        previous=previous,
        inliers=inliers,
    )


def load_odometry(path: str | Path, noise: VoNoise) -> list[FrameOdometry]:
    with np.load(path, allow_pickle=False) as data:
        records = []
        for index, timestamp in enumerate(data["timestamps"]):
            constraint = None
            if data["valid"][index]:
                constraint = VisualOdometryConstraint(
                    timestamp_from=float(data["previous"][index]),
                    timestamp_to=float(timestamp),
                    rotation_wxyz=data["rotations"][index],
                    translation_m=data["translations"][index],
                    covariance_6x6=noise.covariance(),
                    scale_known=True,
                    num_inliers=int(data["inliers"][index]),
                )
            records.append(FrameOdometry(float(timestamp), constraint))
    return records
