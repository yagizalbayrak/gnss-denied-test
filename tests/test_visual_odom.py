"""VO geometry on synthetic landmarks. A stub backend replaces images, so features are exact."""

from __future__ import annotations

import gtsam
import numpy as np
import pytest

from gnss_denied.frontend.visual_odom import (
    Features,
    FrameOdometry,
    OrbBackend,
    StereoCalibration,
    StereoVisualOdometry,
    VoNoise,
    load_odometry,
    make_constraint,
    save_odometry,
)

FX, CX, CY, BASELINE = 700.0, 600.0, 180.0, 0.54
# Body is x forward, y left, z up; the camera is x right, y down, z forward, 1 m ahead of the body.
R_BODY_CAMERA = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T_BODY_CAMERA = np.eye(4)
T_BODY_CAMERA[:3, :3] = R_BODY_CAMERA
T_BODY_CAMERA[:3, 3] = [1.0, 0.0, 0.5]
CALIBRATION = StereoCalibration(FX, FX, CX, CY, BASELINE, T_BODY_CAMERA)


def _landmarks(count: int = 300) -> np.ndarray:
    rng = np.random.default_rng(0)
    return np.column_stack(
        [rng.uniform(-15, 15, count), rng.uniform(-4, 4, count), rng.uniform(5, 40, count)]
    )


def _project(points_camera: np.ndarray, shift_x_m: float = 0.0) -> np.ndarray:
    u = FX * (points_camera[:, 0] - shift_x_m) / points_camera[:, 2] + CX
    v = FX * points_camera[:, 1] / points_camera[:, 2] + CY
    return np.column_stack([u, v]).astype(np.float32)


class IdBackend:
    """Images are (keypoints, ids) tuples, and a match means sharing a landmark id."""

    name = "stub"

    def extract(self, image) -> Features:
        return Features(image[0], image[1])

    def match(self, first: Features, second: Features) -> np.ndarray:
        positions = {landmark: i for i, landmark in enumerate(second.descriptors)}
        pairs = [
            (i, positions[landmark])
            for i, landmark in enumerate(first.descriptors)
            if landmark in positions
        ]
        return np.array(pairs, dtype=np.int64).reshape(-1, 2)


def _stereo_views(points_camera: np.ndarray, ids: np.ndarray):
    return (_project(points_camera), ids), (_project(points_camera, BASELINE), ids)


def _moved(points: np.ndarray, rotation_previous_current: np.ndarray, translation: np.ndarray):
    """Express points in a camera that moved by (R, t) relative to the first camera."""
    return (points - translation) @ rotation_previous_current


def test_recovers_camera_motion_in_the_body_frame():
    points = _landmarks()
    ids = np.arange(len(points))
    rotation = gtsam.Rot3.Ry(0.02).matrix()  # A small rotation about the camera's vertical axis.
    translation_camera = np.array([0.1, -0.02, 0.8])

    vo = StereoVisualOdometry(IdBackend(), CALIBRATION)
    assert vo.process(0.0, *_stereo_views(points, ids)).constraint is None
    moved = _moved(points, rotation, translation_camera)
    constraint = vo.process(0.1, *_stereo_views(moved, ids)).constraint

    T_previous_current = np.eye(4)
    T_previous_current[:3, :3] = rotation
    T_previous_current[:3, 3] = translation_camera
    expected = T_BODY_CAMERA @ T_previous_current @ np.linalg.inv(T_BODY_CAMERA)
    w, x, y, z = constraint.rotation_wxyz
    np.testing.assert_allclose(
        gtsam.Rot3.Quaternion(w, x, y, z).matrix(), expected[:3, :3], atol=1e-3
    )
    np.testing.assert_allclose(constraint.translation_m, expected[:3, 3], atol=5e-3)
    assert constraint.scale_known
    assert constraint.timestamp_from == 0.0 and constraint.timestamp_to == 0.1
    assert constraint.num_inliers > 250


def test_forward_camera_motion_reads_as_forward_body_motion():
    points = _landmarks()
    ids = np.arange(len(points))
    vo = StereoVisualOdometry(IdBackend(), CALIBRATION)
    vo.process(0.0, *_stereo_views(points, ids))
    moved = _moved(points, np.eye(3), np.array([0.0, 0.0, 1.0]))
    constraint = vo.process(0.1, *_stereo_views(moved, ids)).constraint
    np.testing.assert_allclose(constraint.translation_m, [1.0, 0.0, 0.0], atol=5e-3)


def test_wrong_temporal_matches_are_rejected_by_ransac():
    points = _landmarks()
    ids = np.arange(len(points))
    vo = StereoVisualOdometry(IdBackend(), CALIBRATION)
    vo.process(0.0, *_stereo_views(points, ids))

    translation = np.array([0.0, 0.0, 0.8])
    moved = _moved(points, np.eye(3), translation)
    left, right = _stereo_views(moved, ids)
    corrupted = left[0].copy()
    rng = np.random.default_rng(1)
    bad = rng.choice(len(points), size=60, replace=False)
    corrupted[bad] += rng.uniform(30, 80, (60, 2)).astype(np.float32)
    constraint = vo.process(0.1, (corrupted, ids), right).constraint

    np.testing.assert_allclose(constraint.translation_m, [0.8, 0.0, 0.0], atol=0.01)
    assert constraint.num_inliers < 250


def test_too_few_matches_is_a_tracking_loss_and_tracking_restarts():
    points = _landmarks()
    ids = np.arange(len(points))
    vo = StereoVisualOdometry(IdBackend(), CALIBRATION)
    vo.process(0.0, *_stereo_views(points, ids))

    other_ids = ids + 10_000
    assert vo.process(0.1, *_stereo_views(points, other_ids)).constraint is None
    restarted = vo.process(0.2, *_stereo_views(points, other_ids)).constraint
    assert restarted is not None
    assert restarted.timestamp_from == 0.1


def test_points_without_valid_disparity_are_not_used():
    points = _landmarks()
    ids = np.arange(len(points))
    left = (_project(points), ids)
    right = (_project(points), ids)  # Zero disparity everywhere: no point can be triangulated.
    vo = StereoVisualOdometry(IdBackend(), CALIBRATION)
    vo.process(0.0, left, right)
    assert vo.process(0.1, left, right).constraint is None


def test_orb_backend_tracks_a_textured_image_shift():
    rng = np.random.default_rng(0)
    base = (rng.random((370, 1226)) * 255).astype(np.uint8)
    import cv2

    image = cv2.GaussianBlur(base, (5, 5), 0)
    shifted = np.roll(image, 5, axis=1)
    backend = OrbBackend()
    first, second = backend.extract(image), backend.extract(shifted)
    matches = backend.match(first, second)
    assert len(matches) > 50
    displacement = second.keypoints[matches[:, 1]] - first.keypoints[matches[:, 0]]
    assert np.median(displacement[:, 0]) == pytest.approx(5.0, abs=1.0)
    assert np.median(displacement[:, 1]) == pytest.approx(0.0, abs=1.0)


def test_orb_backend_handles_a_blank_image():
    backend = OrbBackend()
    blank = backend.extract(np.zeros((100, 100), dtype=np.uint8))
    assert len(backend.match(blank, blank)) == 0


def test_cache_round_trip_keeps_motion_and_applies_new_noise(tmp_path):
    constraint = make_constraint(0.0, 0.1, np.eye(3), np.array([1.0, 2.0, 3.0]), 42, VoNoise())
    path = tmp_path / "vo.npz"
    save_odometry(path, [FrameOdometry(0.0, None), FrameOdometry(0.1, constraint)])
    noise = VoNoise(rotation_sigma_rad=0.1, translation_sigma_m=1.0)
    first, second = load_odometry(path, noise)
    assert first.constraint is None and first.timestamp == 0.0
    assert second.constraint.timestamp_from == 0.0
    assert second.constraint.num_inliers == 42
    np.testing.assert_allclose(second.constraint.translation_m, [1.0, 2.0, 3.0])
    assert second.constraint.covariance_6x6[0, 0] == pytest.approx(0.01)
    assert second.constraint.covariance_6x6[5, 5] == pytest.approx(1.0)
