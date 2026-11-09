"""Back-end tests on a synthetic circular drive, where the exact answer is known."""

from __future__ import annotations

import gtsam
import numpy as np
import pytest

from gnss_denied.backend.factor_graph import (
    DeadReckoner,
    FactorGraphEstimator,
    ImuParams,
    InitialState,
)
from gnss_denied.core.geodesy import LocalEnuFrame
from gnss_denied.core.types import (
    GnssMeasurement,
    GnssStatus,
    GroundTruthSample,
    ImuMeasurement,
    VisualOdometryConstraint,
)
from gnss_denied.frontend.visual_odom import FrameOdometry, VoNoise, make_constraint
from gnss_denied.pipeline import run_fusion, window_errors
from gnss_denied.simulator.gnss_degrader import (
    DegradationMode,
    DegradationWindow,
    GnssDegrader,
    GnssDegraderConfig,
)

GRAVITY = 9.809
SPEED = 8.0
YAW_RATE = 0.1
RADIUS = SPEED / YAW_RATE
ORIGIN = (49.0, 8.4, 110.0)
FRAME = LocalEnuFrame(*ORIGIN)
PARAMS = ImuParams(gravity_mps2=GRAVITY)


def _position(t: float) -> np.ndarray:
    theta = YAW_RATE * t
    return np.array([RADIUS * np.sin(theta), RADIUS * (1.0 - np.cos(theta)), 0.0])


def _imu(
    duration_s: float,
    accel_bias=(0.0, 0.0, 0.0),
    gyro_bias=(0.0, 0.0, 0.0),
    skip: tuple[float, float] | None = None,
) -> list[ImuMeasurement]:
    """Specific force is the centripetal term on y (left) plus gravity reaction on z (up)."""
    accel = np.array([0.0, SPEED * YAW_RATE, GRAVITY]) + np.array(accel_bias)
    gyro = np.array([0.0, 0.0, YAW_RATE]) + np.array(gyro_bias)
    stamps = np.arange(int(duration_s * 100) + 1) / 100.0
    if skip is not None:
        stamps = stamps[(stamps <= skip[0]) | (stamps >= skip[1])]
    return [ImuMeasurement(float(t), accel, gyro) for t in stamps]


def _truth(duration_s: float) -> list[GroundTruthSample]:
    samples = []
    for t in np.arange(int(duration_s * 100) + 1) / 100.0:
        lat, lon, alt = FRAME.to_geodetic(_position(float(t)))
        yaw = YAW_RATE * t
        velocity = [SPEED * np.cos(yaw), SPEED * np.sin(yaw), 0.0]
        samples.append(GroundTruthSample(float(t), lat, lon, alt, 0.0, 0.0, yaw, velocity))
    return samples


def _run(duration_s: float, *windows: DegradationWindow, **imu_kwargs):
    truth = _truth(duration_s)
    gnss = list(
        GnssDegrader(
            GnssDegraderConfig(windows=windows, sigma_horizontal_m=0.1, sigma_vertical_m=0.1)
        ).degrade(truth)
    )
    return run_fusion(_imu(duration_s, **imu_kwargs), truth, gnss, PARAMS)


def _initial() -> InitialState:
    return InitialState(0.0, np.zeros(3), np.array([SPEED, 0.0, 0.0]), 0.0, 0.0, 0.0)


def _fix(t: float, sigma_m: float = 0.1) -> GnssMeasurement:
    lat, lon, alt = FRAME.to_geodetic(_position(t))
    return GnssMeasurement(t, lat, lon, alt, np.eye(3) * sigma_m**2)


def test_dead_reckoning_with_perfect_imu_follows_the_circle():
    reckoner = DeadReckoner(PARAMS, _initial())
    for measurement in _imu(60.0):
        reckoner.add_imu(measurement)
    state = reckoner.state()
    assert state.timestamp == pytest.approx(60.0)
    # Each step uses the attitude at its start, a velocity error of v * yaw_rate * dt / 2 that
    # accumulates to about 0.24 m here. Anything well above that would be a real bug.
    assert np.linalg.norm(state.position_enu_m - _position(60.0)) < 0.3


def test_dead_reckoning_error_grows_quadratically_with_accel_bias():
    bias = 0.02
    errors = {}
    for duration in (20.0, 40.0):
        reckoner = DeadReckoner(PARAMS, _initial())
        for measurement in _imu(duration, accel_bias=(0.0, 0.0, bias)):
            reckoner.add_imu(measurement)
        errors[duration] = np.linalg.norm(reckoner.state().position_enu_m - _position(duration))
    assert errors[20.0] == pytest.approx(0.5 * bias * 20.0**2, rel=0.05)
    assert errors[40.0] / errors[20.0] == pytest.approx(4.0, rel=0.05)


def test_fused_estimate_tracks_truth_with_valid_gnss():
    result = _run(60.0)
    scores = window_errors("fused", (s.fused for s in result.steps), result.truth_enu_by_time, 5.0)
    assert scores.position_rms_m < 0.2


def test_blackout_drifts_and_recovers_while_dead_reckoning_does_not():
    blackout = DegradationWindow(DegradationMode.BLACKOUT, 30.0, 60.0)
    accel_bias, gyro_bias = (0.03, -0.02, 0.02), (0.0, 0.0, 5e-4)
    result = _run(90.0, blackout, accel_bias=accel_bias, gyro_bias=gyro_bias)
    truth = result.truth_enu_by_time
    fused = [s.fused for s in result.steps]
    reckoned = [s.dead_reckoning for s in result.steps]

    before = window_errors("before", fused, truth, 20.0, 29.0)
    during = window_errors("during", fused, truth, 30.0, 60.0)
    after = window_errors("after", fused, truth, 65.0, 90.0)
    open_loop = window_errors("open loop", reckoned, truth, 65.0, 90.0)

    assert during.horizontal_max_m > 2.0 * before.horizontal_max_m
    assert after.horizontal_rms_m < 0.5 * during.horizontal_max_m
    assert open_loop.horizontal_rms_m > 10.0 * after.horizontal_rms_m


def test_estimated_bias_shrinks_blackout_drift_compared_with_open_loop():
    blackout = DegradationWindow(DegradationMode.BLACKOUT, 40.0, 70.0)
    result = _run(70.0, blackout, accel_bias=(0.03, -0.02, 0.02))
    truth = result.truth_enu_by_time
    drift = window_errors("fused", (s.fused for s in result.steps), truth, 40.0, 70.0)
    assert drift.horizontal_end_m < 0.5 * 0.5 * np.hypot(0.03, 0.02) * 30.0**2


def test_constant_bias_is_recovered_from_a_turning_trajectory():
    result = _run(120.0, accel_bias=(0.0, 0.0, 0.05), gyro_bias=(0.0, 0.0, 1e-3))
    assert result.accel_bias_mps2[2] == pytest.approx(0.05, abs=0.01)
    assert result.gyro_bias_radps[2] == pytest.approx(1e-3, abs=5e-4)


def test_position_uncertainty_grows_in_blackout_and_collapses_after():
    blackout = DegradationWindow(DegradationMode.BLACKOUT, 30.0, 60.0)
    steps = _run(70.0, blackout).steps
    by_time = {round(step.timestamp): step for step in steps}
    trace = {t: np.trace(by_time[t].fused.position_covariance_m2) for t in (29, 59, 65)}
    assert trace[59] > 5.0 * trace[29]
    assert trace[65] < 0.5 * trace[59]


def test_denied_epoch_adds_a_keyframe_without_changing_the_prediction():
    estimator = FactorGraphEstimator(PARAMS, FRAME, _initial())
    for measurement in _imu(1.0):
        estimator.add_imu(measurement)
    nan = float("nan")
    denied = GnssMeasurement(1.0, nan, nan, nan, np.diag([np.inf] * 3), GnssStatus.DENIED)
    state = estimator.add_gnss(denied)
    assert estimator.num_keyframes == 2
    assert state.timestamp == pytest.approx(1.0)
    assert np.linalg.norm(state.position_enu_m - _position(1.0)) < 0.01


def test_fix_at_same_time_as_the_keyframe_does_not_create_an_empty_one():
    estimator = FactorGraphEstimator(PARAMS, FRAME, _initial())
    estimator.add_gnss(_fix(0.0))
    assert estimator.num_keyframes == 1


def test_imu_gap_is_bridged_with_extra_keyframes_and_extra_uncertainty():
    # A tight position prior, so that the initial uncertainty does not mask the gap's contribution.
    initial = InitialState(
        0.0, np.zeros(3), np.array([SPEED, 0.0, 0.0]), 0.0, 0.0, 0.0, position_sigma_m=0.1
    )

    def run(skip):
        estimator = FactorGraphEstimator(PARAMS, FRAME, initial)
        states = []
        imu = iter(_imu(6.0, skip=skip))
        pending = next(imu)
        for t in range(1, 7):
            while pending is not None and pending.timestamp <= t:
                estimator.add_imu(pending)
                pending = next(imu, None)
            nan = float("nan")
            states.append(
                estimator.add_gnss(
                    GnssMeasurement(
                        float(t), nan, nan, nan, np.diag([np.inf] * 3), GnssStatus.DENIED
                    )
                )
            )
        return estimator, states

    plain_estimator, plain = run(None)
    gap_estimator, gapped = run((2.2, 3.8))
    assert gap_estimator.num_keyframes > plain_estimator.num_keyframes
    assert np.trace(gapped[-1].position_covariance_m2) > 2.0 * np.trace(
        plain[-1].position_covariance_m2
    )
    # Motion across the hole is still predicted well, since the first-order hold is exact here.
    assert np.linalg.norm(gapped[-1].position_enu_m - _position(6.0)) < 1.0


def test_fix_during_imu_gap_is_rejected_instead_of_corrupting_the_estimate():
    estimator = FactorGraphEstimator(PARAMS, FRAME, _initial())
    imu = iter(_imu(6.0, skip=(2.2, 3.8)))
    pending = next(imu)
    state = None
    for t in range(1, 7):
        while pending is not None and pending.timestamp <= t:
            estimator.add_imu(pending)
            pending = next(imu, None)
        state = estimator.add_gnss(_fix(float(t)))
    assert estimator.rejected_unaligned_fixes == 1
    assert np.linalg.norm(state.position_enu_m - _position(6.0)) < 0.2


def test_first_fix_after_a_large_heading_drift_converges_immediately():
    def imu_with_silent_gyro_fault():
        for measurement in _imu(70.0):
            fault = 0.02 if 30.0 <= measurement.timestamp < 60.0 else 0.0
            yield ImuMeasurement(
                measurement.timestamp,
                measurement.accel_mps2,
                measurement.gyro_radps + np.array([0.0, 0.0, fault]),
            )

    truth = _truth(70.0)
    blackout = DegradationWindow(DegradationMode.BLACKOUT, 30.0, 59.5)
    config = GnssDegraderConfig(windows=(blackout,), sigma_horizontal_m=0.1, sigma_vertical_m=0.1)
    gnss = list(GnssDegrader(config).degrade(truth))
    result = run_fusion(imu_with_silent_gyro_fault(), truth, gnss, PARAMS)

    by_time = {round(step.timestamp): step for step in result.steps}
    assert np.linalg.norm(by_time[59].fused.position_enu_m - by_time[59].truth_enu_m) > 20.0
    assert np.linalg.norm(by_time[60].fused.position_enu_m - by_time[60].truth_enu_m) < 1.0


def test_smoothing_pulls_the_blackout_back_towards_truth():
    blackout = DegradationWindow(DegradationMode.BLACKOUT, 30.0, 60.0)
    result = _run(90.0, blackout, accel_bias=(0.03, -0.02, 0.02))
    truth = result.truth_enu_by_time
    online = window_errors("online", (s.fused for s in result.steps), truth, 30.0, 60.0)
    smoothed = window_errors("smoothed", result.smoothed, truth, 30.0, 60.0)
    assert smoothed.horizontal_max_m < online.horizontal_max_m


def test_non_increasing_imu_stamps_are_rejected():
    estimator = FactorGraphEstimator(PARAMS, FRAME, _initial())
    estimator.add_imu(ImuMeasurement(1.0, np.zeros(3), np.zeros(3)))
    with pytest.raises(ValueError):
        estimator.add_imu(ImuMeasurement(1.0, np.zeros(3), np.zeros(3)))


def _true_motion(t_from: float, t_to: float) -> tuple[np.ndarray, np.ndarray]:
    yaw = YAW_RATE * t_from
    rotation_nav_from = gtsam.Rot3.Yaw(yaw).matrix()
    translation = rotation_nav_from.T @ (_position(t_to) - _position(t_from))
    return gtsam.Rot3.Yaw(YAW_RATE * (t_to - t_from)).matrix(), translation


def _odometry(
    duration_s: float, translation_scale: float = 1.0, skip_frames=frozenset(), noise=None
) -> list[FrameOdometry]:
    noise = noise or VoNoise(rotation_sigma_rad=1e-3, translation_sigma_m=0.02)
    stamps = [0.105 + 0.104 * i for i in range(int((duration_s - 0.105) / 0.104) + 1)]
    records = [FrameOdometry(stamps[0], None)]
    for index, (t_from, t_to) in enumerate(zip(stamps, stamps[1:], strict=False), start=1):
        if index in skip_frames:
            records.append(FrameOdometry(t_to, None))
            continue
        rotation, translation = _true_motion(t_from, t_to)
        records.append(
            FrameOdometry(
                t_to,
                make_constraint(
                    t_from, t_to, rotation, translation * translation_scale, 100, noise
                ),
            )
        )
    return records


def _run_with_vo(duration_s: float, windows, odometry, **imu_kwargs):
    truth = _truth(duration_s)
    gnss = list(
        GnssDegrader(
            GnssDegraderConfig(windows=windows, sigma_horizontal_m=0.1, sigma_vertical_m=0.1)
        ).degrade(truth)
    )
    return run_fusion(_imu(duration_s, **imu_kwargs), truth, gnss, PARAMS, odometry=odometry)


def test_visual_odometry_bounds_blackout_drift():
    blackout = (DegradationWindow(DegradationMode.BLACKOUT, 30.0, 70.0),)
    bias = {"accel_bias": (0.2, -0.15, 0.1), "gyro_bias": (0.0, 0.0, 2e-3)}
    without = _run_with_vo(70.0, blackout, (), **bias)
    with_vo = _run_with_vo(70.0, blackout, _odometry(70.0), **bias)
    truth = without.truth_enu_by_time

    baseline = window_errors("imu", (s.fused for s in without.steps), truth, 30.0, 70.0)
    aided = window_errors("vo", (s.fused for s in with_vo.steps), truth, 30.0, 70.0)
    assert baseline.horizontal_end_m > 20.0
    assert aided.horizontal_end_m < 0.2 * baseline.horizontal_end_m


def test_camera_keyframes_leave_the_scored_epochs_unchanged():
    result = _run_with_vo(20.0, (), _odometry(20.0))
    assert result.num_keyframes > 180
    assert [round(state.timestamp) for state in result.smoothed] == list(range(21))
    assert len(result.steps) == 21


def test_failed_frames_are_bridged_by_the_imu():
    blackout = (DegradationWindow(DegradationMode.BLACKOUT, 5.0, 20.0),)
    result = _run_with_vo(20.0, blackout, _odometry(20.0, skip_frames=frozenset(range(60, 90))))
    final = result.steps[-1]
    assert np.linalg.norm(final.fused.position_enu_m - final.truth_enu_m) < 1.0


def test_huber_limits_the_damage_from_a_gross_visual_outlier():
    blackout = (DegradationWindow(DegradationMode.BLACKOUT, 5.0, 15.0),)
    records = _odometry(15.0)
    bad = records[80].constraint
    records[80] = FrameOdometry(
        records[80].timestamp,
        VisualOdometryConstraint(
            bad.timestamp_from,
            bad.timestamp_to,
            bad.rotation_wxyz,
            bad.translation_m + np.array([20.0, 0.0, 0.0]),
            bad.covariance_6x6,
        ),
    )
    final = _run_with_vo(15.0, blackout, records).steps[-1]
    assert np.linalg.norm(final.fused.position_enu_m - final.truth_enu_m) < 1.0


def test_visual_odometry_carries_the_estimate_through_an_imu_hole():
    blackout = (DegradationWindow(DegradationMode.BLACKOUT, 0.5, 8.0),)
    result = _run_with_vo(8.0, blackout, _odometry(8.0), skip=(3.0, 5.0))
    final = result.steps[-1]
    assert np.linalg.norm(final.fused.position_enu_m - final.truth_enu_m) < 1.0


def test_constraint_without_scale_is_refused():
    estimator = FactorGraphEstimator(PARAMS, FRAME, _initial())
    for measurement in _imu(1.0):
        estimator.add_imu(measurement)
    estimator.add_frame_odometry(FrameOdometry(0.5, None))
    constraint = VisualOdometryConstraint(
        0.5, 1.0, [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0], np.eye(6), scale_known=False
    )
    with pytest.raises(ValueError):
        estimator.add_frame_odometry(FrameOdometry(1.0, constraint))
