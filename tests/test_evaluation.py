"""Metrics checked against trajectories whose errors can be worked out by hand."""

from __future__ import annotations

import numpy as np
import pytest
from evo.core import metrics

from gnss_denied.core.types import GnssMeasurement, GroundTruthSample, StateOutput
from gnss_denied.evaluation import _trajectory, ate_rmse, evaluate_result, rpe_rmse
from gnss_denied.pipeline import FusionResult, FusionStep

IDENTITY = np.tile([1.0, 0.0, 0.0, 0.0], (100, 1))
TIMES = np.arange(100.0)
STRAIGHT = np.column_stack([TIMES * 5.0, np.zeros(100), np.zeros(100)])


def _pair(estimate_positions, orientations=IDENTITY):
    return (
        _trajectory(TIMES, STRAIGHT, IDENTITY),
        _trajectory(TIMES, estimate_positions, orientations),
    )


def test_identical_trajectories_score_zero():
    reference, estimate = _pair(STRAIGHT.copy())
    assert ate_rmse(reference, estimate) == pytest.approx(0.0, abs=1e-9)
    assert rpe_rmse(reference, estimate, 10) == pytest.approx(0.0, abs=1e-9)


def test_constant_offset_shows_in_ate_but_not_in_rpe():
    reference, estimate = _pair(STRAIGHT + np.array([3.0, 4.0, 0.0]))
    assert ate_rmse(reference, estimate) == pytest.approx(5.0)
    assert rpe_rmse(reference, estimate, 10) == pytest.approx(0.0, abs=1e-9)


def test_no_alignment_hides_a_scale_error():
    reference, estimate = _pair(STRAIGHT * 1.1)
    expected = np.sqrt(np.mean((0.1 * STRAIGHT[:, 0]) ** 2))
    assert ate_rmse(reference, estimate) == pytest.approx(expected)
    # 10 poses cover 50 m, so a 10% scale error gives 5 m of relative error.
    assert rpe_rmse(reference, estimate, 10) == pytest.approx(5.0)


def test_rotation_rpe_measures_a_yaw_drift_in_degrees():
    yaw = np.radians(TIMES)  # One degree per pose.
    drifting = np.column_stack([np.cos(yaw / 2), np.zeros(100), np.zeros(100), np.sin(yaw / 2)])
    reference, estimate = _pair(STRAIGHT.copy(), drifting)
    relation = metrics.PoseRelation.rotation_angle_deg
    assert rpe_rmse(reference, estimate, 10, relation) == pytest.approx(10.0, abs=1e-6)


def test_evaluate_result_splits_the_blackout_from_the_full_drive():
    truth, steps = [], []
    for t in TIMES:
        position = np.array([t * 5.0, 0.0, 0.0])
        error_m = 10.0 if 30.0 <= t <= 60.0 else 0.0
        state = StateOutput(
            t,
            position + np.array([0.0, error_m, 0.0]),
            np.zeros(3),
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.eye(3),
        )
        truth.append(GroundTruthSample(t, 49.0, 8.0, 100.0, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0]))
        gnss = GnssMeasurement(t, 49.0, 8.0, 100.0, np.eye(3))
        steps.append(FusionStep(t, position, state, state, gnss, None))
    result = FusionResult(steps, [], {}, 100, np.zeros(3), np.zeros(3))

    scores = evaluate_result(result, truth, (30.0, 60.0))
    assert scores.ate_blackout_m == pytest.approx(10.0)
    assert scores.ate_full_m == pytest.approx(10.0 * np.sqrt(31 / 100))
    assert scores.rpe_translation_blackout_m == pytest.approx(0.0, abs=1e-9)
    assert scores.rpe_rotation_blackout_deg == pytest.approx(0.0, abs=1e-6)
