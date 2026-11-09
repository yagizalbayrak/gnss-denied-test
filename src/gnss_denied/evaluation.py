"""Trajectory metrics through `evo`, the usual reference implementation for APE and RPE.

No SE(3) alignment is applied. Estimates and truth already share a geo-referenced frame, and an
absolute error is the very thing a navigation system has to get right. Aligning first would hide
exactly the drift that this project measures.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import gtsam
import numpy as np
from evo.core import metrics
from evo.core.trajectory import PoseTrajectory3D
from evo.core.units import Unit

from gnss_denied.core.types import FloatArray, GroundTruthSample
from gnss_denied.pipeline import FusionResult


@dataclass(frozen=True)
class TrajectoryMetrics:
    ate_full_m: float
    ate_blackout_m: float
    rpe_translation_blackout_m: float
    rpe_rotation_blackout_deg: float


def _trajectory(
    timestamps: FloatArray, positions: FloatArray, orientations_wxyz: FloatArray
) -> PoseTrajectory3D:
    return PoseTrajectory3D(
        positions_xyz=positions,
        orientations_quat_wxyz=orientations_wxyz,
        timestamps=timestamps,
    )


def ate_rmse(reference: PoseTrajectory3D, estimate: PoseTrajectory3D) -> float:
    """Root mean square of the absolute position error, in metres."""
    metric = metrics.APE(metrics.PoseRelation.translation_part)
    metric.process_data((reference, estimate))
    return float(metric.get_statistic(metrics.StatisticsType.rmse))


def rpe_rmse(
    reference: PoseTrajectory3D,
    estimate: PoseTrajectory3D,
    delta_poses: int,
    relation: metrics.PoseRelation = metrics.PoseRelation.translation_part,
) -> float:
    """Root mean square of the error of motion over `delta_poses` steps."""
    metric = metrics.RPE(relation, delta=delta_poses, delta_unit=Unit.frames, all_pairs=False)
    metric.process_data((reference, estimate))
    return float(metric.get_statistic(metrics.StatisticsType.rmse))


def evaluate_result(
    result: FusionResult,
    truth: Sequence[GroundTruthSample],
    blackout: tuple[float, float],
    rpe_delta_poses: int = 10,
) -> TrajectoryMetrics:
    """Score the online estimate, which is what a vehicle would have had at that moment."""
    truth_by_time = {sample.timestamp: sample for sample in truth}
    timestamps = np.array([step.timestamp for step in result.steps])
    estimated_positions = np.array([step.fused.position_enu_m for step in result.steps])
    estimated_orientations = np.array([step.fused.orientation_wxyz for step in result.steps])
    true_positions = np.array([step.truth_enu_m for step in result.steps])
    true_orientations = np.array([_truth_wxyz(truth_by_time[t]) for t in timestamps])

    def trajectories(mask: FloatArray) -> tuple[PoseTrajectory3D, PoseTrajectory3D]:
        reference = _trajectory(timestamps[mask], true_positions[mask], true_orientations[mask])
        estimate = _trajectory(
            timestamps[mask], estimated_positions[mask], estimated_orientations[mask]
        )
        return reference, estimate

    everything = np.ones(len(timestamps), dtype=bool)
    in_blackout = (timestamps >= blackout[0]) & (timestamps <= blackout[1])
    blackout_pair = trajectories(in_blackout)
    return TrajectoryMetrics(
        ate_full_m=ate_rmse(*trajectories(everything)),
        ate_blackout_m=ate_rmse(*blackout_pair),
        rpe_translation_blackout_m=rpe_rmse(*blackout_pair, rpe_delta_poses),
        rpe_rotation_blackout_deg=rpe_rmse(
            *blackout_pair, rpe_delta_poses, metrics.PoseRelation.rotation_angle_deg
        ),
    )


def _truth_wxyz(sample: GroundTruthSample) -> FloatArray:
    quaternion = gtsam.Rot3.Ypr(sample.yaw_rad, sample.pitch_rad, sample.roll_rad).toQuaternion()
    return np.array([quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()])
