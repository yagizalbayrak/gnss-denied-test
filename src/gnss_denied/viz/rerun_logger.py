"""Streams a fusion run to Rerun: trajectories, covariance, GNSS fixes and error curves."""

from __future__ import annotations

from pathlib import Path

import gtsam
import numpy as np
import rerun as rr

from gnss_denied.core.types import FloatArray
from gnss_denied.pipeline import FusionStep

_TRUTH_COLOR = (230, 230, 230)
_FUSED_COLOR = (64, 160, 255)
_DEAD_RECKONING_COLOR = (255, 140, 60)
_GNSS_COLOR = (90, 200, 120)
_SIGMA_SCALE = 3.0


def covariance_ellipsoid(covariance_m2: FloatArray) -> tuple[FloatArray, FloatArray]:
    """Return 3-sigma half sizes and an (x, y, z, w) quaternion for a position covariance."""
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_m2)
    if np.linalg.det(eigenvectors) < 0.0:
        eigenvectors[:, 0] = -eigenvectors[:, 0]
    quaternion = gtsam.Rot3(eigenvectors).toQuaternion()
    half_sizes = _SIGMA_SCALE * np.sqrt(np.clip(eigenvalues, 0.0, None))
    return half_sizes, np.array([quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()])


class RerunLogger:
    def __init__(self, spawn: bool, save_path: Path | None = None) -> None:
        rr.init("open_gnss_denied", spawn=spawn)
        if save_path is not None:
            rr.save(save_path)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self._truth: list[FloatArray] = []
        self._fused: list[FloatArray] = []
        self._dead_reckoning: list[FloatArray] = []
        self._gnss: list[FloatArray] = []

    def log_step(self, step: FusionStep) -> None:
        rr.set_time("drive_time", duration=step.timestamp)
        self._truth.append(step.truth_enu_m)
        self._fused.append(step.fused.position_enu_m)
        self._dead_reckoning.append(step.dead_reckoning.position_enu_m)

        self._log_path("world/ground_truth", self._truth, _TRUTH_COLOR)
        self._log_path("world/fused", self._fused, _FUSED_COLOR)
        self._log_path("world/dead_reckoning", self._dead_reckoning, _DEAD_RECKONING_COLOR)

        if step.gnss_enu_m is not None:
            self._gnss.append(step.gnss_enu_m)
        rr.log("world/gnss", rr.Points3D(self._gnss, colors=_GNSS_COLOR, radii=0.5))

        half_sizes, quaternion = covariance_ellipsoid(step.fused.position_covariance_m2)
        rr.log(
            "world/fused_3sigma",
            rr.Ellipsoids3D(
                centers=[step.fused.position_enu_m],
                half_sizes=[half_sizes],
                quaternions=[rr.Quaternion(xyzw=quaternion)],
                colors=_FUSED_COLOR,
            ),
        )

        fused_error = _horizontal_error(step.fused.position_enu_m, step.truth_enu_m)
        reckoned_error = _horizontal_error(step.dead_reckoning.position_enu_m, step.truth_enu_m)
        rr.log("errors/fused_m", rr.Scalars(fused_error))
        rr.log("errors/dead_reckoning_m", rr.Scalars(reckoned_error))
        rr.log("gnss/status", rr.TextLog(step.gnss.status.name))

    @staticmethod
    def _log_path(entity: str, points: list[FloatArray], color: tuple[int, int, int]) -> None:
        rr.log(entity, rr.LineStrips3D([np.array(points)], colors=color, radii=0.3))


def _horizontal_error(position: FloatArray, truth: FloatArray) -> float:
    return float(np.linalg.norm((position - truth)[:2]))
