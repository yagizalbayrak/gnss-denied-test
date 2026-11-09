"""GTSAM back-end: fuses IMU pre-integration, GNSS and visual odometry in an incremental graph.

The graph holds one (pose, velocity, bias) triple per keyframe. Keyframes are created at GNSS
epochs, including denied ones, at camera frames, and around holes in the IMU stream. Two
consecutive keyframes are linked by an ImuFactor and a bias random walk. A keyframe with a fix also
gets a GPSFactor, and two camera keyframes can be linked by a robust BetweenFactorPose3.

Navigation frame is ENU with gravity along -z (`MakeSharedU`). Body frame is x forward, y left,
z up, and an accelerometer at rest reads +g on z.
"""

from __future__ import annotations

from dataclasses import dataclass

import gtsam
import numpy as np
from gtsam.symbol_shorthand import B, V, X

from gnss_denied.core.geodesy import LocalEnuFrame
from gnss_denied.core.types import (
    FloatArray,
    GnssMeasurement,
    ImuMeasurement,
    StateOutput,
    VisualOdometryConstraint,
)
from gnss_denied.frontend.visual_odom import FrameOdometry

_NO_COVARIANCE = np.diag([np.inf, np.inf, np.inf])
_VO_HUBER_THRESHOLD = 1.345


@dataclass(frozen=True)
class ImuParams:
    """Noise model assumed by the estimator. Densities are continuous-time values."""

    gravity_mps2: float = 9.809
    accel_noise_density: float = 0.01  # m/s^2/sqrt(Hz)
    gyro_noise_density: float = 2e-4  # rad/s/sqrt(Hz)
    accel_bias_random_walk: float = 1e-3  # m/s^3/sqrt(Hz)
    gyro_bias_random_walk: float = 1e-5  # rad/s^2/sqrt(Hz)
    integration_sigma: float = 1e-4  # m/s/sqrt(Hz)
    # A hole longer than this is bridged by a factor with the uncertainty below, because the white
    # noise model says nothing about the motion that went unobserved.
    max_nominal_gap_s: float = 0.2
    gap_accel_sigma_mps2: float = 1.0
    gap_gyro_sigma_radps: float = 0.1
    max_fix_offset_s: float = 0.02
    # ISAM2 takes one Gauss-Newton step per update, which cannot absorb the correction after a long
    # blackout. A fix this far from the prediction gets extra iterations.
    large_innovation_m: float = 5.0
    large_innovation_iterations: int = 3


@dataclass(frozen=True)
class InitialState:
    """Starting state and its uncertainty. Attitude angles follow the OXTS convention."""

    timestamp: float
    position_enu_m: FloatArray
    velocity_enu_mps: FloatArray
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    position_sigma_m: float = 10.0
    velocity_sigma_mps: float = 0.1
    # Keep the bias priors close to the sensor's datasheet. With a loose gyro prior, the graph
    # explains GNSS noise with a bias the sensor does not have, and then drifts in a blackout.
    attitude_sigma_rad: float = 0.005
    accel_bias_sigma_mps2: float = 0.03
    gyro_bias_sigma_radps: float = 5e-4

    def nav_state(self) -> gtsam.NavState:
        rotation = gtsam.Rot3.Ypr(self.yaw_rad, self.pitch_rad, self.roll_rad)
        pose = gtsam.Pose3(rotation, np.asarray(self.position_enu_m, dtype=np.float64))
        return gtsam.NavState(pose, np.asarray(self.velocity_enu_mps, dtype=np.float64))


def _preintegration_params(
    params: ImuParams, accel_density: float, gyro_density: float
) -> gtsam.PreintegrationParams:
    pim_params = gtsam.PreintegrationParams.MakeSharedU(params.gravity_mps2)
    pim_params.setAccelerometerCovariance(np.eye(3) * accel_density**2)
    pim_params.setGyroscopeCovariance(np.eye(3) * gyro_density**2)
    pim_params.setIntegrationCovariance(np.eye(3) * params.integration_sigma**2)
    return pim_params


def _first_order_hold(
    previous: ImuMeasurement, current: ImuMeasurement
) -> tuple[FloatArray, FloatArray, float]:
    """Mean of two neighbouring samples and the interval between them."""
    dt = current.timestamp - previous.timestamp
    if dt <= 0.0:
        raise ValueError(f"IMU timestamps must increase: {previous.timestamp}, {current.timestamp}")
    accel = 0.5 * (previous.accel_mps2 + current.accel_mps2)
    gyro = 0.5 * (previous.gyro_radps + current.gyro_radps)
    return accel, gyro, dt


def _state_output(
    timestamp: float, state: gtsam.NavState, covariance_m2: FloatArray = _NO_COVARIANCE
) -> StateOutput:
    quaternion = state.attitude().toQuaternion()
    return StateOutput(
        timestamp=timestamp,
        position_enu_m=state.position(),
        velocity_enu_mps=state.velocity(),
        orientation_wxyz=_normalised(quaternion),
        position_covariance_m2=covariance_m2,
    )


def _normalised(quaternion: gtsam.Quaternion) -> FloatArray:
    wxyz = np.array([quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()])
    return wxyz / np.linalg.norm(wxyz)


class DeadReckoner:
    """Open-loop IMU integration with a fixed bias: the baseline with no aiding at all."""

    def __init__(self, params: ImuParams, initial: InitialState) -> None:
        pim_params = _preintegration_params(
            params, params.accel_noise_density, params.gyro_noise_density
        )
        self._bias = gtsam.imuBias.ConstantBias()
        self._pim = gtsam.PreintegratedImuMeasurements(pim_params, self._bias)
        self._state = initial.nav_state()
        self._timestamp = initial.timestamp
        self._previous: ImuMeasurement | None = None

    def add_imu(self, measurement: ImuMeasurement) -> None:
        if self._previous is not None:
            accel, gyro, dt = _first_order_hold(self._previous, measurement)
            self._pim.resetIntegration()
            self._pim.integrateMeasurement(accel, gyro, dt)
            self._state = self._pim.predict(self._state, self._bias)
            self._timestamp = measurement.timestamp
        self._previous = measurement

    def state(self) -> StateOutput:
        return _state_output(self._timestamp, self._state)


class FactorGraphEstimator:
    """Online ISAM2 smoother. Feed it time-ordered IMU and GNSS packets."""

    def __init__(self, params: ImuParams, frame: LocalEnuFrame, initial: InitialState) -> None:
        self._params = params
        self._frame = frame
        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(0.01)
        isam_params.relinearizeSkip = 1
        # Tight IMU links next to very loose gap links are too ill-conditioned for Cholesky, which
        # squares the condition number and reports an indeterminate system.
        isam_params.setFactorization("QR")
        self._isam = gtsam.ISAM2(isam_params)
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()

        self._state = initial.nav_state()
        self._bias = gtsam.imuBias.ConstantBias()
        self._pim_params = _preintegration_params(
            params, params.accel_noise_density, params.gyro_noise_density
        )
        self._pim = gtsam.PreintegratedImuMeasurements(self._pim_params, self._bias)
        self._previous_imu: ImuMeasurement | None = None
        self._integrated_until = initial.timestamp
        self._key = 0
        self._key_timestamps = [initial.timestamp]
        self._key_by_time = {initial.timestamp: 0}
        self.rejected_unaligned_fixes = 0

        pose_sigmas = np.r_[
            np.full(3, initial.attitude_sigma_rad), np.full(3, initial.position_sigma_m)
        ]
        bias_sigmas = np.r_[
            np.full(3, initial.accel_bias_sigma_mps2), np.full(3, initial.gyro_bias_sigma_radps)
        ]
        self._graph.add(
            gtsam.PriorFactorPose3(
                X(0), self._state.pose(), gtsam.noiseModel.Diagonal.Sigmas(pose_sigmas)
            )
        )
        self._graph.add(
            gtsam.PriorFactorVector(
                V(0),
                self._state.velocity(),
                gtsam.noiseModel.Isotropic.Sigma(3, initial.velocity_sigma_mps),
            )
        )
        self._graph.add(
            gtsam.PriorFactorConstantBias(
                B(0), self._bias, gtsam.noiseModel.Diagonal.Sigmas(bias_sigmas)
            )
        )
        self._insert_values()

    @property
    def num_keyframes(self) -> int:
        return self._key + 1

    @property
    def bias(self) -> gtsam.imuBias.ConstantBias:
        return self._bias

    def add_imu(self, measurement: ImuMeasurement) -> None:
        if self._previous_imu is not None:
            accel, gyro, sample_interval_s = _first_order_hold(self._previous_imu, measurement)
            silent = sample_interval_s > self._params.max_nominal_gap_s
            self._integrate(accel, gyro, measurement.timestamp, silent)
        self._previous_imu = measurement

    def add_frame_odometry(self, record: FrameOdometry) -> None:
        """Create a keyframe at this camera frame and attach its motion constraint, if any.

        The IMU rarely samples at the exact frame time, so the latest sample is held up to it. VO
        keeps bounding the motion inside an IMU hole, which is why frames there still get a
        keyframe, with the inflated uncertainty of `_bridge_gap`.
        """
        if self._previous_imu is not None:
            previous = self._previous_imu
            silent = record.timestamp - previous.timestamp > self._params.max_nominal_gap_s
            self._integrate(previous.accel_mps2, previous.gyro_radps, record.timestamp, silent)
        if self._pim.deltaTij() > 0.0:
            self._close_keyframe(self._pim)
        self._key_by_time[record.timestamp] = self._key
        if record.constraint is not None:
            self._add_visual_odometry_factor(record.constraint)
        self._commit()

    def add_gnss(self, measurement: GnssMeasurement) -> StateOutput:
        """Close the keyframe at this epoch, attach the fix if there is one, and re-solve.

        The keyframe sits at the latest IMU sample. A fix taken while the IMU is silent describes
        a different moment, and attaching it would turn motion into a large position error, so it
        is rejected instead.
        """
        if self._pim.deltaTij() > 0.0:
            self._close_keyframe(self._pim)
        extra_iterations = 0
        if measurement.has_fix:
            offset_s = abs(measurement.timestamp - self._integrated_until)
            if offset_s <= self._params.max_fix_offset_s:
                innovation_m = self._add_gps_factor(measurement)
                if innovation_m > self._params.large_innovation_m:
                    extra_iterations = self._params.large_innovation_iterations
            else:
                self.rejected_unaligned_fixes += 1
        self._commit(extra_iterations)
        return self.current_state()

    def current_state(self) -> StateOutput:
        pose_covariance = self._isam.marginalCovariance(X(self._key))
        rotation = self._state.attitude().matrix()
        position_covariance = rotation @ pose_covariance[3:, 3:] @ rotation.T
        symmetric = 0.5 * (position_covariance + position_covariance.T)
        return _state_output(self._integrated_until, self._state, symmetric)

    def smoothed_trajectory(self) -> list[StateOutput]:
        """Every keyframe re-estimated with all data so far, including data from after a blackout.

        This is what a post-mission solution sees. It is not available in real time, so it must
        not be used to judge online drift.
        """
        estimate = self._isam.calculateEstimate()
        return [
            _state_output(
                timestamp,
                gtsam.NavState(estimate.atPose3(X(key)), estimate.atVector(V(key))),
            )
            for key, timestamp in enumerate(self._key_timestamps)
        ]

    def _integrate(self, accel: FloatArray, gyro: FloatArray, until: float, silent: bool) -> None:
        """Integrate from the current point up to `until`, which may be between IMU samples."""
        dt = until - self._integrated_until
        if dt <= 0.0:
            return
        if silent:
            self._bridge_gap(accel, gyro, dt, until)
        else:
            self._pim.integrateMeasurement(accel, gyro, dt)
            self._integrated_until = until

    def _bridge_gap(self, accel: FloatArray, gyro: FloatArray, dt: float, timestamp: float) -> None:
        """Span a hole in the IMU stream with its own keyframe pair and inflated uncertainty.

        Holding a value for dt gives velocity variance Q * dt, so Q = sigma^2 * dt yields a
        velocity uncertainty of sigma * dt for an unknown acceleration of size sigma.
        """
        if self._pim.deltaTij() > 0.0:
            self._close_keyframe(self._pim)
        params = self._params
        gap_params = _preintegration_params(
            params,
            max(params.gap_accel_sigma_mps2 * np.sqrt(dt), params.accel_noise_density),
            max(params.gap_gyro_sigma_radps * np.sqrt(dt), params.gyro_noise_density),
        )
        gap_pim = gtsam.PreintegratedImuMeasurements(gap_params, self._bias)
        gap_pim.integrateMeasurement(accel, gyro, dt)
        self._integrated_until = timestamp
        self._close_keyframe(gap_pim)

    def _close_keyframe(self, pim: gtsam.PreintegratedImuMeasurements) -> None:
        previous = self._key
        self._key += 1
        self._key_timestamps.append(self._integrated_until)
        self._key_by_time[self._integrated_until] = self._key
        self._graph.add(
            gtsam.ImuFactor(X(previous), V(previous), X(self._key), V(self._key), B(previous), pim)
        )
        sqrt_dt = np.sqrt(pim.deltaTij())
        bias_sigmas = (
            sqrt_dt
            * np.r_[
                np.full(3, self._params.accel_bias_random_walk),
                np.full(3, self._params.gyro_bias_random_walk),
            ]
        )
        self._graph.add(
            gtsam.BetweenFactorConstantBias(
                B(previous),
                B(self._key),
                gtsam.imuBias.ConstantBias(),
                gtsam.noiseModel.Diagonal.Sigmas(bias_sigmas),
            )
        )
        self._state = pim.predict(self._state, self._bias)
        self._insert_values()
        self._pim.resetIntegrationAndSetBias(self._bias)

    def _insert_values(self) -> None:
        self._values.insert(X(self._key), self._state.pose())
        self._values.insert(V(self._key), self._state.velocity())
        self._values.insert(B(self._key), self._bias)

    def _add_gps_factor(self, measurement: GnssMeasurement) -> float:
        """Add the fix to the pending graph and return its distance from the prediction."""
        position = self._frame.from_geodetic(
            measurement.lat_deg, measurement.lon_deg, measurement.alt_m
        )
        covariance = self._frame.rotate_covariance_from_fix(
            measurement.covariance_enu_m2, measurement.lat_deg, measurement.lon_deg
        )
        noise = gtsam.noiseModel.Gaussian.Covariance(0.5 * (covariance + covariance.T))
        self._graph.add(gtsam.GPSFactor(X(self._key), position, noise))
        return float(np.linalg.norm(position - self._state.position()))

    def _add_visual_odometry_factor(self, constraint: VisualOdometryConstraint) -> None:
        if not constraint.scale_known:
            raise ValueError("the graph needs metric odometry, but this constraint has no scale")
        key_from = self._key_by_time.get(constraint.timestamp_from)
        key_to = self._key_by_time.get(constraint.timestamp_to)
        if key_from is None or key_to is None:
            raise ValueError(
                f"no keyframe for constraint {constraint.timestamp_from} -> "
                f"{constraint.timestamp_to}"
            )
        w, x, y, z = constraint.rotation_wxyz
        motion = gtsam.Pose3(gtsam.Rot3.Quaternion(w, x, y, z), constraint.translation_m)
        # Huber keeps a bad frame's pull on the graph bounded, instead of quadratic in its error.
        noise = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(_VO_HUBER_THRESHOLD),
            gtsam.noiseModel.Gaussian.Covariance(constraint.covariance_6x6),
        )
        self._graph.add(gtsam.BetweenFactorPose3(X(key_from), X(key_to), motion, noise))

    def _commit(self, extra_iterations: int = 0) -> None:
        self._isam.update(self._graph, self._values)
        for _ in range(extra_iterations):
            self._isam.update()
        self._graph = gtsam.NonlinearFactorGraph()
        self._values = gtsam.Values()
        self._state = gtsam.NavState(
            self._isam.calculateEstimatePose3(X(self._key)),
            self._isam.calculateEstimateVector(V(self._key)),
        )
        self._bias = self._isam.calculateEstimateConstantBias(B(self._key))
        self._pim.resetIntegrationAndSetBias(self._bias)
