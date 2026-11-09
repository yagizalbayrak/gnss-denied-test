"""Runs the fusion back-end over recorded streams and scores it against ground truth."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from gnss_denied.backend.factor_graph import (
    DeadReckoner,
    FactorGraphEstimator,
    ImuParams,
    InitialState,
)
from gnss_denied.core.geodesy import LocalEnuFrame
from gnss_denied.core.streams import merge_streams
from gnss_denied.core.types import (
    FloatArray,
    GnssMeasurement,
    GroundTruthSample,
    ImuMeasurement,
    StateOutput,
)
from gnss_denied.frontend.visual_odom import FrameOdometry
from gnss_denied.simulator.imu_degrader import ImuNoiseProfile


@dataclass(frozen=True)
class FusionStep:
    """Everything known at one GNSS epoch. `fused` is the online estimate at that moment."""

    timestamp: float
    truth_enu_m: FloatArray
    fused: StateOutput
    dead_reckoning: StateOutput
    gnss: GnssMeasurement
    gnss_enu_m: FloatArray | None


@dataclass(frozen=True)
class FusionResult:
    steps: list[FusionStep]
    smoothed: list[StateOutput]
    truth_enu_by_time: dict[float, FloatArray]
    num_keyframes: int
    accel_bias_mps2: FloatArray
    gyro_bias_radps: FloatArray


@dataclass(frozen=True)
class WindowErrors:
    """Horizontal and 3D error statistics of one estimate over one time window."""

    name: str
    samples: int
    horizontal_rms_m: float
    horizontal_max_m: float
    horizontal_end_m: float
    position_rms_m: float


@dataclass(frozen=True)
class ImuProfile:
    """One sensor grade: the noise to inject, the estimator's model and its initial sigmas."""

    estimator: ImuParams
    injected: ImuNoiseProfile | None
    initial_sigmas: dict[str, float]


def load_imu_profile(path: str | Path, profile_name: str) -> ImuProfile:
    with open(path, encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)["imu_profiles"][profile_name]
    injected = profile.get("injected")
    return ImuProfile(
        estimator=ImuParams(**profile["estimator"]),
        injected=None if injected is None else ImuNoiseProfile(**injected),
        initial_sigmas=profile.get("initial_sigmas", {}),
    )


def run_fusion(
    imu: Iterable[ImuMeasurement],
    truth: Sequence[GroundTruthSample],
    gnss: Sequence[GnssMeasurement],
    params: ImuParams,
    on_step: Callable[[FusionStep], None] | None = None,
    initial_sigmas: dict[str, float] | None = None,
    odometry: Iterable[FrameOdometry] = (),
) -> FusionResult:
    """Fuse IMU and GNSS. The local frame is anchored at the first valid fix.

    Position starts at that fix. Velocity and attitude come from the first ground truth sample,
    standing in for the initial alignment that a real system does before it starts moving.
    """
    first_fix = next(measurement for measurement in gnss if measurement.has_fix)
    frame = LocalEnuFrame(first_fix.lat_deg, first_fix.lon_deg, first_fix.alt_m)
    first = truth[0]
    initial = InitialState(
        timestamp=first.timestamp,
        position_enu_m=frame.from_geodetic(first_fix.lat_deg, first_fix.lon_deg, first_fix.alt_m),
        velocity_enu_mps=first.velocity_enu_mps,
        roll_rad=first.roll_rad,
        pitch_rad=first.pitch_rad,
        yaw_rad=first.yaw_rad,
        **(initial_sigmas or {}),
    )
    estimator = FactorGraphEstimator(params, frame, initial)
    dead_reckoner = DeadReckoner(params, initial)
    truth_enu_by_time = {
        sample.timestamp: frame.from_geodetic(sample.lat_deg, sample.lon_deg, sample.alt_m)
        for sample in truth
    }

    steps: list[FusionStep] = []
    for packet in merge_streams(imu, odometry, gnss):
        if isinstance(packet, ImuMeasurement):
            estimator.add_imu(packet)
            dead_reckoner.add_imu(packet)
            continue
        if isinstance(packet, FrameOdometry):
            estimator.add_frame_odometry(packet)
            continue
        step = FusionStep(
            timestamp=packet.timestamp,
            truth_enu_m=truth_enu_by_time[packet.timestamp],
            fused=estimator.add_gnss(packet),
            dead_reckoning=dead_reckoner.state(),
            gnss=packet,
            gnss_enu_m=(
                frame.from_geodetic(packet.lat_deg, packet.lon_deg, packet.alt_m)
                if packet.has_fix
                else None
            ),
        )
        steps.append(step)
        if on_step is not None:
            on_step(step)

    return FusionResult(
        steps=steps,
        # Camera keyframes fall between truth samples, so only the GNSS epochs can be scored.
        smoothed=[
            state
            for state in estimator.smoothed_trajectory()
            if state.timestamp in truth_enu_by_time
        ],
        truth_enu_by_time=truth_enu_by_time,
        num_keyframes=estimator.num_keyframes,
        accel_bias_mps2=np.array(estimator.bias.accelerometer()),
        gyro_bias_radps=np.array(estimator.bias.gyroscope()),
    )


def window_errors(
    name: str,
    states: Iterable[StateOutput],
    truth_enu_by_time: dict[float, FloatArray],
    start_s: float = -np.inf,
    end_s: float = np.inf,
) -> WindowErrors:
    """Score the states whose timestamp lies in the closed window [start_s, end_s]."""
    errors = np.array(
        [
            state.position_enu_m - truth_enu_by_time[state.timestamp]
            for state in states
            if start_s <= state.timestamp <= end_s
        ]
    )
    if errors.size == 0:
        raise ValueError(f"no states in window [{start_s}, {end_s}] for {name}")
    horizontal = np.linalg.norm(errors[:, :2], axis=1)
    return WindowErrors(
        name=name,
        samples=len(errors),
        horizontal_rms_m=float(np.sqrt(np.mean(horizontal**2))),
        horizontal_max_m=float(horizontal.max()),
        horizontal_end_m=float(horizontal[-1]),
        position_rms_m=float(np.sqrt(np.mean(np.sum(errors**2, axis=1)))),
    )
