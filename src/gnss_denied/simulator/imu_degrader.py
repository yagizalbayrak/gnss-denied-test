"""Synthetic IMU corruption: makes the dataset's navigation-grade OXTS behave like a cheaper sensor.

OXTS gyros and accelerometers are far better than the MEMS parts on a typical small platform, so
open-loop drift on the raw data understates the problem this project is about.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass

import numpy as np

from gnss_denied.core.types import ImuMeasurement


@dataclass(frozen=True)
class ImuNoiseProfile:
    """Errors added on top of the raw signal. Densities are continuous-time values."""

    accel_noise_density: float  # m/s^2/sqrt(Hz)
    gyro_noise_density: float  # rad/s/sqrt(Hz)
    accel_bias_random_walk: float  # m/s^3/sqrt(Hz)
    gyro_bias_random_walk: float  # rad/s^2/sqrt(Hz)
    accel_turn_on_bias_sigma: float  # m/s^2
    gyro_turn_on_bias_sigma: float  # rad/s
    seed: int = 0


def corrupt_imu(
    measurements: Iterable[ImuMeasurement], profile: ImuNoiseProfile
) -> Iterator[ImuMeasurement]:
    """Add a constant turn-on bias, a bias random walk and white noise to each sample.

    White noise with density d has standard deviation d / sqrt(dt) at sample interval dt. A random
    walk with density d grows by d * sqrt(dt) per step.
    """
    rng = np.random.default_rng(profile.seed)
    accel_bias = rng.standard_normal(3) * profile.accel_turn_on_bias_sigma
    gyro_bias = rng.standard_normal(3) * profile.gyro_turn_on_bias_sigma
    previous_timestamp: float | None = None
    nominal_dt = 0.01

    for measurement in measurements:
        dt = (
            nominal_dt if previous_timestamp is None else measurement.timestamp - previous_timestamp
        )
        previous_timestamp = measurement.timestamp
        accel_bias = accel_bias + (
            rng.standard_normal(3) * profile.accel_bias_random_walk * np.sqrt(dt)
        )
        gyro_bias = gyro_bias + rng.standard_normal(3) * profile.gyro_bias_random_walk * np.sqrt(dt)
        accel_noise = rng.standard_normal(3) * profile.accel_noise_density / np.sqrt(dt)
        gyro_noise = rng.standard_normal(3) * profile.gyro_noise_density / np.sqrt(dt)
        yield ImuMeasurement(
            measurement.timestamp,
            measurement.accel_mps2 + accel_bias + accel_noise,
            measurement.gyro_radps + gyro_bias + gyro_noise,
        )
