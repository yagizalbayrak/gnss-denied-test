from __future__ import annotations

import numpy as np
import pytest

from gnss_denied.core.types import ImuMeasurement
from gnss_denied.simulator.imu_degrader import ImuNoiseProfile, corrupt_imu


def _still(count: int = 20000) -> list[ImuMeasurement]:
    return [ImuMeasurement(i / 100.0, np.zeros(3), np.zeros(3)) for i in range(count)]


def _profile(**overrides) -> ImuNoiseProfile:
    values = dict(
        accel_noise_density=0.0,
        gyro_noise_density=0.0,
        accel_bias_random_walk=0.0,
        gyro_bias_random_walk=0.0,
        accel_turn_on_bias_sigma=0.0,
        gyro_turn_on_bias_sigma=0.0,
    )
    return ImuNoiseProfile(**(values | overrides))


def test_white_noise_std_follows_density_and_rate():
    samples = list(corrupt_imu(_still(), _profile(accel_noise_density=0.002)))
    accel = np.array([s.accel_mps2 for s in samples])
    assert accel.std(axis=0) == pytest.approx([0.02] * 3, rel=0.05)
    assert np.array([s.gyro_radps for s in samples]).max() == 0.0


def test_turn_on_bias_is_constant_and_nonzero():
    samples = list(corrupt_imu(_still(100), _profile(gyro_turn_on_bias_sigma=1e-3)))
    gyro = np.array([s.gyro_radps for s in samples])
    assert np.ptp(gyro, axis=0).max() == 0.0
    assert np.abs(gyro[0]).max() > 0.0


def test_bias_random_walk_grows_with_sqrt_time():
    runs = [
        np.array(
            [
                s.accel_mps2
                for s in corrupt_imu(_still(10000), _profile(accel_bias_random_walk=1e-3, seed=i))
            ]
        )[-1]
        for i in range(200)
    ]
    # After 100 s the standard deviation should be 1e-3 * sqrt(100).
    assert np.std(runs) == pytest.approx(1e-2, rel=0.2)


def test_seed_is_reproducible_and_timestamps_are_preserved():
    profile = _profile(accel_noise_density=0.01, seed=3)
    first = list(corrupt_imu(_still(50), profile))
    second = list(corrupt_imu(_still(50), profile))
    assert all((a.accel_mps2 == b.accel_mps2).all() for a, b in zip(first, second, strict=True))
    assert [s.timestamp for s in first] == [i / 100.0 for i in range(50)]
