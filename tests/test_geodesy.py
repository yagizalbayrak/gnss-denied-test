"""Verification gate for Phase 1: round-trip geodetic conversions must stay below 1 mm."""

from __future__ import annotations

import numpy as np
import pytest

from gnss_denied.core.geodesy import (
    LocalEnuFrame,
    ecef_to_enu_rotation,
    ecef_to_geodetic,
    geodetic_to_ecef,
)
from gnss_denied.core.types import GnssMeasurement, GnssStatus, ImuMeasurement

MAX_ERROR_M = 1e-3

# Karlsruhe (KITTI), the equator, a southern site across the antimeridian, near-pole, high altitude.
SITES = [
    (49.0113, 8.4163, 112.0),
    (0.0, 0.0, 0.0),
    (-41.2865, 179.9, 50.0),
    (89.5, -120.0, 2800.0),
    (39.9, 32.85, 10000.0),
]


def _position_error_m(lat_a, lon_a, alt_a, lat_b, lon_b, alt_b) -> float:
    point_a = geodetic_to_ecef(lat_a, lon_a, alt_a)
    point_b = geodetic_to_ecef(lat_b, lon_b, alt_b)
    return float(np.linalg.norm(point_a - point_b))


def test_ecef_matches_known_value_at_equator_prime_meridian():
    np.testing.assert_allclose(geodetic_to_ecef(0.0, 0.0, 0.0), [6378137.0, 0.0, 0.0], atol=1e-6)


def test_ecef_matches_wgs84_polar_radius():
    np.testing.assert_allclose(
        geodetic_to_ecef(90.0, 0.0, 0.0), [0.0, 0.0, 6356752.314245], atol=1e-5
    )


@pytest.mark.parametrize("lat, lon, alt", SITES)
def test_geodetic_ecef_round_trip(lat, lon, alt):
    recovered = ecef_to_geodetic(geodetic_to_ecef(lat, lon, alt))
    assert _position_error_m(lat, lon, alt, *recovered) < MAX_ERROR_M


@pytest.mark.parametrize("lat, lon, alt", SITES)
@pytest.mark.parametrize(
    "enu",
    [(0.0, 0.0, 0.0), (120.5, -340.25, 3.0), (-5000.0, 12000.0, -80.0), (40000.0, 40000.0, 500.0)],
)
def test_enu_geodetic_round_trip(lat, lon, alt, enu):
    frame = LocalEnuFrame(lat, lon, alt)
    enu = np.array(enu)
    recovered = frame.from_geodetic(*frame.to_geodetic(enu))
    assert np.linalg.norm(recovered - enu) < MAX_ERROR_M


@pytest.mark.parametrize("lat, lon, alt", SITES)
def test_origin_maps_to_zero(lat, lon, alt):
    frame = LocalEnuFrame(lat, lon, alt)
    assert np.linalg.norm(frame.from_geodetic(lat, lon, alt)) < MAX_ERROR_M


def test_enu_axes_point_the_right_way():
    frame = LocalEnuFrame(49.0, 8.0, 100.0)
    east, north, up = frame.from_geodetic(49.0, 8.001, 100.0)
    assert east > 0 and abs(north) < 1.0 and abs(up) < 1.0
    east, north, up = frame.from_geodetic(49.001, 8.0, 100.0)
    assert north > 0 and abs(east) < 1.0 and abs(up) < 1.0
    east, north, up = frame.from_geodetic(49.0, 8.0, 150.0)
    np.testing.assert_allclose([east, north, up], [0.0, 0.0, 50.0], atol=MAX_ERROR_M)


@pytest.mark.parametrize("lat, lon, alt", SITES)
def test_rotation_is_orthonormal_and_agrees_with_pymap3d(lat, lon, alt):
    rotation = ecef_to_enu_rotation(lat, lon)
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)

    frame = LocalEnuFrame(lat, lon, alt)
    offset_ecef = np.array([12.0, -7.0, 3.0])
    expected = frame.from_ecef(geodetic_to_ecef(lat, lon, alt) + offset_ecef)
    np.testing.assert_allclose(rotation @ offset_ecef, expected, atol=MAX_ERROR_M)


def test_covariance_rotation_is_identity_at_origin():
    frame = LocalEnuFrame(49.0, 8.0, 100.0)
    covariance = np.diag([1.0, 4.0, 9.0])
    np.testing.assert_allclose(
        frame.rotate_covariance_from_fix(covariance, 49.0, 8.0), covariance, atol=1e-12
    )


def test_covariance_rotation_preserves_eigenvalues_and_rotates_axes():
    frame = LocalEnuFrame(0.0, 0.0, 0.0)
    covariance = np.diag([1.0, 4.0, 9.0])
    # 90 degrees east on the equator: local East is the origin's -Up, local Up is the origin's East.
    rotated = frame.rotate_covariance_from_fix(covariance, 0.0, 90.0)
    np.testing.assert_allclose(np.linalg.eigvalsh(rotated), [1.0, 4.0, 9.0], atol=1e-9)
    np.testing.assert_allclose(rotated, np.diag([9.0, 4.0, 1.0]), atol=1e-9)


def test_frame_rejects_invalid_origin():
    with pytest.raises(ValueError):
        LocalEnuFrame(91.0, 0.0, 0.0)
    with pytest.raises(ValueError):
        LocalEnuFrame(float("nan"), 0.0, 0.0)
    with pytest.raises(ValueError):
        LocalEnuFrame(0.0, float("nan"), 0.0)


def test_measurements_are_immutable():
    imu = ImuMeasurement(0.0, [0.0, 0.0, 9.81], [0.0, 0.0, 0.0])
    with pytest.raises(AttributeError):
        imu.timestamp = 1.0
    with pytest.raises(ValueError):
        imu.accel_mps2[0] = 1.0


def test_measurements_validate_shape_and_symmetry():
    with pytest.raises(ValueError):
        ImuMeasurement(0.0, [0.0, 0.0], [0.0, 0.0, 0.0])
    asymmetric = np.eye(3)
    asymmetric[0, 1] = 1.0
    with pytest.raises(ValueError):
        GnssMeasurement(0.0, 49.0, 8.0, 100.0, asymmetric)


def test_denied_measurement_has_no_fix():
    nan = float("nan")
    denied = GnssMeasurement(0.0, nan, nan, nan, np.eye(3), GnssStatus.DENIED)
    assert not denied.has_fix
    assert GnssMeasurement(0.0, 49.0, 8.0, 100.0, np.eye(3)).has_fix
