"""Geodetic transformations between WGS84, ECEF and a local ENU tangent plane.

The local frame is anchored at the first valid GNSS fix P0. `pyproj` handles WGS84 <-> ECEF and
`pymap3d` handles ECEF <-> ENU, so both legs stay on the same WGS84 ellipsoid.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pymap3d
from pyproj import Transformer

from gnss_denied.core.types import FloatArray

_WGS84_GEODETIC = "EPSG:4979"
_WGS84_ECEF = "EPSG:4978"

_GEODETIC_TO_ECEF = Transformer.from_crs(_WGS84_GEODETIC, _WGS84_ECEF, always_xy=True)
_ECEF_TO_GEODETIC = Transformer.from_crs(_WGS84_ECEF, _WGS84_GEODETIC, always_xy=True)

_ELL = pymap3d.Ellipsoid.from_name("wgs84")


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> FloatArray:
    """Convert WGS84 latitude, longitude and ellipsoidal height to ECEF metres."""
    x, y, z = _GEODETIC_TO_ECEF.transform(lon_deg, lat_deg, alt_m)
    return np.array([x, y, z], dtype=np.float64)


def ecef_to_geodetic(ecef_m: FloatArray) -> tuple[float, float, float]:
    """Convert ECEF metres to WGS84 (lat_deg, lon_deg, alt_m)."""
    lon_deg, lat_deg, alt_m = _ECEF_TO_GEODETIC.transform(ecef_m[0], ecef_m[1], ecef_m[2])
    return float(lat_deg), float(lon_deg), float(alt_m)


def ecef_to_enu_rotation(lat_deg: float, lon_deg: float) -> FloatArray:
    """Return R such that v_enu = R @ v_ecef at the given geodetic position.

    This is also the Jacobian of ENU position with respect to ECEF position, so covariances
    transform as C_enu = R @ C_ecef @ R.T.
    """
    lat, lon = np.radians(lat_deg), np.radians(lon_deg)
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    sin_lon, cos_lon = np.sin(lon), np.cos(lon)
    return np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class LocalEnuFrame:
    """Local tangent plane anchored at the geodetic origin P0."""

    origin_lat_deg: float
    origin_lon_deg: float
    origin_alt_m: float

    def __post_init__(self) -> None:
        if not -90.0 <= self.origin_lat_deg <= 90.0:
            raise ValueError(f"origin latitude out of range: {self.origin_lat_deg}")
        if not np.isfinite([self.origin_lon_deg, self.origin_alt_m]).all():
            raise ValueError("origin longitude and altitude must be finite")

    def from_geodetic(self, lat_deg: float, lon_deg: float, alt_m: float) -> FloatArray:
        """Convert a WGS84 position to ENU metres relative to P0."""
        return self.from_ecef(geodetic_to_ecef(lat_deg, lon_deg, alt_m))

    def to_geodetic(self, enu_m: FloatArray) -> tuple[float, float, float]:
        """Convert ENU metres relative to P0 to WGS84 (lat_deg, lon_deg, alt_m)."""
        return ecef_to_geodetic(self.to_ecef(enu_m))

    def from_ecef(self, ecef_m: FloatArray) -> FloatArray:
        """Convert an ECEF position to ENU metres relative to P0."""
        east, north, up = pymap3d.ecef2enu(
            ecef_m[0],
            ecef_m[1],
            ecef_m[2],
            self.origin_lat_deg,
            self.origin_lon_deg,
            self.origin_alt_m,
            ell=_ELL,
        )
        return np.array([east, north, up], dtype=np.float64)

    def to_ecef(self, enu_m: FloatArray) -> FloatArray:
        """Convert ENU metres relative to P0 to an ECEF position."""
        x, y, z = pymap3d.enu2ecef(
            enu_m[0],
            enu_m[1],
            enu_m[2],
            self.origin_lat_deg,
            self.origin_lon_deg,
            self.origin_alt_m,
            ell=_ELL,
        )
        return np.array([x, y, z], dtype=np.float64)

    def rotate_covariance_from_fix(
        self, covariance_enu_at_fix_m2: FloatArray, lat_deg: float, lon_deg: float
    ) -> FloatArray:
        """Re-express a covariance from the ENU axes at a fix into this frame's ENU axes.

        Tangent planes at different points are not parallel, so a covariance reported at the fix
        has to be rotated into the axes anchored at P0.
        """
        r_fix = ecef_to_enu_rotation(lat_deg, lon_deg)
        r_origin = ecef_to_enu_rotation(self.origin_lat_deg, self.origin_lon_deg)
        rotation = r_origin @ r_fix.T
        return rotation @ covariance_enu_at_fix_m2 @ rotation.T
