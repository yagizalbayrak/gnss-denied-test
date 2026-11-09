from __future__ import annotations

import gtsam
import numpy as np
import pytest

from gnss_denied.viz.rerun_logger import covariance_ellipsoid


def test_ellipsoid_half_sizes_are_three_sigma():
    half_sizes, _ = covariance_ellipsoid(np.diag([1.0, 4.0, 9.0]))
    assert half_sizes == pytest.approx([3.0, 6.0, 9.0])


def test_ellipsoid_rotation_reconstructs_the_covariance():
    rotation = gtsam.Rot3.Ypr(0.5, -0.2, 0.3).matrix()
    covariance = rotation @ np.diag([1.0, 4.0, 9.0]) @ rotation.T
    half_sizes, xyzw = covariance_ellipsoid(covariance)
    recovered = gtsam.Rot3.Quaternion(xyzw[3], xyzw[0], xyzw[1], xyzw[2]).matrix()
    rebuilt = recovered @ np.diag((half_sizes / 3.0) ** 2) @ recovered.T
    np.testing.assert_allclose(rebuilt, covariance, atol=1e-9)
