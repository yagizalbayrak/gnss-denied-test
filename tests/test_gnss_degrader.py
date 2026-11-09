from __future__ import annotations

import numpy as np
import pytest

from gnss_denied.core.geodesy import LocalEnuFrame
from gnss_denied.core.types import GnssStatus, GroundTruthSample
from gnss_denied.simulator.gnss_degrader import (
    DegradationMode,
    DegradationWindow,
    GnssDegrader,
    GnssDegraderConfig,
    load_degrader_config,
)

ORIGIN = (49.0, 8.4, 110.0)
FRAME = LocalEnuFrame(*ORIGIN)


def _truth(duration_s: float = 120.0, rate_hz: float = 100.0) -> list[GroundTruthSample]:
    """A stationary vehicle, so every measured offset comes from the degrader."""
    count = int(duration_s * rate_hz) + 1
    return [
        GroundTruthSample(i / rate_hz, *ORIGIN, 0.0, 0.0, 0.0, [0.0, 0.0, 0.0])
        for i in range(count)
    ]


def _run(*windows: DegradationWindow, **overrides):
    config = GnssDegraderConfig(windows=windows, **overrides)
    return list(GnssDegrader(config).degrade(_truth()))


def _offset(measurement) -> np.ndarray:
    return FRAME.from_geodetic(measurement.lat_deg, measurement.lon_deg, measurement.alt_m)


def test_downsamples_to_receiver_rate():
    timestamps = [m.timestamp for m in _run()]
    np.testing.assert_allclose(timestamps, np.arange(121.0), atol=1e-9)


def test_nominal_noise_matches_configured_sigmas():
    offsets = np.array([_offset(m) for m in _run()])
    assert abs(offsets[:, 0].std() - 0.5) < 0.15
    assert abs(offsets[:, 2].std() - 1.0) < 0.3


def test_blackout_denies_exactly_the_closed_window():
    window = DegradationWindow(DegradationMode.BLACKOUT, 30.0, 90.0)
    measurements = _run(window)
    denied = [m.timestamp for m in measurements if m.status is GnssStatus.DENIED]
    np.testing.assert_allclose(denied, np.arange(30.0, 91.0), atol=1e-9)
    for measurement in measurements:
        assert measurement.has_fix == (measurement.status is not GnssStatus.DENIED)
        if not measurement.has_fix:
            assert np.isnan(measurement.lat_deg)
            assert np.isinf(np.diag(measurement.covariance_enu_m2)).all()


def test_windows_do_not_change_noise_outside_them():
    baseline = _run()
    degraded = _run(
        DegradationWindow(DegradationMode.BLACKOUT, 30.0, 40.0),
        DegradationWindow(DegradationMode.URBAN_CANYON, 50.0, 60.0),
    )
    for before, after in zip(baseline, degraded, strict=True):
        if after.timestamp < 30.0 or 40.0 < after.timestamp < 50.0 or after.timestamp > 60.0:
            assert before.lat_deg == after.lat_deg


def test_seed_makes_runs_reproducible():
    window = DegradationWindow(DegradationMode.URBAN_CANYON, 10.0, 100.0)
    first, second = _run(window, seed=7), _run(window, seed=7)
    assert [m.lat_deg for m in first] == [m.lat_deg for m in second]
    assert [m.lat_deg for m in first] != [m.lat_deg for m in _run(window, seed=8)]


def test_urban_canyon_is_heavy_tailed_clipped_and_flagged():
    window = DegradationWindow(DegradationMode.URBAN_CANYON, 0.0, 120.0)
    measurements = _run(window, urban_max_jump_m=40.0, sigma_horizontal_m=0.01)
    offsets = np.array([_offset(m) for m in measurements])
    horizontal = np.abs(offsets[:, :2])
    assert horizontal.max() <= 40.0 + 0.1
    assert horizontal.max() > 20.0  # A Gaussian with this scale would practically never get here.
    assert all(m.status is GnssStatus.DEGRADED for m in measurements)
    nominal = np.diag([0.01**2, 0.01**2, 1.0])
    np.testing.assert_allclose(measurements[0].covariance_enu_m2, nominal * 25.0)


def test_spoofing_ramps_along_bearing_and_reports_valid():
    window = DegradationWindow(DegradationMode.SPOOFING, 30.0, 90.0)
    measurements = _run(
        window, spoof_drift_mps=2.0, spoof_bearing_deg=90.0, sigma_horizontal_m=1e-6
    )
    by_time = {round(m.timestamp): m for m in measurements}

    assert _offset(by_time[29])[0] == pytest.approx(0.0, abs=1e-3)
    assert _offset(by_time[30])[0] == pytest.approx(0.0, abs=1e-3)
    assert _offset(by_time[60])[0] == pytest.approx(60.0, abs=1e-3)
    assert _offset(by_time[90])[0] == pytest.approx(120.0, abs=1e-3)
    assert _offset(by_time[60])[1] == pytest.approx(0.0, abs=1e-3)
    # Once the attack ends, the position snaps back.
    assert _offset(by_time[91])[0] == pytest.approx(0.0, abs=1e-3)

    # The receiver is fooled: nothing in the packet reveals the attack.
    assert all(m.status is GnssStatus.VALID for m in measurements)
    np.testing.assert_allclose(
        by_time[60].covariance_enu_m2, by_time[10].covariance_enu_m2, atol=1e-15
    )


def test_gap_in_truth_does_not_emit_a_burst():
    truth = [s for s in _truth() if not 10.0 < s.timestamp < 20.0]
    measurements = list(GnssDegrader(GnssDegraderConfig()).degrade(truth))
    assert np.diff([m.timestamp for m in measurements]).min() > 0.99


def test_invalid_config_is_rejected():
    with pytest.raises(ValueError):
        DegradationWindow(DegradationMode.BLACKOUT, 10.0, 5.0)
    with pytest.raises(ValueError):
        GnssDegraderConfig(rate_hz=0.0)
    with pytest.raises(ValueError):
        GnssDegraderConfig(
            windows=(
                DegradationWindow(DegradationMode.BLACKOUT, 10.0, 20.0),
                DegradationWindow(DegradationMode.SPOOFING, 15.0, 25.0),
            )
        )


def test_default_config_matches_the_receipt_scenario():
    config = load_degrader_config("configs/default_sim.yaml")
    assert config.rate_hz == 1.0
    assert config.windows == (DegradationWindow(DegradationMode.BLACKOUT, 30.0, 90.0),)
