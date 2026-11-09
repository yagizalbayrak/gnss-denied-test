"""Synthetic GNSS degradation: turns a ground truth stream into a realistic, failing receiver.

The dataset's RTK solution is treated as truth. Outside any window the degrader emits truth plus
Gaussian noise at the receiver rate. Inside a window it applies one failure mode:

    BLACKOUT      no fix at all (status DENIED).
    URBAN_CANYON  heavy-tailed Cauchy multipath jumps and an inflated covariance (status DEGRADED).
    SPOOFING      a growing position ramp. The status stays VALID and the covariance stays nominal,
                  because a spoofed receiver has no idea it is being lied to.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import numpy as np
import yaml

from gnss_denied.core.geodesy import LocalEnuFrame
from gnss_denied.core.types import FloatArray, GnssMeasurement, GnssStatus, GroundTruthSample

_TIME_EPSILON_S = 1e-9


class DegradationMode(Enum):
    BLACKOUT = "blackout"
    URBAN_CANYON = "urban_canyon"
    SPOOFING = "spoofing"


@dataclass(frozen=True)
class DegradationWindow:
    """One failure active for start_s <= t <= end_s on the dataset clock."""

    mode: DegradationMode
    start_s: float
    end_s: float

    def __post_init__(self) -> None:
        if self.end_s <= self.start_s:
            raise ValueError(f"window end must follow its start: {self.start_s}, {self.end_s}")

    def contains(self, timestamp: float) -> bool:
        return self.start_s <= timestamp <= self.end_s


@dataclass(frozen=True)
class GnssDegraderConfig:
    rate_hz: float = 1.0
    sigma_horizontal_m: float = 0.5
    sigma_vertical_m: float = 1.0
    urban_cauchy_scale_m: float = 5.0
    urban_max_jump_m: float = 100.0
    urban_covariance_inflation: float = 25.0
    spoof_drift_mps: float = 1.0
    spoof_bearing_deg: float = 90.0
    windows: tuple[DegradationWindow, ...] = ()
    seed: int = 0

    def __post_init__(self) -> None:
        if self.rate_hz <= 0.0:
            raise ValueError(f"rate_hz must be positive, got {self.rate_hz}")
        ordered = sorted(self.windows, key=lambda window: window.start_s)
        for earlier, later in zip(ordered, ordered[1:], strict=False):
            if later.start_s <= earlier.end_s:
                raise ValueError(f"degradation windows overlap: {earlier}, {later}")


def load_degrader_config(path: str | Path) -> GnssDegraderConfig:
    """Read the `gnss` section of a simulation YAML file."""
    with open(path, encoding="utf-8") as stream:
        section = dict(yaml.safe_load(stream)["gnss"])
    section["windows"] = tuple(
        DegradationWindow(DegradationMode(item["mode"]), item["start_s"], item["end_s"])
        for item in section.get("windows", ())
    )
    return GnssDegraderConfig(**section)


class GnssDegrader:
    def __init__(self, config: GnssDegraderConfig) -> None:
        self.config = config

    def degrade(self, truth: Iterable[GroundTruthSample]) -> Iterator[GnssMeasurement]:
        """Downsample `truth` to the receiver rate and apply the configured failures.

        Noise is drawn at every epoch, including denied ones, so adding or moving a window never
        changes the noise realisation of the epochs outside it.
        """
        rng = np.random.default_rng(self.config.seed)
        period_s = 1.0 / self.config.rate_hz
        next_epoch_s: float | None = None

        for sample in truth:
            if next_epoch_s is None:
                next_epoch_s = sample.timestamp
            if sample.timestamp < next_epoch_s - _TIME_EPSILON_S:
                continue
            while next_epoch_s <= sample.timestamp + _TIME_EPSILON_S:
                next_epoch_s += period_s

            gaussian = rng.standard_normal(3)
            cauchy = rng.standard_cauchy(2)
            yield self._measure(sample, gaussian, cauchy)

    def _active_window(self, timestamp: float) -> DegradationWindow | None:
        for window in self.config.windows:
            if window.contains(timestamp):
                return window
        return None

    def _measure(
        self, sample: GroundTruthSample, gaussian: FloatArray, cauchy: FloatArray
    ) -> GnssMeasurement:
        config = self.config
        sigmas = np.array(
            [config.sigma_horizontal_m, config.sigma_horizontal_m, config.sigma_vertical_m]
        )
        covariance = np.diag(sigmas**2)
        offset_enu = sigmas * gaussian
        status = GnssStatus.VALID

        window = self._active_window(sample.timestamp)
        if window is not None and window.mode is DegradationMode.BLACKOUT:
            nan = float("nan")
            no_information = np.diag([np.inf, np.inf, np.inf])
            return GnssMeasurement(
                sample.timestamp, nan, nan, nan, no_information, GnssStatus.DENIED
            )
        if window is not None and window.mode is DegradationMode.URBAN_CANYON:
            # Unclipped Cauchy samples can reach thousands of kilometres, which no receiver reports.
            jump = np.clip(
                config.urban_cauchy_scale_m * cauchy,
                -config.urban_max_jump_m,
                config.urban_max_jump_m,
            )
            offset_enu = offset_enu + np.array([jump[0], jump[1], 0.0])
            covariance = covariance * config.urban_covariance_inflation
            status = GnssStatus.DEGRADED
        if window is not None and window.mode is DegradationMode.SPOOFING:
            bearing = np.radians(config.spoof_bearing_deg)
            direction = np.array([np.sin(bearing), np.cos(bearing), 0.0])
            ramp_m = config.spoof_drift_mps * (sample.timestamp - window.start_s)
            offset_enu = offset_enu + ramp_m * direction

        frame_at_fix = LocalEnuFrame(sample.lat_deg, sample.lon_deg, sample.alt_m)
        lat_deg, lon_deg, alt_m = frame_at_fix.to_geodetic(offset_enu)
        return GnssMeasurement(sample.timestamp, lat_deg, lon_deg, alt_m, covariance, status)
