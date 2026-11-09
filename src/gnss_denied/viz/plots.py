"""Static summary of a blackout run: a map of the trajectories and the error over time."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from gnss_denied.pipeline import FusionResult  # noqa: E402

_SURFACE = "#fcfcfb"
_TEXT_PRIMARY = "#0b0b0b"
_TEXT_SECONDARY = "#52514e"
_GRID = "#e4e3df"
_BLACKOUT_BAND = "#ecebe6"
_TRUTH = "#52514e"
_FUSED = "#2a78d6"
_DEAD_RECKONING = "#eb6834"
_SMOOTHED = "#1baf7a"
_ERROR_FLOOR_M = 0.05
# Fixed categorical order, validated all-pairs for colour vision deficiency.
_MODE_COLORS = ("#2a78d6", "#eb6834", "#1baf7a")


@dataclass(frozen=True)
class ModeSeries:
    """One benchmark mode: per-seed horizontal error over time, and the first seed's path."""

    times: np.ndarray
    errors_m: np.ndarray  # seeds x epochs
    path_enu_m: np.ndarray


def _style(axis) -> None:
    axis.set_facecolor(_SURFACE)
    axis.grid(color=_GRID, linewidth=0.8)
    axis.set_axisbelow(True)
    axis.tick_params(colors=_TEXT_SECONDARY, length=0)
    for side in ("top", "right", "left", "bottom"):
        axis.spines[side].set_visible(False)


def plot_blackout(
    result: FusionResult, blackout: tuple[float, float], title: str, path: str | Path
) -> None:
    times = np.array([step.timestamp for step in result.steps])
    truth = np.array([step.truth_enu_m for step in result.steps])
    fused = np.array([step.fused.position_enu_m for step in result.steps])
    reckoned = np.array([step.dead_reckoning.position_enu_m for step in result.steps])
    smoothed_times = np.array([state.timestamp for state in result.smoothed])
    smoothed = np.array(
        [
            state.position_enu_m - result.truth_enu_by_time[state.timestamp]
            for state in result.smoothed
        ]
    )

    figure, (map_axis, error_axis) = plt.subplots(
        1, 2, figsize=(13, 5.6), width_ratios=[1, 1.35], facecolor=_SURFACE
    )
    figure.suptitle(title, color=_TEXT_PRIMARY, fontsize=13, x=0.02, ha="left")

    _style(map_axis)
    map_axis.plot(reckoned[:, 0], reckoned[:, 1], color=_DEAD_RECKONING, lw=2, label="IMU only")
    map_axis.plot(fused[:, 0], fused[:, 1], color=_FUSED, lw=2, label="Fused (online)")
    map_axis.plot(truth[:, 0], truth[:, 1], color=_TRUTH, lw=2, ls=(0, (2, 1.5)), label="Truth")
    margin_m = 60.0
    map_axis.set_xlim(truth[:, 0].min() - margin_m, truth[:, 0].max() + margin_m)
    map_axis.set_ylim(truth[:, 1].min() - margin_m, truth[:, 1].max() + margin_m)
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.set_xlabel("East (m)", color=_TEXT_SECONDARY)
    map_axis.set_ylabel("North (m)", color=_TEXT_SECONDARY)
    map_axis.set_title("Trajectory", color=_TEXT_PRIMARY, loc="left", fontsize=11)
    map_axis.legend(frameon=False, labelcolor=_TEXT_SECONDARY, loc="best")

    _style(error_axis)
    error_axis.axvspan(*blackout, color=_BLACKOUT_BAND, zorder=0)
    series = (
        (times, reckoned - truth, _DEAD_RECKONING, "IMU only"),
        (times, fused - truth, _FUSED, "Fused (online)"),
        (smoothed_times, smoothed, _SMOOTHED, "Fused (smoothed)"),
    )
    for series_times, errors, color, label in series:
        horizontal = np.maximum(np.linalg.norm(errors[:, :2], axis=1), _ERROR_FLOOR_M)
        error_axis.plot(series_times, horizontal, color=color, lw=2, label=label)
    error_axis.set_yscale("log")
    error_axis.set_xlim(times[0], times[-1])
    error_axis.set_xlabel("Drive time (s)", color=_TEXT_SECONDARY)
    error_axis.set_ylabel("Horizontal error (m, log scale)", color=_TEXT_SECONDARY)
    error_axis.set_title("Error vs truth", color=_TEXT_PRIMARY, loc="left", fontsize=11)
    error_axis.text(
        np.mean(blackout),
        0.97,
        "GNSS blackout",
        transform=error_axis.get_xaxis_transform(),
        ha="center",
        va="top",
        color=_TEXT_SECONDARY,
    )
    error_axis.legend(
        frameon=False,
        labelcolor=_TEXT_SECONDARY,
        loc="lower right",
        bbox_to_anchor=(1.0, 1.0),
        ncols=3,
        borderaxespad=0.0,
    )

    figure.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150, facecolor=_SURFACE)
    plt.close(figure)


def plot_benchmark(
    series: dict[str, ModeSeries],
    truth_path: np.ndarray,
    blackout: tuple[float, float],
    title: str,
    path: str | Path,
) -> None:
    """Blackout paths on a map, and each mode's error as a median with its seed range."""
    figure, (map_axis, error_axis) = plt.subplots(
        1, 2, figsize=(13, 5.6), width_ratios=[1, 1.35], facecolor=_SURFACE
    )
    figure.suptitle(title, color=_TEXT_PRIMARY, fontsize=13, x=0.02, ha="left")

    _style(map_axis)
    _style(error_axis)
    error_axis.axvspan(*blackout, color=_BLACKOUT_BAND, zorder=0)
    first = next(iter(series.values()))
    in_blackout = (first.times >= blackout[0]) & (first.times <= blackout[1])
    for (mode, data), color in zip(series.items(), _MODE_COLORS, strict=True):
        map_axis.plot(*data.path_enu_m[in_blackout, :2].T, color=color, lw=2, label=mode)
        floored = np.maximum(data.errors_m, _ERROR_FLOOR_M)
        error_axis.fill_between(
            data.times, floored.min(axis=0), floored.max(axis=0), color=color, alpha=0.15, lw=0
        )
        error_axis.plot(data.times, np.median(floored, axis=0), color=color, lw=2, label=mode)
    truth_segment = truth_path[in_blackout, :2]
    map_axis.plot(*truth_segment.T, color=_TRUTH, lw=2, ls=(0, (2, 1.5)), label="Truth")
    margin_m = 30.0
    map_axis.set_xlim(truth_segment[:, 0].min() - margin_m, truth_segment[:, 0].max() + margin_m)
    map_axis.set_ylim(truth_segment[:, 1].min() - margin_m, truth_segment[:, 1].max() + margin_m)
    map_axis.set_aspect("equal", adjustable="box")
    map_axis.set_xlabel("East (m)", color=_TEXT_SECONDARY)
    map_axis.set_ylabel("North (m)", color=_TEXT_SECONDARY)
    map_axis.set_title(
        "Path during the blackout, seed 0", color=_TEXT_PRIMARY, loc="left", fontsize=11
    )
    map_axis.legend(frameon=False, labelcolor=_TEXT_SECONDARY, fontsize=8, loc="best")

    error_axis.set_yscale("log")
    error_axis.set_xlim(blackout[0] - 15.0, blackout[1] + 15.0)
    error_axis.set_xlabel("Drive time (s)", color=_TEXT_SECONDARY)
    error_axis.set_ylabel("Horizontal error (m, log scale)", color=_TEXT_SECONDARY)
    error_axis.set_title("Median and seed range", color=_TEXT_PRIMARY, loc="left", fontsize=11)
    error_axis.text(
        np.mean(blackout),
        0.97,
        "GNSS blackout",
        transform=error_axis.get_xaxis_transform(),
        ha="center",
        va="top",
        color=_TEXT_SECONDARY,
    )
    error_axis.legend(frameon=False, labelcolor=_TEXT_SECONDARY, fontsize=8, loc="lower left")

    figure.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, dpi=150, facecolor=_SURFACE)
    plt.close(figure)
