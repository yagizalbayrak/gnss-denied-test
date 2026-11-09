"""Timeline and data helpers for the demo video, kept free of rendering so they can be tested."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from gnss_denied.core.types import FloatArray


@dataclass(frozen=True)
class Segment:
    """Playback from the end of the previous segment up to these video and drive times."""

    video_end_s: float
    drive_end_s: float


class DemoTimeline:
    """Piecewise-linear map from video time to drive time."""

    def __init__(self, drive_start_s: float, segments: Sequence[Segment]) -> None:
        if not segments:
            raise ValueError("a timeline needs at least one segment")
        self._video = np.array([0.0, *(s.video_end_s for s in segments)])
        self._drive = np.array([drive_start_s, *(s.drive_end_s for s in segments)])
        if np.any(np.diff(self._video) <= 0.0) or np.any(np.diff(self._drive) <= 0.0):
            raise ValueError("video and drive times must both increase")
        self._segments = list(segments)

    @property
    def duration_s(self) -> float:
        return float(self._video[-1])

    @property
    def drive_span_s(self) -> tuple[float, float]:
        return float(self._drive[0]), float(self._drive[-1])

    def _index(self, video_s: float) -> int:
        index = int(np.searchsorted(self._video, video_s, side="right")) - 1
        return min(max(index, 0), len(self._segments) - 1)

    def drive_time(self, video_s: float) -> float:
        return float(np.interp(video_s, self._video, self._drive))

    def playback_speed(self, video_s: float) -> float:
        index = self._index(video_s)
        drive_span = self._drive[index + 1] - self._drive[index]
        return float(drive_span / (self._video[index + 1] - self._video[index]))


def median_case_seed(end_errors_m: Sequence[float]) -> int:
    """Index of the run closest to the median, so a video shows neither the best nor worst case."""
    errors = np.asarray(end_errors_m, dtype=np.float64)
    return int(np.argmin(np.abs(errors - np.median(errors))))


def interpolate_path(times_s: FloatArray, positions_m: FloatArray, time_s: float) -> FloatArray:
    """Position at any time. Estimates exist once per second, and the video needs smooth motion."""
    return np.array([np.interp(time_s, times_s, positions_m[:, axis]) for axis in range(3)])
