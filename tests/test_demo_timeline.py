from __future__ import annotations

import numpy as np
import pytest

from gnss_denied.viz.demo_timeline import (
    DemoTimeline,
    Segment,
    interpolate_path,
    median_case_seed,
)


def _timeline() -> DemoTimeline:
    return DemoTimeline(10.0, [Segment(5.0, 15.0), Segment(15.0, 45.0)])


def test_drive_time_is_piecewise_linear_and_clamped():
    timeline = _timeline()
    assert timeline.drive_time(0.0) == 10.0
    assert timeline.drive_time(2.5) == 12.5
    assert timeline.drive_time(10.0) == 30.0
    assert timeline.drive_time(99.0) == 45.0
    assert timeline.duration_s == 15.0
    assert timeline.drive_span_s == (10.0, 45.0)


def test_playback_speed_follows_the_segment():
    timeline = _timeline()
    assert timeline.playback_speed(1.0) == 1.0
    assert timeline.playback_speed(6.0) == 3.0
    assert timeline.playback_speed(15.0) == 3.0


@pytest.mark.parametrize(
    "segments",
    [
        [],
        [Segment(5.0, 20.0), Segment(5.0, 30.0)],
        [Segment(5.0, 20.0), Segment(6.0, 20.0)],
    ],
)
def test_invalid_timelines_are_rejected(segments):
    with pytest.raises(ValueError):
        DemoTimeline(10.0, segments)


def test_median_case_seed_picks_neither_extreme():
    assert median_case_seed([40.0, 197.0, 100.0, 60.0, 130.0]) == 2
    assert median_case_seed([1.0, 2.0, 100.0, 104.0, 102.0, 200.0]) in (2, 4)


def test_interpolate_path_is_linear_between_epochs_and_keeps_the_jump_short():
    times = np.array([0.0, 1.0, 2.0])
    positions = np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 20.0, 2.0]])
    np.testing.assert_allclose(interpolate_path(times, positions, 0.5), [5.0, 0.0, 0.0])
    np.testing.assert_allclose(interpolate_path(times, positions, 1.5), [10.0, 10.0, 1.0])
