from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from gnss_denied.core.streams import merge_streams
from gnss_denied.datasets.kitti_raw import KittiRawDrive, load_stereo_calibration

DATE = "2011_09_30"
DRIVE = "0018"


def _oxts_line(index: int) -> str:
    values = [float(index) + field / 100.0 for field in range(30)]
    return " ".join(str(value) for value in values)


def _write_stamps(path, stamps: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(stamps) + "\n", encoding="utf-8")


def _build_drive(root, oxts_stamps: list[str], frame_stamps: list[str]) -> None:
    extract = root / DATE / f"{DATE}_drive_{DRIVE}_extract" / "oxts"
    _write_stamps(extract / "timestamps.txt", oxts_stamps)
    (extract / "data").mkdir(parents=True)
    for index in range(len(oxts_stamps)):
        (extract / "data" / f"{index:010d}.txt").write_text(_oxts_line(index), encoding="utf-8")

    camera = root / DATE / f"{DATE}_drive_{DRIVE}_sync" / "image_00"
    _write_stamps(camera / "timestamps.txt", frame_stamps)
    (camera / "data").mkdir(parents=True)
    for index in range(len(frame_stamps)):
        (camera / "data" / f"{index:010d}.png").write_bytes(b"")


def _stamp(seconds: float) -> str:
    return f"2011-09-30 12:00:{seconds:012.9f}"


def test_maps_oxts_fields_to_the_right_quantities(tmp_path):
    _build_drive(tmp_path, [_stamp(1.0), _stamp(1.01)], [_stamp(1.0)])
    drive = KittiRawDrive(tmp_path, DATE, DRIVE)

    imu = next(drive.imu())
    np.testing.assert_allclose(imu.accel_mps2, [0.11, 0.12, 0.13])
    np.testing.assert_allclose(imu.gyro_radps, [0.17, 0.18, 0.19])

    truth = next(drive.ground_truth())
    assert (truth.lat_deg, truth.lon_deg, truth.alt_m) == (0.0, 0.01, 0.02)
    assert (truth.roll_rad, truth.pitch_rad, truth.yaw_rad) == (0.03, 0.04, 0.05)
    # Fields 6 and 7 are vn and ve, field 10 is vu; the output order is East, North, Up.
    np.testing.assert_allclose(truth.velocity_enu_mps, [0.07, 0.06, 0.10])


def test_timestamps_share_a_clock_starting_at_the_earliest_sample(tmp_path):
    _build_drive(tmp_path, [_stamp(2.0), _stamp(2.01)], [_stamp(1.5), _stamp(2.5)])
    drive = KittiRawDrive(tmp_path, DATE, DRIVE)
    assert [f.timestamp for f in drive.frames()] == pytest.approx([0.0, 1.0])
    assert [s.timestamp for s in drive.imu()] == pytest.approx([0.5, 0.51])


def test_drops_blank_and_truncated_packets(tmp_path):
    stamps = [_stamp(1.0), "", _stamp(1.02), _stamp(1.03), _stamp(1.04)]
    _build_drive(tmp_path, stamps, [_stamp(1.0)])
    oxts_dir = tmp_path / DATE / f"{DATE}_drive_{DRIVE}_extract" / "oxts" / "data"
    (oxts_dir / f"{3:010d}.txt").write_text("1.0 2.0", encoding="utf-8")

    drive = KittiRawDrive(tmp_path, DATE, DRIVE)
    assert [s.timestamp for s in drive.imu()] == pytest.approx([0.0, 0.02, 0.04])
    assert drive.dropped_oxts_samples == 2


def test_late_stamped_burst_is_dropped_and_good_packets_survive(tmp_path):
    # Same shape as drive 0018 index 14042: two packets stamped 60 ms late, then the clock recovers.
    seconds = [1.00, 1.07, 1.08, 1.03, 1.04, 1.05, 1.06, 1.09, 1.10]
    _build_drive(tmp_path, [_stamp(s) for s in seconds], [_stamp(1.0)])
    drive = KittiRawDrive(tmp_path, DATE, DRIVE)
    expected = [0.00, 0.03, 0.04, 0.05, 0.06, 0.09, 0.10]
    assert [s.timestamp for s in drive.imu()] == pytest.approx(expected)
    assert drive.dropped_oxts_samples == 2


def test_single_early_stamp_does_not_discard_what_came_before(tmp_path):
    seconds = [1.00, 1.01, 1.02, 0.50, 1.04, 1.05]
    _build_drive(tmp_path, [_stamp(s) for s in seconds], [_stamp(1.0)])
    drive = KittiRawDrive(tmp_path, DATE, DRIVE)
    assert [s.timestamp for s in drive.imu()] == pytest.approx([0.0, 0.01, 0.02, 0.04, 0.05])
    assert drive.dropped_oxts_samples == 1


def test_missing_dataset_says_how_to_fetch_it(tmp_path):
    with pytest.raises(FileNotFoundError, match="download_sample"):
        KittiRawDrive(tmp_path, DATE, DRIVE)


def test_missing_oxts_data_raises(tmp_path):
    _build_drive(tmp_path, [""], [_stamp(1.0)])
    with pytest.raises(ValueError):
        KittiRawDrive(tmp_path, DATE, DRIVE)


def test_merge_is_chronological_and_ties_keep_argument_order():
    def packets(name, stamps):
        return [SimpleNamespace(timestamp=stamp, name=name) for stamp in stamps]

    merged = list(
        merge_streams(
            packets("imu", [0.0, 0.1, 0.2]), packets("cam", [0.1]), packets("gnss", [0.0])
        )
    )
    assert [p.timestamp for p in merged] == [0.0, 0.0, 0.1, 0.1, 0.2]
    assert [p.name for p in merged] == ["imu", "gnss", "imu", "cam", "imu"]


def test_stereo_calibration_composes_the_transform_chain(tmp_path):
    date_dir = tmp_path / DATE
    date_dir.mkdir()
    (date_dir / "calib_cam_to_cam.txt").write_text(
        "calib_time: 09-Jan-2012 13:59:33\n"
        "R_rect_00: 1 0 0 0 1 0 0 0 1\n"
        "P_rect_00: 700 0 600 0 0 700 180 0 0 0 1 0\n"
        "P_rect_01: 700 0 600 -378 0 700 180 0 0 0 1 0\n",
        encoding="utf-8",
    )
    # A camera looking along the velodyne's x axis, and a velodyne 1 m above the IMU.
    (date_dir / "calib_velo_to_cam.txt").write_text(
        "R: 0 -1 0 0 0 -1 1 0 0\nT: 0 0 0\n", encoding="utf-8"
    )
    (date_dir / "calib_imu_to_velo.txt").write_text(
        "R: 1 0 0 0 1 0 0 0 1\nT: 0 0 -1\n", encoding="utf-8"
    )
    calibration = load_stereo_calibration(tmp_path, DATE)
    assert calibration.fx == 700.0 and calibration.cx == 600.0
    assert calibration.baseline_m == pytest.approx(0.54)
    T_body_camera = calibration.T_body_camera
    # The camera's forward axis (z) is the body's forward axis (x), and its right (x) is -y.
    np.testing.assert_allclose(T_body_camera[:3, :3] @ [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(
        T_body_camera[:3, :3] @ [1.0, 0.0, 0.0], [0.0, -1.0, 0.0], atol=1e-12
    )
    np.testing.assert_allclose(T_body_camera[:3, 3], [0.0, 0.0, 1.0], atol=1e-12)
