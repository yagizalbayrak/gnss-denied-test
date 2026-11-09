from __future__ import annotations

import importlib.util
import io
import sys
import zipfile
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parents[1] / "data" / "download_sample.py"
_spec = importlib.util.spec_from_file_location("download_sample", _SCRIPT)
download_sample = importlib.util.module_from_spec(_spec)
sys.modules["download_sample"] = download_sample
_spec.loader.exec_module(download_sample)


class InMemoryRangeFile(download_sample.HttpRangeFile):
    """Serves ranges from bytes, recording requests so that selectivity can be asserted."""

    def __init__(self, payload: bytes, read_ahead: int) -> None:
        self.payload = payload
        super().__init__("memory://archive", read_ahead=read_ahead)

    def _fetch_size(self) -> int:
        return len(self.payload)

    def _fetch_range(self, start: int, end: int) -> bytes:
        return self.payload[start:end]


def _archive_bytes() -> bytes:
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_STORED) as archive:
        archive.writestr("day/drive/oxts/data/0.txt", b"oxts-0")
        archive.writestr("day/drive/oxts/data/1.txt", b"oxts-1")
        archive.writestr("day/drive/velodyne/0.bin", bytes(1_000_000))
    return buffer.getvalue()


def test_range_file_matches_real_reads_across_buffer_boundaries():
    payload = bytes(range(256)) * 100
    remote = InMemoryRangeFile(payload, read_ahead=64)
    assert remote.read(10) == payload[:10]
    assert remote.read(200) == payload[10:210]
    remote.seek(-5, 2)
    assert remote.read(100) == payload[-5:]
    assert remote.read(1) == b""
    remote.seek(100)
    assert remote.read() == payload[100:]


def test_extracts_only_selected_members_without_fetching_the_rest(tmp_path):
    remote = InMemoryRangeFile(_archive_bytes(), read_ahead=1024)
    with zipfile.ZipFile(remote) as archive:
        names = download_sample.select_members(archive.namelist(), ["day/drive/oxts/"])
        download_sample.extract_members(archive, names, tmp_path)

    assert (tmp_path / "day/drive/oxts/data/0.txt").read_bytes() == b"oxts-0"
    assert (tmp_path / "day/drive/oxts/data/1.txt").read_bytes() == b"oxts-1"
    assert not (tmp_path / "day/drive/velodyne").exists()
    assert remote.bytes_fetched < 100_000  # The 1 MB LiDAR member stayed on the server.
    assert not list(tmp_path.rglob("*.part"))


def test_rerun_skips_finished_files(tmp_path):
    remote = InMemoryRangeFile(_archive_bytes(), read_ahead=1024)
    with zipfile.ZipFile(remote) as archive:
        names = download_sample.select_members(archive.namelist(), ["day/drive/oxts/"])
        download_sample.extract_members(archive, names, tmp_path)
        fetched_after_first_run = remote.bytes_fetched
        download_sample.extract_members(archive, names, tmp_path)
    assert remote.bytes_fetched == fetched_after_first_run


def test_corrupt_member_leaves_no_file_behind(tmp_path):
    payload = bytearray(_archive_bytes())
    payload[payload.index(b"oxts-0")] ^= 0xFF
    remote = InMemoryRangeFile(bytes(payload), read_ahead=1024)
    with zipfile.ZipFile(remote) as archive:
        with pytest.raises(zipfile.BadZipFile):
            download_sample.extract_members(archive, ["day/drive/oxts/data/0.txt"], tmp_path)
    assert not (tmp_path / "day/drive/oxts/data/0.txt").exists()


def test_selection_rejects_paths_that_escape_the_output_dir():
    names = ["day/ok.txt", "day/../../evil.txt", "/day/abs.txt", "day/subdir/", "other/no.txt"]
    assert download_sample.select_members(names, ["day/"]) == ["day/ok.txt"]
