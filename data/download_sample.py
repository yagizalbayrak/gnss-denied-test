"""Fetch the parts of a KITTI Raw drive that this project uses, without the full archives.

The official archives also hold LiDAR and colour images, which we do not need. HTTP range requests
let us read each ZIP's central directory and pull only the selected members:

    extract.zip  oxts/              100 Hz inertial and RTK stream
    sync.zip     image_00, image_01 10 Hz rectified grayscale stereo pair
    calib.zip    calib_*.txt        camera and IMU calibration

Usage:
    python data/download_sample.py --dry-run
    python data/download_sample.py

Finished files are skipped and a file only takes its final name once its CRC has been verified, so
an interrupted run can simply be restarted. KITTI is licensed CC BY-NC-SA 3.0 for non-commercial
use.
"""

from __future__ import annotations

import argparse
import io
import os
import shutil
import sys
import time
import urllib.error
import urllib.request
import zipfile
from collections.abc import Iterable
from pathlib import Path, PurePosixPath

BASE_URL = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "kitti_raw"
READ_AHEAD_BYTES = 16 * 1024 * 1024
MAX_ATTEMPTS = 5
TIMEOUT_S = 60


class HttpRangeFile(io.RawIOBase):
    """Seekable read-only view of a remote file, with read-ahead for sequential ZIP access."""

    def __init__(self, url: str, read_ahead: int = READ_AHEAD_BYTES) -> None:
        super().__init__()
        self.url = url
        self.read_ahead = read_ahead
        self.bytes_fetched = 0
        self._position = 0
        self._buffer = b""
        self._buffer_start = 0
        self._size = self._fetch_size()

    def _with_retries(self, operation):
        for attempt in range(MAX_ATTEMPTS):
            try:
                return operation()
            except (urllib.error.URLError, TimeoutError, ConnectionError, OSError) as error:
                if attempt == MAX_ATTEMPTS - 1:
                    raise
                delay_s = 2**attempt
                print(f"  retrying in {delay_s}s after: {error}", file=sys.stderr)
                time.sleep(delay_s)
        raise AssertionError("unreachable")

    def _fetch_size(self) -> int:
        def head() -> int:
            request = urllib.request.Request(self.url, method="HEAD")
            with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
                return int(response.headers["Content-Length"])

        return self._with_retries(head)

    def _fetch_range(self, start: int, end: int) -> bytes:
        def get() -> bytes:
            headers = {"Range": f"bytes={start}-{end - 1}"}
            request = urllib.request.Request(self.url, headers=headers)
            with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
                payload = response.read()
            if response.status != 206 or len(payload) != end - start:
                raise ConnectionError(f"bad range response for {start}-{end}: {response.status}")
            return payload

        return self._with_retries(get)

    def readable(self) -> bool:
        return True

    def seekable(self) -> bool:
        return True

    def tell(self) -> int:
        return self._position

    def seek(self, offset: int, whence: int = os.SEEK_SET) -> int:
        origin = {os.SEEK_SET: 0, os.SEEK_CUR: self._position, os.SEEK_END: self._size}[whence]
        self._position = origin + offset
        return self._position

    def read(self, size: int = -1) -> bytes:
        remaining = self._size - self._position
        size = remaining if size is None or size < 0 else min(size, remaining)
        chunks = []
        while size > 0:
            offset = self._position - self._buffer_start
            if not 0 <= offset < len(self._buffer):
                end = min(self._position + max(size, self.read_ahead), self._size)
                self._buffer = self._fetch_range(self._position, end)
                self.bytes_fetched += len(self._buffer)
                self._buffer_start = self._position
                offset = 0
            chunk = self._buffer[offset : offset + size]
            chunks.append(chunk)
            self._position += len(chunk)
            size -= len(chunk)
        return b"".join(chunks)


def select_members(names: Iterable[str], prefixes: Iterable[str]) -> list[str]:
    """Keep regular files under `prefixes`, refusing any path that could escape the output dir."""
    prefixes = tuple(prefixes)
    selected = []
    for name in names:
        path = PurePosixPath(name)
        if name.endswith("/") or path.is_absolute() or ".." in path.parts:
            continue
        if name.startswith(prefixes):
            selected.append(name)
    return selected


def extract_members(archive: zipfile.ZipFile, names: list[str], output: Path) -> None:
    """Extract in on-disk order, so the read-ahead buffer serves most requests."""
    infos = sorted((archive.getinfo(name) for name in names), key=lambda info: info.header_offset)
    for index, info in enumerate(infos, start=1):
        target = output / info.filename
        if target.exists() and target.stat().st_size == info.file_size:
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        partial = target.with_name(target.name + ".part")
        with archive.open(info) as source, open(partial, "wb") as destination:
            shutil.copyfileobj(source, destination)  # ZipFile verifies the CRC at end of member.
        partial.replace(target)
        if index % 500 == 0 or index == len(infos):
            print(f"  {index}/{len(infos)} files", flush=True)


def archive_plan(date: str, drive: str, cameras: Iterable[int]) -> list[tuple[str, list[str]]]:
    drive_name = f"{date}_drive_{drive}"
    drive_url = f"{BASE_URL}/{drive_name}/{drive_name}"
    sync_prefixes = [f"{date}/{drive_name}_sync/image_{camera:02d}/" for camera in cameras]
    return [
        (f"{BASE_URL}/{date}_calib.zip", [f"{date}/"]),
        (f"{drive_url}_extract.zip", [f"{date}/{drive_name}_extract/oxts/"]),
        (f"{drive_url}_sync.zip", sync_prefixes),
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--date", default="2011_09_30")
    parser.add_argument("--drive", default="0018")
    parser.add_argument("--cameras", type=int, nargs="+", default=[0, 1])
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--dry-run", action="store_true", help="report sizes without downloading members"
    )
    args = parser.parse_args()

    total_bytes = 0
    for url, prefixes in archive_plan(args.date, args.drive, args.cameras):
        with HttpRangeFile(url) as remote, zipfile.ZipFile(remote) as archive:
            names = select_members(archive.namelist(), prefixes)
            if not names:
                raise SystemExit(f"nothing matches {prefixes} in {url}")
            size = sum(archive.getinfo(name).compress_size for name in names)
            total_bytes += size
            print(f"{url.rsplit('/', 1)[-1]}: {len(names)} files, {size / 1e9:.3f} GB")
            for prefix in prefixes:
                matched = [n for n in names if n.startswith(prefix)]
                matched_bytes = sum(archive.getinfo(n).compress_size for n in matched)
                print(f"  {prefix}: {len(matched)} files, {matched_bytes / 1e9:.3f} GB")
            if not args.dry_run:
                extract_members(archive, names, args.output)

    verb = "would fetch" if args.dry_run else "fetched"
    print(f"Total {verb}: {total_bytes / 1e9:.3f} GB -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
