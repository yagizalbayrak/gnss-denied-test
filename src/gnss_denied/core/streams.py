"""Merging of independently sampled sensor streams into one chronological stream."""

from __future__ import annotations

import heapq
from collections.abc import Iterable, Iterator
from typing import Protocol, TypeVar


class Timestamped(Protocol):
    @property
    def timestamp(self) -> float: ...


PacketT = TypeVar("PacketT", bound=Timestamped)


def merge_streams(*streams: Iterable[PacketT]) -> Iterator[PacketT]:
    """Lazily merge streams that are each sorted by time.

    Ties keep the argument order, so passing the IMU first means a frame or a GNSS fix sharing a
    timestamp with an IMU sample is seen after the motion leading up to it.
    """
    return heapq.merge(*streams, key=lambda packet: packet.timestamp)
