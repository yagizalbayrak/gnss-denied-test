"""Runs only where the pretrained weights are already cached, so the suite never downloads."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest
import torch

from gnss_denied.frontend.deep_features import AlikedLightGlueBackend

_CHECKPOINTS = Path(torch.hub.get_dir()) / "checkpoints"
pytestmark = pytest.mark.skipif(
    not (_CHECKPOINTS / "aliked-n16.pth").exists(), reason="ALIKED weights are not cached"
)


@pytest.fixture(scope="module")
def backend() -> AlikedLightGlueBackend:
    return AlikedLightGlueBackend(max_keypoints=512, device=torch.device("cpu"))


def _texture() -> np.ndarray:
    noise = (np.random.default_rng(0).random((240, 400)) * 255).astype(np.uint8)
    return cv2.GaussianBlur(noise, (7, 7), 0)


def test_recovers_a_known_image_shift(backend):
    image = _texture()
    shifted = np.roll(image, 6, axis=1)
    first, second = backend.extract(image), backend.extract(shifted)
    matches = backend.match(first, second)
    assert len(matches) > 30
    displacement = second.keypoints[matches[:, 1]] - first.keypoints[matches[:, 0]]
    assert np.median(displacement[:, 0]) == pytest.approx(6.0, abs=0.5)
    assert np.median(displacement[:, 1]) == pytest.approx(0.0, abs=0.5)


def test_blank_image_gives_no_matches(backend):
    blank = backend.extract(np.zeros((240, 400), dtype=np.uint8))
    other = backend.extract(_texture())
    assert backend.match(blank, other).shape[1] == 2
    assert len(backend.match(blank, other)) == 0
