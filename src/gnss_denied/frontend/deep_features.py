"""Learned front-end: ALIKED keypoints matched by LightGlue, both from `kornia`.

`kornia` ships LightGlue's weights for SuperPoint but not the SuperPoint network, and those weights
are for non-commercial use. ALIKED is available end to end and is the closest replacement.
"""

from __future__ import annotations

import kornia.feature as KF
import numpy as np
import torch
from numpy.typing import NDArray

from gnss_denied.frontend.visual_odom import Features


def best_device() -> torch.device:
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


class AlikedLightGlueBackend:
    name = "aliked"

    def __init__(
        self,
        max_keypoints: int = 2048,
        detection_threshold: float = 0.2,
        device: torch.device | None = None,
    ) -> None:
        self._device = device or best_device()
        self._extractor = KF.ALIKED.from_pretrained(
            "aliked-n16",
            max_num_keypoints=max_keypoints,
            detection_threshold=detection_threshold,
            device=self._device,
        )
        self._matcher = KF.LightGlue("aliked").eval().to(self._device)
        self._image_size: tuple[int, int] | None = None

    @torch.inference_mode()
    def extract(self, image: np.ndarray) -> Features:
        """Detect on an 8-bit grayscale image. All images of a run must have the same size."""
        self._image_size = (image.shape[1], image.shape[0])
        tensor = torch.from_numpy(image).float().div(255.0)[None, None].to(self._device)
        features = self._extractor(tensor)[0]
        return Features(
            features.keypoints.cpu().numpy().astype(np.float32),
            features.descriptors.cpu().numpy(),
        )

    @torch.inference_mode()
    def match(self, first: Features, second: Features) -> NDArray[np.int64]:
        if len(first.keypoints) == 0 or len(second.keypoints) == 0:
            return np.empty((0, 2), dtype=np.int64)
        size = torch.tensor([self._image_size], device=self._device)
        output = self._matcher(
            {
                "image0": self._matcher_input(first, size),
                "image1": self._matcher_input(second, size),
            }
        )
        return output["matches"][0].cpu().numpy().astype(np.int64).reshape(-1, 2)

    def _matcher_input(self, features: Features, size: torch.Tensor) -> dict[str, torch.Tensor]:
        return {
            "keypoints": torch.from_numpy(features.keypoints)[None].to(self._device),
            "descriptors": torch.from_numpy(features.descriptors)[None].to(self._device),
            "image_size": size,
        }
