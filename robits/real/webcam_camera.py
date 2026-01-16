"""
Requires pip install imageio[ffmpeg]

"""
from typing import Any
from typing import Dict
from typing import Tuple
from typing import List

import logging
import time

import numpy as np

import imageio.v3 as iio

from robits.core.abc.camera import CameraBase
from robits.core.data_model.camera_capture import CameraData
from robits.utils import vision_utils


logger = logging.getLogger(__name__)

class WebcamCamera(CameraBase):
    """
    Simple webcam implementation backed by imageio.
    """

    def __init__(
        self,
        camera_name: str = "webcam",
        width: int = 640,
        height: int = 480,
        hz: int = 30,
        source: str = "<video0>",
        **_: Any,
    ) -> None:
        self._camera_name = camera_name
        self.width = int(width)
        self.height = int(height)
        self.hz = int(hz)
        self.source = source

        """
        _metadata = iio.immeta(self.source)
        print(_metadata)
        {'plugin': 'ffmpeg', 'nframes': inf, 'ffmpeg_version': '7.1 built with Apple clang version 13.1.6 (clang-1316.0.21.2.5)', 'codec': 'rawvideo', 'pix_fmt': 'uyvy422', 'fps': 0, 'source_size': (1080, 1920), 'size': (1080, 1920), 'rotate': 0, 'duration': 0}        
        """
        self._iter = iio.imiter(self.source)
        self._seq = 0

        logger.warning("Unused parameter hz %d. Setting the fps is currently not supported.", hz)


    @classmethod
    def list_camera_info(cls) -> List[Tuple[str, str]]:
        found = []
        for i in range(2):
            uri = f"<video{i}>"
            try:
                if (metadata := iio.immeta(uri)) is not None:
                    found.append((metadata["plugin"], uri))
            except Exception:
                return found
        return found

    @property
    def camera_name(self) -> str:
        return self._camera_name

    def get_camera_data(self) -> Tuple[CameraData, Dict[str, Any]]:
        frame = next(self._iter)

        # Ensure HxWx3 uint8 RGB
        if frame.ndim == 2:
            frame = np.stack([frame] * 3, axis=-1)
        if frame.shape[-1] == 4:  # drop alpha if present
            frame = frame[..., :3]
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)

        h, w = frame.shape[:2]
        
        if self.width != w or self.height != h:
            if self._seq == 0:
                logger.warning("Image resolution differs from actual resoluation. Expected %d x %d. Actual %d x %d", self.width, self.height, w, h)
            frame = np.resize(frame, (self.height, self.width, 3))

        depth = np.zeros((h, w), dtype=np.float32)
        metadata: Dict[str, Any] = {
            "timestamp": time.time(),
            "seq": self._seq,
        }
        self._seq += 1

        return CameraData(rgb_image=frame, depth_image=depth), metadata

    @property
    def intrinsics(self) -> np.ndarray:
        fx = fy = 600.0
        return vision_utils.make_camera_intrinsics(fx, fy, self.width, self.height)

    @property
    def extrinsics(self) -> np.ndarray:
        # Identity pose by default (camera frame == world frame)
        return np.identity(4, dtype=np.float32)

    def get_info(self) -> Dict[str, Any]:
        info = super().get_info()
        info.update(
            {
                "backend": "imageio",
                "source": self.source,
                "width": self.width,
                "height": self.height,
                "hz": self.hz,
            }
        )
        return info

    def __del__(self) -> None:
        # The iterator will close its resources on GC; nothing explicit to do.
        self._iter = None  # type: ignore[assignment]

