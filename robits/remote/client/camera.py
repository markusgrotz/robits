from typing import Tuple
from typing import Dict
from typing import Any

import numpy as np

from robits.core.data_model.camera_capture import CameraData
from robits.core.abc.camera import CameraBase
from robits.remote.client.client_base import ZMQClient


class CameraZMQClient(CameraBase):
    """
    Camera client for ZMQ
    """

    def __init__(self, address="localhost", port=5060, **kwargs):
        self.client = ZMQClient(address, port)

    def get_camera_data(self) -> Tuple[CameraData, Dict[str, Any]]:
        result = self.client.call("get_camera_data")
        # cam = result["camera_data"]
        cam = result[0]
        return (
            CameraData(
                rgb_image=np.array(cam["rgb_image"], dtype=np.uint8),
                depth_image=np.array(cam["depth_image"], dtype=np.float32),
            ),
            {},
        )  # result["metadata"]

    @property
    def intrinsics(self) -> np.ndarray:
        return np.array(self.client.call("intrinsics"))

    @property
    def extrinsics(self) -> np.ndarray:
        return np.array(self.client.call("extrinsics"))

    @property
    def camera_name(self) -> str:
        return self.client.call("camera_name")

    def get_info(self) -> Dict[str, Any]:
        return self.client.call("get_info")
