from typing import Tuple
from typing import Dict
from typing import Any

import logging
from functools import lru_cache

import numpy as np

from robits.core.abc.camera import CameraBase
from robits.core.data_model.camera_capture import CameraData

from robits.sim.blueprints import CameraBlueprint
from robits.sim.env_client import MujocoEnvClient
from robits.sim.env_design import env_designer

logger = logging.getLogger(__name__)


class MujocoCamera(CameraBase, MujocoEnvClient):
    """
    Implements a camera in Mujoco
    """

    def __init__(self, camera_name, width=640, height=480, **kwargs) -> None:
        """
        Initializes the camera
        """
        if width != 640 or height != 480:
            raise ValueError(
                "Different camera resolutions are currently not supported."
            )
        self._camera_name = camera_name
        self.width = width
        self.height = height
        env_designer.add(CameraBlueprint(camera_name))

    @property
    def camera_name(self) -> str:
        return self._camera_name

    def get_camera_data(self) -> Tuple[CameraData, Dict[str, Any]]:
        with self.env.image_lock:
            rgb_image = self.env.camera_data[f"{self.camera_name}_rgb"].copy()
            depth_image = self.env.camera_data[f"{self.camera_name}_depth"].copy()
            metadata = self.env.camera_data[f"{self.camera_name}_metadata"].copy()
        return CameraData(rgb_image, depth_image), metadata

    @property
    def extrinsics(self):
        c = self.env.data.cam(self.camera_name)
        extrinsics = np.identity(4)
        extrinsics[:3, :3] = c.xmat.reshape(3, 3)
        extrinsics[:3, 3] = c.xpos

        R_flip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        extrinsics[:3, :3] = R_flip @ extrinsics[:3, :3]
        return np.linalg.inv(extrinsics)

    @property
    @lru_cache(1)
    def intrinsics(self):
        c = self.env.model.cam(self.camera_name)

        fy = (self.height / 2.0) / np.tan(np.deg2rad(c.fovy / 2.0))
        fx = fy

        intrinsics = np.identity(3)
        intrinsics[0, 0] = fx
        intrinsics[1, 1] = fy
        intrinsics[0, 2] = self.width / 2.0
        intrinsics[1, 2] = self.height / 2.0

        return intrinsics
