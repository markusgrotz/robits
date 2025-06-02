from typing import Dict
from typing import Any
from typing import Tuple

from abc import ABC
from abc import abstractmethod

import os
from functools import lru_cache

import numpy as np

from robits.core.data_model.camera_capture import CameraData
from robits.core.config_manager import config_manager
from robits.core.config import CameraCalibration


class CameraBase(ABC):
    """
    A general class that models a camera
    """

    @property
    @abstractmethod
    def camera_name(self) -> str:
        """
        Name of the camera

        :returns: The camera name
        """
        pass

    def __str__(self) -> str:
        return f"{self.camera_name}"

    def is_wrist_camera(self) -> bool:
        """
        :returns: True if the camera is mounted to a robot
        """
        return False

    @abstractmethod
    def get_camera_data(self) -> Tuple[CameraData, Dict[str, Any]]:
        """
        Gets the camera images

        :returns: the camera images and metadata
        """
        pass

    @property
    @lru_cache()
    def calibration(self) -> CameraCalibration:
        """
        Loads the camera calibration. The camera calibration file must be available in the config folder. A custom config folder can be specified with the environment variable ROBITS_CONFIG_DIR.

        :returns: the camera calibration
        """
        resource = f"calibration_{self.camera_name}_camera"
        calibration = config_manager.load_dict(resource)

        if self.camera_name != calibration.pop("camera_name", None):
            raise ValueError("Invalid calibration")
        return CameraCalibration(camera_name=self.camera_name, **calibration)

    def save_calibration(self, config_folder) -> None:
        """
        Stores the camera calibration

        :param config_folder: Path to the top-level config folder
        """
        config_path = os.path.join(
            config_folder, f"calibration_{self.camera_name}_camera.json"
        )
        self.calibration.save_config(config_path)

    @property
    def extrinsics(self) -> np.ndarray:
        """
        Extrinsics parameters

        :returns: the extrinsic parameters of the camera as 4x4 matrix
        """
        return np.array(self.calibration.extrinsics)

    @property
    def intrinsics(self) -> np.ndarray:
        """
        Intrinsic parameters

        :returns: the intrinsic parameters of the camera as 3x3 matrix
        """
        return np.array(self.calibration.intrinsics)

    def get_info(self) -> Dict[str, Any]:
        """
        General information about the camera
        """
        return {"camera_name": self.camera_name}
