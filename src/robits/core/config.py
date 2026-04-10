from typing import Dict
from typing import Any
from typing import Optional
from typing import Sequence
from typing import TypeVar
from typing import Type

from abc import ABC

from enum import unique

from dataclasses import dataclass
from dataclasses import _MISSING_TYPE
from dataclasses import field

import json
from pathlib import Path

import numpy as np

from robits.core.compat import StrEnum

T = TypeVar("T", bound="BaseConfig")


@dataclass(frozen=True)
class BaseConfig(ABC):
    """
    A basic configuration.
    Additional parameters are stored in a kwargs dictionary.
    """

    def __init__(self) -> None:
        # contains all the data that is not mapped
        self.kwargs: Dict[str, Any]

    @classmethod
    def from_dict(cls: Type[T], data: Dict[str, Any]) -> T:
        """
        Initializes the class from a Dict
        """
        parameter_keys = cls.__dataclass_fields__.keys()
        default_parameters = cls.get_default_parameters()
        default_parameters.update(
            {k: v for k, v in data.items() if k in parameter_keys}
        )
        other_parameters = {k: v for k, v in data.items() if k not in parameter_keys}

        instance = cls(**default_parameters)
        instance.kwargs.update(other_parameters)

        return instance

    @classmethod
    def get_default_parameters(cls) -> Dict[str, Any]:
        return {
            k: v.default
            for k, v in cls.__dataclass_fields__.items()
            if not isinstance(v.default, _MISSING_TYPE)
        }

    def save_config(self, config_path) -> None:
        """
        Serializes the current state

        :param config_path: the path to serialize to
        """
        with open(config_path, "w") as f:
            json.dump(self.to_dict(), f, indent=4)

    def to_dict(self) -> Dict[str, Any]:
        """
        Converts the class to a dictionary

        :returns: the
        """
        data = self.__dict__.copy()
        data.update(data.pop("kwargs"))
        return data

    def to_json(self) -> str:
        """
        Serializes to JSON

        :returns: the JSON string
        """
        return json.dumps(self.to_dict(), indent=4)


@dataclass(frozen=True)
class MainConfig(BaseConfig):
    """
    A general configuration file.
    """

    min_scene_bounds: Optional[Sequence[float]] = field(
        default_factory=lambda: [0.2, -0.4, 0.30]
    )
    max_scene_bounds: Optional[Sequence[float]] = field(
        default_factory=lambda: [0.62, 0.4, 0.70]
    )

    default_cache_dir: Path = Path.home() / ".cache" / "robits"

    kwargs: Dict[str, Any] = field(default_factory=lambda: {})


@dataclass(frozen=True)
class CameraConfig(BaseConfig):
    """
    Configuration class for a camera.

    :param camera_name: Name of the camera.
    :param width: Image width in pixels (default: 640).
    :param height: Image height in pixels (default: 480).
    :param hz: Frame rate in Hertz (default: 30).
    :param rgb: Whether RGB images are enabled (default: True).
    :param depth: Whether depth images are enabled (default: True).
    :param point_cloud: Whether point cloud data is enabled (default: True).
    :param kwargs: Additional configuration parameters.
    """

    camera_name: str

    width: int = 640

    height: int = 480

    hz: int = 30

    rgb: bool = True
    depth: bool = True
    point_cloud: bool = True

    kwargs: Dict[str, Any] = field(default_factory=lambda: {})


@dataclass(frozen=True)
class RobotConfig(BaseConfig):
    """
    Configuration class for a robot.

    :param robot_name: Name of the robot.
    :param default_joint_positions: The joint positions for the home pose (default: None).
    :param joint_names: Names of the joints (default: None).
    :param gripper: Gripper configuration (default: None).
    :param cameras: List of camera configurations (default: None).
    :param transform_robot_to_world: 4x4 matrix of the root  in world coordinates
    :param kwargs: Additional configuration parameters.
    """

    robot_name: str

    default_joint_positions: Optional[Sequence[float]] = None

    joint_names: Optional[Sequence[str]] = None

    gripper: Optional[Any] = None

    cameras: Optional[Sequence[Any]] = None

    transform_robot_to_world: Sequence[Sequence[float]] = field(
        default_factory=lambda: np.identity(4).tolist()
    )

    kwargs: Dict[str, Any] = field(default_factory=lambda: {})


@dataclass(frozen=True)
class GripperConfig(BaseConfig):
    """
    Configuration class for a gripper.

    :param gripper_name: Name of the gripper.
    :param kwargs: Additional configuration parameters.
    """

    gripper_name: str

    kwargs: Dict[str, Any] = field(default_factory=lambda: {})


@dataclass(frozen=True)
class AudioConfig(BaseConfig):
    """
    Configuration class for audio settings.

    :param backend: Name of the audio backend
    :param kwargs: Additional configuration parameters.
    """

    audio_backend_name: str

    kwargs: Dict[str, Any] = field(default_factory=lambda: {})


@dataclass(frozen=True)
class SpeechConfig(BaseConfig):
    """
    Configuration class for speech settings.

    :param cache_path: Path to the cache folder
    :param kwargs: Additional configuration parameters.
    """

    cache_path: str = ""

    kwargs: Dict[str, Any] = field(default_factory=lambda: {})


@dataclass(frozen=True)
class CameraCalibration(BaseConfig):
    """
    Configuration class for camera calibration

    :param camera_name: Name of the camera
    :param date_update: Date when the calibration file has been updated.
    :param extrinsics: The camera extrinsics as 4x4 matrix
    :param intrinsics: The camera intrinsics as 3x3 matrix
    :param width: width of the image size
    :param height: height of the image size
    :param kwargs: Additional configuration parameters.
    """

    camera_name: str
    date_updated: Optional[str] = None

    extrinsics: np.ndarray = field(default_factory=lambda: np.identity(4))
    intrinsics: np.ndarray = field(default_factory=lambda: np.identity(3))

    width: int = 640
    height: int = 480

    kwargs: Dict[str, Any] = field(default_factory=lambda: {})


@unique
class ConfigTypes(StrEnum):
    """
    Represents the different config types
    """

    main = "main"
    robot = "robot"
    gripper = "gripper"
    camera = "camera"
    audio = "audio"
    speech = "speech"
    camera_calibration = "calibration"  # .. todo:: name camera_calibration


""" Maps the config type to the class """
config_type_to_class = {
    ConfigTypes.main: MainConfig,
    ConfigTypes.robot: RobotConfig,
    ConfigTypes.gripper: GripperConfig,
    ConfigTypes.camera: CameraConfig,
    ConfigTypes.audio: AudioConfig,
    ConfigTypes.speech: SpeechConfig,
    ConfigTypes.camera_calibration: CameraCalibration,
}
