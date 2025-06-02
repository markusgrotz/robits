from typing import Any
from typing import Dict
from typing import Union

import importlib
import logging

from abc import ABC
from abc import abstractmethod

from robits.core.abc.robot import UnimanualRobot
from robits.core.abc.robot import BimanualRobot
from robits.core.abc.gripper import GripperBase
from robits.core.abc.camera import CameraBase
from robits.core.abc.audio import AudioBase
from robits.core.abc.speech import SpeechBase

from robits.core.config_manager import config_manager
from robits.core.config import ConfigTypes

RobotType = Union[UnimanualRobot, BimanualRobot]

DeviceType = Union[RobotType, GripperBase, CameraBase, AudioBase, SpeechBase]


logger = logging.getLogger(__name__)


class BaseFactory(ABC):
    """
    Base class for common factory logic, including dynamic class instantiation.
    """

    def __init__(self, config_type: ConfigTypes, config_name: str) -> None:
        """
        :param config_type: Type of the configuration
        :param config_name: Name of the configuration
        """
        self.config_type = config_type
        self.config_name = config_name

    @abstractmethod
    def build(self) -> Any:
        """
        Builds an instance
        """
        pass

    def build_instance(self, config_dict: Dict[str, Any]) -> Any:
        """
        Dynamically imports and constructs a class instance from a configuration dictionary.

        :param config_type: The type of component to build.
        :param config_dict: The configuration dictionary.
        :return: An instance of the specified class.
        """

        if "class_path" not in config_dict:
            logger.error("Unable to determine class path.")
            return None

        class_path = config_dict["class_path"]
        config = config_manager.from_dict(self.config_type, config_dict)

        module_name, class_name = class_path.rsplit(".", 1)
        module = importlib.import_module(module_name)
        cls = getattr(module, class_name)
        args = config.__dict__.copy()
        args.update(args.pop("kwargs"))
        return cls(**args)

    def load_config_dict(self) -> Dict[str, Any]:
        """
        The config to build from

        :returns: the loaded configuration dict
        """
        return config_manager.load_dict(self.config_name)


class AudioFactory(BaseFactory):
    def __init__(self, config_name) -> None:
        super().__init__(ConfigTypes.audio, config_name)

    def build(self) -> AudioBase:
        logger.info("Building %s from config %s", self.config_type, self.config_name)
        config_dict = config_manager.load_dict(self.config_name)

        if (config_name := config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            config_dict[self.config_type] = AudioFactory(config_name).build()

        return self.build_instance(config_dict)


class SpeechFactory(BaseFactory):
    def __init__(self, config_name) -> None:
        super().__init__(ConfigTypes.speech, config_name)

    def build(self) -> SpeechBase:
        logger.info("Building %s from config %s", self.config_type, self.config_name)
        config_dict = config_manager.load_dict(self.config_name)

        if (config_name := config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            config_dict[self.config_type] = SpeechFactory(config_name).build()

        return self.build_instance(config_dict)


class GripperFactory(BaseFactory):
    def __init__(self, config_name) -> None:
        super().__init__(ConfigTypes.gripper, config_name)

    def build(self) -> GripperBase:
        logger.info("Building gripper from config %s", self.config_name)
        config_dict = config_manager.load_dict(self.config_name)

        if (config_name := config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            config_dict[self.config_type] = GripperFactory(config_name).build()

        return self.build_instance(config_dict)


class CameraFactory(BaseFactory):
    def __init__(self, config_name) -> None:
        super().__init__(ConfigTypes.camera, config_name)

    def build(self) -> CameraBase:
        logger.info("Building camera from config %s", self.config_name)
        config_dict = config_manager.load_dict(self.config_name)

        if (config_name := config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            config_dict[self.config_type] = CameraFactory(config_name).build()

        return self.build_instance(config_dict)


class RobotFactory(BaseFactory):

    def __init__(self, config_name) -> None:
        super().__init__(ConfigTypes.robot, config_name)

    def build(self) -> RobotType:
        logger.info("Building robot from config %s", self.config_name)
        config_dict = config_manager.load_dict(self.config_name)

        for arm_side in ["left_robot", "right_robot"]:
            if arm_side in config_dict:
                config_name = config_dict[arm_side]
                config_dict[arm_side] = RobotFactory(config_name).build()

        for config_type, factory_cls in config_factories.items():
            type_name = str(config_type.value)
            if type_name in config_dict:
                if config_name := config_dict.get(type_name, None):
                    if isinstance(config_name, str):
                        config_dict[type_name] = factory_cls(config_name).build()
                    else:
                        logger.warning("Already initialized.")

        cameras = []
        if "cameras" in config_dict:
            for camera_name in config_dict["cameras"]:
                cameras.append(CameraFactory(camera_name).build())
        elif ConfigTypes.camera in config_dict:
            cameras.append(config_dict[ConfigTypes.camera])
            config_dict.pop(ConfigTypes.camera)
        config_dict["cameras"] = cameras

        logger.info("Building robot with config %s", config_dict)
        return self.build_instance(config_dict)


config_factories = {
    ConfigTypes.audio: AudioFactory,
    ConfigTypes.camera: CameraFactory,
    ConfigTypes.gripper: GripperFactory,
    ConfigTypes.speech: SpeechFactory,
    ConfigTypes.robot: RobotFactory,
}


class RobitsFactory(BaseFactory):
    """
    .. todo:: we need to derive the config type
    """

    def build(self) -> DeviceType:
        factory = config_factories[self.config_type]
        return factory(self.config_name).build()
