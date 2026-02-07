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

        logger.info(
            "Loading %s config with name %s", self.config_type, self.config_name
        )
        self.config_dict = config_manager.load_dict(self.config_name)

    @abstractmethod
    def build(self) -> Any:
        """
        Builds an instance
        """
        pass

    def build_instance(self) -> Any:
        """
        Dynamically imports and constructs a class instance from a configuration dictionary.

        :param config_type: The type of component to build.
        :param config_dict: The configuration dictionary.
        :return: An instance of the specified class.
        """
        if "class_path" not in self.config_dict:
            logger.error("Unable to determine class path.")
            return None

        class_path = self.config_dict["class_path"]
        config = config_manager.from_dict(self.config_type, self.config_dict)

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
        if (config_name := self.config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            self.config_dict[self.config_type] = AudioFactory(config_name).build()

        return self.build_instance()


class SpeechFactory(BaseFactory):
    def __init__(self, config_name) -> None:
        super().__init__(ConfigTypes.speech, config_name)

    def build(self) -> SpeechBase:
        if (config_name := self.config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            self.config_dict[self.config_type] = SpeechFactory(config_name).build()

        return self.build_instance()


class GripperFactory(BaseFactory):
    def __init__(self, config_name: str) -> None:
        super().__init__(ConfigTypes.gripper, config_name)

    def build(self) -> GripperBase:
        if (config_name := self.config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            self.config_dict[self.config_type] = GripperFactory(config_name).build()

        return self.build_instance()


class CameraFactory(BaseFactory):
    def __init__(self, config_name: str) -> None:
        super().__init__(ConfigTypes.camera, config_name)

    def build(self) -> CameraBase:
        if (config_name := self.config_dict.get(self.config_type, None)) and isinstance(
            config_name, str
        ):
            self.config_dict[self.config_type] = CameraFactory(config_name).build()

        return self.build_instance()


class RobotFactory(BaseFactory):
    def __init__(self, config_name: str, prefix: str = "") -> None:
        super().__init__(ConfigTypes.robot, config_name)
        self.prefix = prefix

    def build(self) -> RobotType:
        config_dict = self.config_dict
        for side_name in ["left", "right"]:
            arm_side = f"{side_name}_robot"
            if arm_side in config_dict:
                config_name = config_dict[arm_side]
                config_dict[arm_side] = RobotFactory(config_name, side_name).build()

        for config_type, factory_cls in config_factories.items():
            type_name = str(config_type.value)
            if type_name in config_dict:
                if config_name := config_dict.get(type_name, None):
                    if isinstance(config_name, str):
                        factory = factory_cls(config_name)
                        for key in ["joint_names", "actuator_names", "gripper_name"]:
                            if self.prefix and (
                                value := factory.config_dict.get(key, None)
                            ):
                                prefix = (
                                    f"{self.prefix}_{self.config_dict['robot_name']}"
                                )
                                if isinstance(value, str):
                                    factory.config_dict[key] = f"{prefix}/{value}"
                                else:
                                    factory.config_dict[key] = [
                                        f"{prefix}/{e}" for e in value
                                    ]
                        config_dict[type_name] = factory.build()
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
        return self.build_instance()


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
