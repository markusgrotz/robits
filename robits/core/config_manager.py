from typing import List
from typing import Dict
from typing import Any
from typing import Optional
from typing import Sequence

from abc import ABC
from abc import abstractmethod

import logging
import json
import os
from pathlib import Path
from functools import lru_cache

import importlib.util
from importlib.resources import files
from importlib.resources import as_file


from robits.core.config import BaseConfig
from robits.core.config import MainConfig
from robits.core.config import ConfigTypes
from robits.core.config import config_type_to_class

from robits_config import robot as robot_config_package
from robits_config import gripper as gripper_config_package
from robits_config import camera as camera_config_package
from robits_config import audio as audio_config_package
from robits_config import speech as speech_config_package
from robits_config import camera_data as camera_calibration_package

logger = logging.getLogger(__name__)


class ConfigFinder(ABC):

    @abstractmethod
    def find_config(self, config_name: str) -> Optional[Path]:
        """
        Get the full path of a JSON configuration file. If a config cannot be found, None is returned.
        :param config_name: Name of the config file (without extension).
        :return: Path object representing the full path to the JSON config file. None if not found
        """
        pass

    @abstractmethod
    def list(self) -> Sequence[str]:
        """
        Lists available configurations.

        :returns: A list of available configuration names.
        """
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """
        Name of the config finder
        """
        pass

    @property
    @abstractmethod
    def path(self) -> Path:
        """
        Root path the ConfigFinder operates in
        """
        pass


class PackageResourceFinder(ConfigFinder):
    """
    Searches for json configs inside a given package
    """

    def __init__(self, package) -> None:
        self.package = package

    def find_config(self, config_name: str) -> Optional[Path]:
        resource = f"{config_name}.json"
        try:
            resource_path = files(self.package).joinpath(resource)
            if resource_path.is_file():
                with as_file(resource_path) as path:
                    return path
        except ModuleNotFoundError:
            pass
        return None

    def list(self) -> List[str]:
        return [
            Path(s.name).stem
            for s in files(self.package).iterdir()
            if s.name.endswith(".json")
        ]

    @property
    def name(self) -> str:
        return self.package.__name__.rsplit(".")[-1]

    @property
    def path(self) -> Path:
        return Path(self.package.__path__[0])


class WorkspaceConfigFinder(ConfigFinder):
    """
    Searches for configs inside a folder
    """

    def __init__(self, config_dir) -> None:
        self.config_dir = config_dir
        if not self.config_dir.is_dir():
            raise ValueError(f"Invalid configuration directory: {self.config_dir}")

    def find_config(self, config_name: str) -> Optional[Path]:
        config_path = self.config_dir / f"{config_name}.json"
        if not config_path.exists():
            return None
        return config_path

    def list(self) -> List[str]:
        return [p.stem for p in self.config_dir.glob("*.json")]

    @property
    def name(self) -> str:
        return self.config_dir.name

    @property
    def path(self) -> Path:
        return self.config_dir


class ConfigManager:
    """
    Manages configuration files for different hardware components such as robots, grippers, cameras,
    audio backends, and speech backends. Provides methods to retrieve configuration paths, load configurations,
    and list available configurations.
    """

    def __init__(self):
        self.meta_path = []
        self.meta_path.append(WorkspaceConfigFinder(self.get_user_config_dir()))

        self.meta_path.append(PackageResourceFinder(robot_config_package))
        self.meta_path.append(PackageResourceFinder(gripper_config_package))
        self.meta_path.append(PackageResourceFinder(camera_config_package))
        self.meta_path.append(PackageResourceFinder(audio_config_package))
        self.meta_path.append(PackageResourceFinder(speech_config_package))
        self.meta_path.append(PackageResourceFinder(camera_calibration_package))

        from robits_config.additional_config import (
            remote_config as remote_config_package,
        )

        if all(
            [importlib.util.find_spec(d) for d in remote_config_package.dependencies]
        ):
            self.meta_path.append(PackageResourceFinder(remote_config_package))

        # from robits_config.additional_config import sim as sim_config_package
        # if all([importlib.util.find_spec(d) for d in sim_config_package.dependencies]):
        #    self.meta_path.append(PackageResourceFinder(sim_config_package))

    def get_user_config_dir(self) -> Path:
        """
        :returns: path where the user configuration is stored.
        """
        if env_config_dir := os.environ.get("ROBITS_CONFIG_DIR"):
            config_dir = Path(env_config_dir)
        else:
            config_dir = Path("/tmp/robits_config")
            logger.warning(
                "Unable to find a user configuration. Please set one with export ROBITS_CONFIG_DIR. Defaulting to %s.",
                config_dir,
            )

        config_dir = config_dir.resolve()

        os.makedirs(config_dir, exist_ok=True)
        if not config_dir.is_dir():
            raise ValueError(f"Invalid configuration directory: {config_dir}")
        return config_dir

    def get_main_config(self) -> MainConfig:
        if "main" in self.list():
            return MainConfig.from_dict(self.load_dict("main"))
        return MainConfig()

    def get_config_path(self, config_name: str) -> Path:
        """
        Retrieves the file path of a configuration file.

        :param config_name: The name of the configuration.
        :returns: The full path to the configuration file.
        """
        for finder in self.meta_path:
            if path := finder.find_config(config_name):
                return path
        # raise Exception("Configuration not found.")

    def load_dict(self, config_name: str) -> Dict[str, Any]:
        """
        Loads a configuration file as a dictionary.

        :param config_name: The name of the configuration.
        :returns: The configuration data.
        """
        for finder in self.meta_path:
            if path := finder.find_config(config_name):
                with path.open("r", encoding="utf-8") as file:
                    return json.load(file)

        logger.error("Unable to load configuration with name %s", config_name)
        raise FileNotFoundError("Unable to find configuration file")

    def from_dict(self, config_type: ConfigTypes, config: Dict[str, Any]) -> BaseConfig:
        """
        Converts a dictionary into a configuration object of the appropriate type.

        :param config_type: The type of configuration.
        :param config: The configuration dictionary.
        :returns: An instance of the appropriate configuration class.
        """
        cls = config_type_to_class[config_type]
        return cls.from_dict(config)

    @lru_cache()
    def list(self, unique=True) -> List[str]:
        """
        Lists all available configurations. User configurations can have the
        same name as system configurations. You can use the unique flag to
        remove duplicates

        :param unique: Whether duplicated configuration should be filtered.
        :returns: A list of available configuration names.
        """
        config_names = []
        for finder in self.meta_path:
            config_names.extend(finder.list())
        if unique:
            return list(dict.fromkeys(config_names))
        return config_names

    @property
    def available_robots(self) -> List[str]:
        """
        Retrieves a list of available robot configurations.

        :returns: Available robot configuration names.
        """
        return [n for n in self.list() if n.startswith("robot_")]

    @property
    def available_bimanual_robots(self) -> List[str]:
        """
        Retrieves a list of available bimanual robot configurations.

        :returns: Available bimanual robot configuration names.
        """
        return [n for n in self.list() if n.startswith("robot_") and "bimanual" in n]

    @property
    def available_grippers(self) -> List[str]:
        """
        Retrieves a list of available gripper configurations.

        :returns: Available gripper configuration names.
        """
        return [n for n in self.list() if n.startswith("gripper_")]

    @property
    def available_cameras(self) -> List[str]:
        """
        Retrieves a list of available camera configurations.

        :returns: Available camera configuration names.
        """
        return [n for n in self.list() if n.startswith("camera_")]

    @property
    def available_audio_backends(self) -> List[str]:
        """
        Retrieves a list of available audio backend configurations.

        :returns: Available audio backend configuration names.
        """
        return [n for n in self.list() if n.startswith("audio_")]

    @property
    def available_speech_backends(self) -> List[str]:
        """
        Retrieves a list of available speech backend configurations.

        :returns: Available speech backend configuration names.
        """
        return [n for n in self.list() if n.startswith("speech_")]

    @property
    def available_camera_calibrations(self) -> List[str]:
        """
        Retrieves a list of available camera calibrations.

        :returns: Available camera calibrations.
        """
        return [n for n in self.list() if n.startswith("calibration_")]

    @classmethod
    @lru_cache(maxsize=1)
    def get(cls) -> "ConfigManager":
        """
        Retrieves a singleton instance of the ConfigManager.

        :returns: A singleton instance of the class.
        """
        return ConfigManager()


config_manager = ConfigManager.get()
