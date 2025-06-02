from typing import Dict
from typing import Any
from typing import Optional
from typing import List
from typing import Union

import logging

from pathlib import Path

import numpy as np

from robits.core.abc.camera import CameraBase
from robits.core.abc.robot import UnimanualRobot

from robits.dataset.camera import DatasetCamera
from robits.dataset.io.reader import DatasetReader

from robits.core.abc.control import ControlManager
from robits.core.abc.control import ControllerBase
from robits.core.abc.control import control_types

from robits.core.utils import FrequencyTimer


logger = logging.getLogger(__name__)


class DatasetRobot(UnimanualRobot):
    """
    A robot that replays actions and sensor data from a dataset.
    """

    def __init__(
        self,
        cameras: Optional[Union[List[CameraBase], List[str]]] = None,
        *args,
        **kwargs
    ) -> None:
        """
        Initialize the DatasetRobot.

        :param cameras: Already loaded cameras or the name of the cameras
        :param args: Additional positional arguments.
        :param kwargs: Additional keyword arguments, expects 'config' containing 'task_folder'.
        """
        config = kwargs["config"]
        task_folder = Path(config["task_folder"])

        self.dataset = DatasetReader(task_folder).load()

        transform_robot_to_world = self.dataset.metadata["robot_info"](
            "transform_robot_to_world", np.identity(4)
        )
        self.transform_robot_to_world = np.asarray(transform_robot_to_world)

        self.len_obs = len(self.dataset.entries)
        self.seq = 0

        self.control = ControlManager()
        self.control.register_controller(ControllerBase(control_types.cartesian))
        self.control.register_controller(ControllerBase(control_types.position))

        if not cameras:  # ..todo:: should we automatically load cameras?
            self.cameras = []
            for camera_name in self.dataset.metadata["camera_names"]:
                self.cameras.append(DatasetCamera(camera_name, self.dataset))
        elif isinstance(cameras[0], str):
            self.cameras = [DatasetCamera(n, self.dataset) for n in cameras]
        else:
            self.cameras = cameras
        self.frequency_timer = FrequencyTimer(self.dataset.metadata["frequency"])

    def stop(self):
        """ """
        pass

    def get_proprioception_data(
        self, include_eef: bool = True, include_gripper_obs: bool = True
    ) -> Dict[str, Any]:
        """
        .. seealso:: :py:meth:`UnimanualRobot.get_proprioception_data`
        """
        self.seq += 1
        self.frequency_timer.wait_for_cycle()
        return self.dataset.entries[self.idx].proprioception

    @property
    def eef_pose(self):
        pose = self.dataset.entries[self.idx].proprioception["gripper_pose"]
        return pose[:3], pose[3:7]

    @property
    def eef_matrix(self):
        matrix = self.dataset.entries[self.idx].proprioception["gripper_matrix"]
        return np.asarray(matrix)

    @property
    def idx(self):
        """
        Get the current index in the dataset.

        :return: The index of the current entry.
        """
        return self.seq % self.len_obs
