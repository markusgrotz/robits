from typing import Dict
from typing import List
from typing import Any
from typing import Optional
from typing import Sequence

import time
import logging

import numpy as np
from scipy.spatial.transform import Rotation as R

import urx

from robits.core.abc.camera import CameraBase
from robits.core.abc.robot import UnimanualRobot
from robits.core.abc.gripper import GripperBase

logger = logging.getLogger(__name__)


class URRobot(UnimanualRobot):
    """
    Robot implementation for Universal Robots using the `urx` library.
    """

    def __init__(
        self,
        robot_name: str,
        transform_robot_to_world: Sequence[Sequence[float]],
        gripper: Optional[GripperBase],
        cameras: Optional[List[CameraBase]],
        ip_addr: str = "192.168.0.1",
        **kwargs
    ):
        """
        :param robot_name: Name of the robot
        """

        self._robot_name = robot_name
        self.transform_robot_to_world = np.asarray(transform_robot_to_world)

        self.gripper = gripper
        self.cameras = cameras or []
        self.audio = kwargs.get("audio", None)
        self.ip_addr = ip_addr

        self.robot = self.connect_to_robot()

    def connect_to_robot(self) -> urx.Robot:
        logger.debug("Connecting to robot at %s", self.ip_addr)
        robot = urx.Robot(self.ip_addr)
        robot.set_tcp((0, 0, 0, 0, 0, 0))
        robot.set_payload(1.0)
        return robot

    @property
    def robot_name(self) -> str:
        return self._robot_name

    def get_proprioception_data(
        self, include_eef=True, include_gripper_obs=True
    ) -> Dict[str, Any]:
        obs = {
            "timestamp": time.time(),
            "joint_positions": self.robot.getj(),
            "joint_velocities": None,
            "joint_forces": None,
        }

        if include_gripper_obs and self.gripper:
            gripper_obs = self.gripper.get_obs()
            obs["gripper_open"] = self.gripper.is_open()
            obs["gripper_touch_forces"] = None
            obs["gripper_joint_positions"] = gripper_obs["finger_positions"]

        if include_eef:
            obs["gripper_pose"] = self.eef_pose
            obs["gripper_matrix"] = self.eef_matrix

        return obs

    @property
    def eef_pose(self):
        pose = self.robot.getl()
        translation, orientation_xyz = pose[:3], pose[3:]
        orientation_quat = R.from_euler("xyz", orientation_xyz).as_quat()
        return translation, orientation_quat

    @property
    def eef_matrix(self):
        translation, orientation = self.eef_pose
        m = np.identity(4)
        m[:3, 3] = translation
        m[:3, :3] = R.from_quat(orientation).as_matrix()
        return m
