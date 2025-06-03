import time
import logging

from typing import Optional
from typing import List
from typing import Dict
from typing import Any
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R


from xarm.wrapper import XArmAPI

from robits.core.abc.camera import CameraBase
from robits.core.abc.robot import UnimanualRobot
from robits.core.abc.gripper import GripperBase
from robits.core.abc.audio import AudioBase

from robits.real.xarm.gripper import XArmGripper

logger = logging.getLogger(__name__)


class XArmLite6(UnimanualRobot):
    def __init__(
        self,
        robot_name: str,
        transform_robot_to_world: Sequence[Sequence[float]],
        gripper: Optional[GripperBase] = None,
        cameras: Optional[List[CameraBase]] = None,
        audio: Optional[AudioBase] = None,
        ip_addr: str = "192.168.1.123",
        **kwargs
    ):
        """
        :param robot_name: Name of the robot
        :param gripper: Optional a gripper instance
        :param cameras: A list of cameras
        :param audio: Optional audio backend
        :param ip_addr: The IP address of the robot
        """
        self._robot_name = robot_name
        self.transform_robot_to_world = np.asarray(transform_robot_to_world)
        self.cameras = cameras or []
        self.audio = audio
        self.ip_addr = ip_addr
        self.robot = self.connect_to_robot()
        self.gripper = gripper or XArmGripper(self.robot)

    def connect_to_robot(self) -> XArmAPI:
        logger.debug("Connecting to robot at %s", self.ip_addr)
        robot = XArmAPI(self.ip_addr)
        robot.connect()
        robot.motion_enable(enable=True)
        robot.set_mode(0)
        robot.set_state(0)
        logger.info("Connected to robot at %s", self.ip_addr)
        return robot

    @property
    def robot_name(self) -> str:
        return self._robot_name

    def get_proprioception_data(
        self, include_eef=True, include_gripper_obs=True
    ) -> Dict[str, Any]:

        obs = {}
        obs["timestamp"] = time.time()
        code, angles = self.robot.get_servo_angle(is_radian=True)
        if code == 0:
            # .. todo Check.
            obs["joint_positions"] = np.array(angles[:-1])
            obs["joint_velocities"] = np.zeros(6)
            obs["joint_forces"] = np.zeros(6)

        if include_gripper_obs and self.gripper:
            gripper_obs = self.gripper.get_obs()
            obs["gripper_open"] = self.gripper.is_open()
            obs["gripper_joint_positions"] = gripper_obs.get("finger_positions", [0.0])
            obs["gripper_touch_forces"] = None

        if include_eef:
            translation, orientation = self.robot.eef_pose()
            obs["gripper_pose"] = (translation, orientation)
            m = np.identity()
            m[:3, :3] = R.from_quat(orientation).as_matrix()
            m[:3, 3] = translation
            obs["gripper_matrix"] = m
        return obs

    @property
    def eef_pose(self):
        """
        Xarm uses position + RPY angles instead of quaternion.
        """
        code, pose = self.robot.get_position(is_radian=True)

        if code == 0:
            translation, orientation_xyz = pose
            orientation_quat = R.from_euler("xyz", orientation_xyz).as_quat()
            return translation, orientation_quat
        return None, None

    @property
    def eef_matrix(self):
        translation, orientation = self.eef_pose
        m = np.identity(4)
        m[:3, 3] = translation
        m[:3, :3] = R.from_quat(orientation).as_matrix()
        return m

    def get_info(self):
        info = {
            "robot_name": self.robot_name,
            "ip": self.ip_addr,
            "cameras": [c.get_info() for c in self.cameras],
            "gripper": self.gripper.get_info() if self.gripper else "",
        }
        return info
