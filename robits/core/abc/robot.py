from abc import ABC
from abc import abstractmethod

from typing import Dict
from typing import Any
from typing import List
from typing import Tuple
from typing import Optional
from typing import Union
from typing import Sequence

from functools import lru_cache

import logging


import numpy as np

from robits.core.abc.camera import CameraBase
from robits.core.abc.control import ControlManager
from robits.core.abc.control import BimanualControlManager
from robits.core.abc.gripper import GripperBase


logger = logging.getLogger(__name__)


class Perception:
    """
    Base class for handling multiple cameras
    """

    def __init__(self, cameras: List[CameraBase]):
        self.cameras = cameras

    def get_vision_data(
        self, include_point_cloud: bool = False, swap_channels: bool = False
    ) -> Dict[str, np.ndarray]:
        """
        Returns all the vision data
        """
        obs = {}

        for camera in self.cameras:
            camera_name = camera.camera_name

            camera_data, _medatadata = camera.get_camera_data()
            intrinsics = camera.intrinsics

            depth_image = camera_data.depth_image
            rgb_image = camera_data.rgb_image

            logger.debug("RGB image has shape %s", rgb_image.shape)

            obs[f"{camera_name}_camera_extrinsics"] = camera.extrinsics
            obs[f"{camera_name}_camera_intrinsics"] = intrinsics

            if swap_channels:
                # c x w x h
                obs[f"{camera_name}_rgb"] = rgb_image.transpose((2, 1, 0))
            else:
                # h x w x c
                obs[f"{camera_name}_rgb"] = rgb_image

            # h x w

            logger.debug("RGB image has shape %s", obs[f"{camera_name}_rgb"].shape)

            obs[f"{camera_name}_depth"] = depth_image

            logger.debug("Depth image has shape %s", obs[f"{camera_name}_depth"].shape)

            if include_point_cloud:
                from robits.utils import vision_utils

                pcd = vision_utils.depth_to_pcd(
                    camera_data, camera, apply_extrinsics=True
                )
                point_cloud = np.asarray(pcd.points)

                if swap_channels:
                    # c x h x w
                    obs[f"{camera_name}_point_cloud"] = point_cloud.transpose((2, 1, 0))
                else:
                    obs[f"{camera_name}_point_cloud"] = point_cloud

                logger.info(
                    "Point cloud has shape %s", obs[f"{camera_name}_point_cloud"].shape
                )

        return obs


class RobotBase(ABC):
    """
    Base class for robot arms with common functionality.

    Provides an interface for robot control and state access.
    """

    def __init__(
        self,
        transform_robot_to_world: Sequence[Sequence[float]],
        gripper: Optional[GripperBase] = None,
    ):
        self.transform_robot_to_world = np.asarray(transform_robot_to_world)
        self.gripper = gripper
        self.control: ControlManager

    @property
    @abstractmethod
    def robot_name(self) -> str:
        """
        Returns the name of the robot.

        :returns: The robot name
        """
        pass

    @abstractmethod
    def get_proprioception_data(
        self, include_eef: bool = True, include_gripper_obs: bool = True
    ) -> Dict[str, Any]:
        """
        Gets the proprioception data from a robot. This includes

         - joint position
         - joint velocities
         - gripper joint position (optional)
         - gripper pose (optional)

        :param include_eef: include pose of the end-effector
        :param include_gripper_obs: include joint positions and other proprioception of the gripper
        :returns: The proprioception data
        """
        pass

    @property
    @abstractmethod
    def eef_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        The pose of the end-effector as (position, quaternion).
        Quaternion format is xyzw

        :returns: The pose of the robot as tuple
        """
        pass

    @property
    @abstractmethod
    def eef_matrix(self) -> np.ndarray:
        """
        The 4x4 matrix of the current end-effector pose

        :returns: The pose of the robot as matrix
        """
        pass

    # @abstractmethod
    def get_info(self) -> Dict[str, str]:
        return {}

    @property
    @lru_cache(1)
    def transform_world_to_robot(self):
        """
        returns the transformation from the world to the robot
        """
        return np.linalg.inv(self.transform_robot_to_world)


class UnimanualRobot(Perception, RobotBase):
    def get_obs(self) -> Dict[str, Any]:
        """
        Returns the observation from the robot

        .. seealso:: Perception.get_vision_data()
        .. seealso:: RobotBase.get_proprioception_data()

        Also updates extrinsics parameters for wrist cameras
        """
        obs = {}

        obs.update(self.get_vision_data())
        obs.update(self.get_proprioception_data())

        gripper_matrix = obs["gripper_matrix"]
        m = np.linalg.inv(gripper_matrix)
        for camera in self.cameras:
            if camera.is_wrist_camera():
                extrinsics = obs[f"{camera.camera_name}_camera_extrinsics"]
                obs[f"{camera.camera_name}_camera_extrinsics"] = np.dot(
                    self.transform_robot_to_world, np.dot(extrinsics, m)
                )

        return obs


class BimanualRobot(Perception):
    def __init__(self, right_robot: RobotBase, left_robot: RobotBase, **kwargs):
        self.left_robot = left_robot
        self.right_robot = right_robot
        self.control: Union[ControlManager, BimanualControlManager]

        # self.control = BimanualControlManager(right_robot.control, left_robot.control)

    def get_obs(self) -> Dict[str, Any]:
        """
        Returns the observation from the robot

        .. seealso:: Perception.get_vision_data()
        .. seealso:: RobotBase.get_proprioception_data()
        """
        obs = {}
        obs.update(self.get_vision_data())
        obs.update(
            {
                f"right_{k}": v
                for k, v in self.right_robot.get_proprioception_data().items()
            }
        )
        obs.update(
            {
                f"left_{k}": v
                for k, v in self.left_robot.get_proprioception_data().items()
            }
        )

        for camera in self.cameras:
            if camera.is_wrist_camera():
                logger.warning("Wrist cameras are not implemented yet.")
        return obs


class DummyRobot(UnimanualRobot):
    """
    Mock class with dummy data
    """

    def __init__(
        self,
        gripper: Optional[GripperBase],
        cameras: List[CameraBase],
        audio=None,
        speech=None,
        **kwargs,
    ):
        self.gripper = gripper
        self.cameras = cameras or []
        self.audio = audio
        self.speech = speech
        self.transform_robot_to_world = np.identity(4)

    @property
    def robot_name(self) -> str:
        return "dummy"

    def get_proprioception_data(
        self, include_eef: bool = True, include_gripper_obs: bool = True
    ) -> Dict[str, Any]:
        return {"joint_positions": np.zeros(7), "gripper_matrix": np.identity(4)}

    @property
    def eef_pose(self):
        return np.zeros(3), np.array([0, 0, 0, 1])

    @property
    def eef_matrix(self):
        return np.identity(4)
