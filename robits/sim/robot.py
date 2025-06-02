from typing import Dict
from typing import List
from typing import Optional
from typing import Any
from typing import Sequence

import time
import logging
from functools import lru_cache

import numpy as np
from scipy.spatial.transform import Rotation as R

from robits.core.abc.camera import CameraBase
from robits.core.abc.robot import UnimanualRobot
from robits.core.abc.robot import BimanualRobot
from robits.core.abc.control import BimanualControlManager

from robits.utils.transform_utils import transform_pose

from robits.sim.control import MujocoControlManager
from robits.sim.env_client import MujocoJointControlClient
from robits.sim.env_client import MujocoEnvClient

from robits.sim.env_design import env_designer
from robits.sim.gripper import MujocoGripper

from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import RobotDescriptionModel
from robits.sim.blueprints import Attachment
from robits.sim.blueprints import Pose


logger = logging.getLogger(__name__)


class MujocoRobot(MujocoJointControlClient, UnimanualRobot):
    def __init__(
        self,
        robot_name: str,
        joint_names: List[str],
        default_joint_positions: List[float],
        transform_robot_to_world: Sequence[Sequence[float]],
        gripper: Optional[MujocoGripper],
        actuator_names: Optional[List[str]] = None,
        cameras: Optional[List[CameraBase]] = None,
        description_name: str = "",
        variant_name: Optional[str] = None,
        side_name: Optional[str] = None,
        **kwargs,
    ):
        """
        .. todo:: can we read the default joint positions from the home key?

        :param robot_name: name of the robot
        :param joint_names: list of the joint names in the model
        :param default_joint_positions: joint positions for the default/home position of the robot
        :param gripper: the gripper
        :param cameras: the cameras
        :param transform_robot_to_world: pose of the robot in the world frame.
        """
        super().__init__(joint_names, actuator_names)
        if side_name:
            self._robot_name = f"{side_name}_{robot_name}"
        else:
            self._robot_name = robot_name

        if (
            gripper
        ):  # update the joint names of the gripper since it is attached to the robot
            prefix = side_name or robot_name
            gripper.joint_names = [f"{prefix}/{j}" for j in gripper.joint_names]
            gripper.actuator_names = [f"{prefix}/{a}" for a in gripper.actuator_names]

        self.default_joint_positions = default_joint_positions
        self.gripper = gripper
        self.cameras = cameras or []
        self.transform_robot_to_world = np.asarray(transform_robot_to_world)

        attachment: Optional[Attachment] = None
        if gripper:
            wrist_pose = Pose()

            if quat := kwargs.get("wrist_quat", None):
                wrist_pose = wrist_pose.with_quat(np.fromstring(quat, sep=" "))

            if pos := kwargs.get("wrist_pos", None):
                wrist_pose = wrist_pose.with_position(np.fromstring(pos, sep=" "))

            attachment = Attachment(
                blueprint_id=f"gripperblueprint_{gripper.gripper_name}",
                wrist_name=kwargs.get("wrist_name", "wrist"),
                wrist_pose=wrist_pose,
                attachment_site=kwargs.get("attachment_site", "attachment_site"),
            )

        description = RobotDescriptionModel(description_name, variant_name, side_name)
        pose = Pose(self.transform_robot_to_world)
        env_designer.add(
            RobotBlueprint(
                self._robot_name, model=description, pose=pose, attachment=attachment
            )
        )

    @property
    def robot_name(self):
        return self._robot_name

    @property
    @lru_cache(1)
    def control(self):
        kwargs = {
            "joint_names": self.joint_names,
            "actuator_names": self.actuator_names,
            "site": self.site,
            "default_joint_positions": self.default_joint_positions,
            "transform_robot_to_world": self.transform_robot_to_world,
        }
        return MujocoControlManager(**kwargs)

    def get_proprioception_data(
        self, include_eef: bool = True, include_gripper_obs: bool = True
    ) -> Dict[str, Any]:
        obs: Dict[str, Any] = {}
        obs["timestamp"] = time.time()
        obs["joint_positions"] = self.data.qpos[self.joint_ids].copy()  # Filter
        obs["joint_velocities"] = self.data.qvel[self.joint_ids].copy()
        obs["joint_forces"] = np.zeros_like(obs["joint_positions"])

        if include_gripper_obs:
            if not self.gripper:
                logger.warning("Gripper is not connected")
            else:
                gripper_obs = self.gripper.get_obs()
                obs["gripper_open"] = self.gripper.is_open()
                obs["gripper_touch_forces"] = None
                obs["gripper_joint_positions"] = gripper_obs["finger_positions"]
        if include_eef:
            obs["gripper_pose"] = self.eef_pose
            obs["gripper_matrix"] = self.eef_matrix
        return obs

    @property
    @lru_cache(1)
    def site(self):
        """
        Heuristic to search for a site
        """
        if self.model.nsite == 1:
            logger.info("Found a single site")
            return self.data.site(0)

        logger.warning("Found multiple sites.")
        # TODO test if site is unique in the robot model ...
        robot_side = self.joint_names[-1].split("/")[0]

        for i in range(self.model.nsite):
            logger.info("%s", self.data.site(i).name)

        for i in range(self.model.nsite):
            site_name = self.data.site(i).name
            if robot_side in site_name and "gripper" in site_name:
                logger.warning("Choosing site %s", site_name)
                return self.data.site(i)

        for i in range(self.model.nsite):
            if robot_side in self.data.site(i).name:
                return self.data.site(i)
        raise ValueError("Unable to find a site")

    @property
    def eef_pose(self):
        pose = self.eef_pose_by_site(self.site)
        position, quaternion = pose[:3], pose[3:]
        return transform_pose(self.transform_world_to_robot, position, quaternion)

    @property
    def eef_matrix(self):
        return self.transform_world_to_robot @ self.eef_matrix_by_site(self.site)

    def get_info(self):
        info = {
            "cameras": [c.get_info() for c in self.cameras],
            "robot_name": self.robot_name,
        }
        if self.gripper:
            info.update({"gripper": self.gripper.get_info()})
        return info

    def eef_pose_by_site(self, site):
        """
        Convenience function
        """
        mat = site.xmat.reshape(3, 3)
        q = R.from_matrix(mat).as_quat()
        return np.concatenate([site.xpos, q], axis=0)

    def eef_matrix_by_site(self, site):
        """
        ..todo:: rename
        Convenience function
        """
        m = np.identity(4)
        m[:3, :3] = site.xmat.reshape(3, 3)
        m[:3, 3] = site.xpos
        return m


class BimanualMujocoRobot(BimanualRobot, MujocoEnvClient):
    def __init__(self, robot_name: str, right_robot, left_robot, **kwargs):
        """
        .. seealso:: MujocoRobot

        :param robot_name: name of the robot
        :param right_robot: instance of the right robot
        :param left_robot: instance of the left robot
        """
        self._robot_name = robot_name
        self.left_robot = left_robot
        self.right_robot = right_robot

    @property
    @lru_cache(1)
    def control(self):
        return BimanualControlManager(self.right_robot.control, self.left_robot.control)
