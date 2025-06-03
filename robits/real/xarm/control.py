from typing import Tuple
import logging

import numpy as np
from scipy.spatial.transform import Rotation as R

from xarm.wrapper import XArmAPI

from robits.core.abc.control import ControllerBase
from robits.core.abc.control import ControlManager
from robits.core.abc.control import control_types

logger = logging.getLogger(__name__)


class XArmPositionControl(ControllerBase):
    """
    Joint position control for xArm Lite 6.
    """

    def __init__(self, robot: XArmAPI):
        super().__init__(control_types.position)
        self.robot = robot

    def start_controller(self):
        pass

    def stop_controller(self):
        pass

    def update(self, joint_positions: np.ndarray, relative=False):
        if relative:
            code, current = self.robot.get_servo_angle(is_radian=True)
            if code != 0:
                raise Exception("Failed to read joint angles")
            joint_positions = np.array(current) + joint_positions

        joint_positions = joint_positions.tolist()
        ret = self.robot.set_servo_angle(
            joint_positions, is_radian=True, wait=self.asynchronous
        )
        if ret != 0:
            raise Exception(f"xArm joint motion failed with code {ret}")


class XArmCartesianControl(ControllerBase):
    """
    Cartesian control for xArm Lite 6.
    """

    def __init__(self, robot: XArmAPI):
        super().__init__(control_types.cartesian)
        self.robot = robot

    def start_controller(self):
        pass

    def stop_controller(self):
        pass

    def update(self, pose: Tuple[np.ndarray, np.ndarray], relative=False):
        position, quaternion = pose

        if relative:
            robot_position, robot_quaternion = self.robot.eef_pose()
            new_pos = robot_position + position
            new_quat = R.from_quat(robot_quaternion) * R.from_quat(quaternion)
            new_rpy = new_quat.as_euler("xyz")
        else:
            new_pos = position
            new_rpy = R.from_quat(quaternion).as_euler("xyz")

        # ..todo:: Transform to robot's coordinate system

        pose_cmd = list(new_pos) + list(new_rpy)
        ret = self.robot.set_position(*pose_cmd, is_radian=True, wait=self.asynchronous)
        if ret != 0:
            raise Exception(f"xArm Cartesian motion failed with code {ret}")


class XArmControlManager(ControlManager):
    """
    Manages control operations for the XArm robot.

    :param robot: The robot instance to control.
    :param default_joint_positions: The default joint positions for homing.
    """

    def __init__(self, robot, default_joint_positions, **kwargs):
        super().__init__()
        self.robot = robot
        self.default_joint_positions = default_joint_positions
        self.register_controller(XArmPositionControl(robot))
        self.register_controller(XArmCartesianControl(robot))

    def move_home(self):
        """
        Move the robot to its home position.
        """
        with self(control_types.position) as ctrl:
            ctrl.update(self.default_joint_positions)

    def stop(self):
        """
        Stop the control manager. This is a default implementation
        """
        logger.info("Shutting down control manager.")
