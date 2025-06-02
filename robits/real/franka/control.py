"""
Control implementations for the Franka Panda robot.

This module provides concrete implementations of the control interfaces
for the Franka Panda robot, including position control and Cartesian control.
These controllers use the franky library to send commands to the robot hardware.
"""

from typing import Tuple
import logging

import numpy as np
from scipy.spatial.transform import Rotation as R

from franky import JointMotion
from franky import JointState
from franky import ControlException
from franky import Affine
from franky import CartesianMotion

from robits.core.abc.control import ControllerBase
from robits.core.abc.control import ControlManager
from robits.core.abc.control import control_types

logger = logging.getLogger(__name__)


class FrankyPositionControl(ControllerBase):
    """
    Position control for the Franka panda robot.

    :param robot_impl: The robot implementation to control.
    """

    def __init__(self, robot_impl):
        super().__init__(control_types.position)
        self.robot = robot_impl

    def start_controller(self):
        """
        Start the position controller.
        """
        pass

    def stop_controller(self):
        """
        Stop the position controller.
        """
        pass

    def update(self, joint_positions: np.ndarray, relative=False) -> None:
        """
        Update the joint positions of the robot.

        :param joint_positions: The target joint angles as a numpy array
        :param relative: If True, updates joint angles relative to current positions
        :raises ControlException: If the motion command fails

        Example:
            with robot.control(control_types.position) as ctrl:
                # Move each joint 0.1 radians from current position
                ctrl.update(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]), relative=True)
        """
        if relative:
            current_joint_positions = self.robot.current_joint_positions
            joint_positions = current_joint_positions + joint_positions

        motion = JointMotion(JointState(joint_positions))
        self.robot.move(motion, asynchronous=self.asynchronous)


class FrankyCartesianControl(ControllerBase):
    """
    Cartesian control for the Franka Panda robot.

    This controller allows moving the robot's end-effector in Cartesian space,
    specifying position and orientation. It supports both absolute and relative
    movements.

    :param robot_impl: The robot implementation to control
    """

    def __init__(self, robot_impl):
        super().__init__(control_types.cartesian)
        self.robot = robot_impl

    def start_controller(self):
        """
        Start the cartesian controller.
        """
        pass

    def stop_controller(self):
        """
        Stop the cartesian controller.
        """
        pass

    def update(self, pose: Tuple[np.ndarray, np.ndarray], relative=False):
        """
        Update the control target for the end-effector. Quaternions follow the xyzw format

        :param pose: A tuple containing the position and quaternion of the end effector.
        :param relative: If True, updates the pose relative to the current state.

        .. code-block:: python

            with robot.control(control_types.cartesian) as ctrl:

                ctrl.update((np.array([0.1, 0.2, 0.3]), np.array([0, 0, 0, 1])), relative=True)

        """
        assert len(pose) == 2

        if relative:
            position_delta, quaternion_delta = pose
            cartesian_state = self.robot.current_cartesian_state
            robot_pose = cartesian_state.pose
            ee_pose = robot_pose.end_effector_pose
            robot_position, robot_quaternion = ee_pose.translation, ee_pose.quaternion
            position = robot_position + position_delta
            quaternion = (
                R.from_quat(robot_quaternion) * R.from_quat(quaternion_delta)
            ).as_quat()
        else:
            position, quaternion = pose

        motion = CartesianMotion(Affine(position, quaternion))

        try:
            self.robot.move(motion, asynchronous=self.asynchronous)
        except ControlException as e:
            logger.warning("Error while controlling the robot. Exception was %s", e)
            if self.robot.recover_from_errors():
                logger.info("Recovered from errors")
            else:
                logger.error("Unable to recover from errors. Exception was %s", e)


class FrankyControlManager(ControlManager):
    """
    Manages control operations for the Franka Panda robot.

    :param robot: The robot instance to control.
    :param default_joint_positions: The default joint positions for homing.
    """

    def __init__(self, robot, default_joint_positions, **kwargs):
        super().__init__()
        self.robot = robot
        self.default_joint_positions = default_joint_positions
        self.register_controller(FrankyPositionControl(robot))
        self.register_controller(FrankyCartesianControl(robot))
        self.set_impedance_from_config("default")

    def set_joint_impedance(self, values):
        """
        Set the joint impedance values.

        :param values: List of impedance values for each joint.
        """
        if self.active_controller:
            logger.error("Robot is currently controlled. Please disable controllers.")
            return
        self.robot.set_joint_impedance(values)

    def set_impedance_from_config(self, name="default"):
        """
        Set impedance values from a predefined configuration.

        :param name: Name of the impedance configuration.
        """
        joint_impedance_values = {
            "disabled": [3000] * 7,
            "smooth": [10, 10, 5, 5, 5, 5, 5],
            "default": [100, 100, 100, 50, 50, 50, 50],
        }
        return self.set_joint_impedance(joint_impedance_values[name])

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
