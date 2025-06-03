from typing import Tuple
import logging

import numpy as np
from scipy.spatial.transform import Rotation as R

from robits.core.abc.control import ControllerBase
from robits.core.abc.control import ControlManager
from robits.core.abc.control import control_types

logger = logging.getLogger(__name__)


class URPositionControl(ControllerBase):
    """
    Joint position control for UR robots using the `urx` interface.
    """

    def __init__(self, robot_impl):
        super().__init__(control_types.position)
        self.robot = robot_impl

    def start_controller(self):
        pass

    def stop_controller(self):
        pass

    def update(self, joint_positions: np.ndarray, relative: bool = False) -> None:
        try:
            if relative:
                current_positions = self.robot.getj()
                joint_positions = np.array(current_positions) + np.array(
                    joint_positions
                )

            self.robot.movej(
                joint_positions.tolist(), acc=1.0, vel=1.0, wait=not self.asynchronous
            )
        except Exception as e:
            logger.warning("URX Joint motion failed: %s", e)


class URCartesianControl(ControllerBase):
    """
    Cartesian control for UR robots using `movel` with pose (x, y, z, rx, ry, rz).
    """

    def __init__(self, robot_impl):
        super().__init__(control_types.cartesian)
        self.robot = robot_impl

    def start_controller(self):
        pass

    def stop_controller(self):
        pass

    def update(self, pose: Tuple[np.ndarray, np.ndarray], relative=False):
        try:
            position, orientation = pose
            orientation = R.from_quat(orientation).as_euler("xyz")
            current_pose = self.robot.getl()

            if relative:
                target = [current_pose[i + 0] + position[i] for i in range(3)] + [
                    current_pose[i + 3] + orientation[i] for i in range(3)
                ]
            else:
                target = list(position) + list(orientation)

            # ..todo:: Transform to robot's coordinate system

            self.robot.movel(target, acc=0.25, vel=0.25, wait=not self.asynchronous)
        except Exception as e:
            logger.warning("URX Cartesian motion failed: %s", e)


class URControlManager(ControlManager):
    """
    Control manager for Universal Robots with joint and Cartesian support.
    """

    def __init__(self, robot, default_joint_positions, **kwargs):
        super().__init__()
        self.robot = robot
        self.default_joint_positions = default_joint_positions
        self.register_controller(URPositionControl(robot))
        self.register_controller(URCartesianControl(robot))

    def move_home(self):
        """
        Move the robot to its default home joint position.
        """
        with self(control_types.position) as ctrl:
            ctrl.update(self.default_joint_positions)

    def stop(self):
        """
        Stop control—basic for URX.
        """
        try:
            self.robot.stopl()
        except Exception:
            pass
        logger.info("UR control manager stopped.")
