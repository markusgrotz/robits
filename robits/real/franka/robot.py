from typing import Dict
from typing import List
from typing import Any
from typing import Optional
from typing import Sequence

import time
import numpy as np

import logging

from franky import Robot as FrankyRobot

from franky import RealtimeConfig
from franky import ControlException

from robits.core.data_model.action import CartesianAction

from robits.core.abc.camera import CameraBase
from robits.core.abc.robot import UnimanualRobot
from robits.core.abc.gripper import GripperBase
from robits.core.abc.audio import AudioBase


from robits.core.utils import check_bounds

from robits.core.abc.control import control_types
from robits.real.franka.control import FrankyControlManager

from robits.real.franka import DEFAULT_ROBOT_IP_ADDR

from robits.utils import system_utils

logger = logging.getLogger(__name__)


class Franka(UnimanualRobot):
    """
    Robot implementation for the Franka Panda robot using franky.

    This class provides a concrete implementation of the RoBits robot interface
    for controlling a physical Franka Panda robot arm. It establishes connection
    to the robot hardware and integrates with gripper and camera components.

    The implementation uses the franky library for low-level communication with
    the robot controller and exposes a high-level interface for robot control.
    """

    def __init__(
        self,
        robot_name: str,
        transform_robot_to_world: Sequence[Sequence[float]],
        gripper: Optional[GripperBase] = None,
        cameras: Optional[List[CameraBase]] = None,
        audio: Optional[AudioBase] = None,
        ip_addr: str = DEFAULT_ROBOT_IP_ADDR,
        dynamics_factor: float = 0.2,
        **kwargs
    ):
        """
        Initialize a Franka Panda robot controller.

        Establishes connection to the physical Franka Panda robot and initializes
        the control interface, gripper, and cameras.

        :param robot_name: Identifier for this robot instance
        :param gripper: Gripper implementation for the robot's end-effector
        :param cameras: List of camera instances for perception
        :param audio: Optional audio interface for sound feedback
        :param ip_addr: IP address of the robot (defaults to DEFAULT_ROBOT_IP_ADDR)
        :param dynamics_factor: Scape parameter for the relative dynamics of the robot, such as speed
        :param kwargs: Additional parameters passed to the control manager
        """
        self.gripper = gripper
        self.cameras = cameras
        self.audio = audio

        self.transform_robot_to_world = np.asarray(transform_robot_to_world)

        self.ip_addr = ip_addr

        self.robot = self.connect_to_robot()

        self.dynamics_factor = dynamics_factor
        self.control = FrankyControlManager(self.robot, **kwargs)
        self._robot_name = robot_name

    @property
    def robot_name(self) -> str:
        return self._robot_name

    def connect_to_robot(self) -> FrankyRobot:
        """
        Connect to the physical Franka robot sing the franky library.
        Configures real-time behavior based on system capabilities and
        sets up initial safety parameters.

        :returns: Initialized FrankyRobot instance ready for control
        :raises: ControlException if connection fails
        """
        if system_utils.has_rt_support():
            logger.info("Using RT support")
            rt_config = RealtimeConfig.Enforce
        else:
            logger.warning(
                "No RT support. This is not recommended. Please install a real-time kernel."
            )
            logger.warning(
                "[blink]It is highly recommended to use a real-time kernel.[/blink]"
            )
            rt_config = RealtimeConfig.Ignore

        robot = FrankyRobot(fci_hostname=self.ip_addr, realtime_config=rt_config)
        robot.recover_from_errors()

        robot.relative_dynamics_factor = self.dynamics_factor
        logger.info("robot's dynamics factor is %s", self.dynamics_factor)

        robot.set_collision_behavior([100] * 7, [100] * 7, [100] * 6, [100] * 6)

        return robot

    def get_proprioception_data(
        self, include_eef=True, include_gripper_obs=True
    ) -> Dict[str, Any]:
        joint_state = self.robot.current_joint_state
        obs: Dict[str, Any] = {}
        obs["timestamp"] = time.time()
        obs["joint_positions"] = joint_state.position
        obs["joint_velocities"] = joint_state.velocity
        obs["joint_forces"] = np.zeros_like(joint_state.position)

        if include_gripper_obs:

            if not self.gripper:
                logger.warning("Gripper is not connected")
                return obs

            gripper_obs = self.gripper.get_obs()
            obs["gripper_open"] = self.gripper.is_open()
            obs["gripper_touch_forces"] = None
            obs["gripper_joint_positions"] = gripper_obs["finger_positions"]

        if include_eef:
            cartesian_state = self.robot.current_cartesian_state
            robot_pose = cartesian_state.pose
            ee_pose = robot_pose.end_effector_pose

            obs["gripper_pose"] = (ee_pose.translation, ee_pose.quaternion)
            obs["gripper_matrix"] = ee_pose.matrix

        return obs

    @property
    def eef_pose(self):
        cartesian_state = self.robot.current_cartesian_state
        robot_pose = cartesian_state.pose
        ee_pose = robot_pose.end_effector_pose
        return ee_pose.translation, ee_pose.quaternion

    @property
    def eef_matrix(self):
        cartesian_state = self.robot.current_cartesian_state
        robot_pose = cartesian_state.pose
        return robot_pose.end_effector_pose.matrix

    def get_info(self):
        state = self.robot.state
        keys = [x for x in dir(state) if not x.startswith("_")]
        state = {k: getattr(state, str(k)) for k in keys}

        if self.gripper:
            state.update({"gripper": self.gripper.get_info()})
        state.update(
            {
                "ip": self.ip_addr,
                "cameras": [c.get_info() for c in self.cameras],
                "robot_name": self.robot_name,
            }
        )

        for k, v in state.items():
            if v is None:
                state[k] = ""
            if isinstance(v, (tuple, list, set)):
                # ..todo:: we need to convert cameras
                pass
            if not isinstance(v, (float, int, bool, str)):
                state[k] = str(v)

        return state

    @check_bounds()
    def control_arm(self, action: CartesianAction, auto_recover=True):
        """
        Control the robot arm to move to a specified Cartesian pose.

        Moves the robot's end-effector to the position and orientation
        specified in the CartesianAction. The movement is safety-checked
        using the check_bounds decorator.

        :param action: CartesianAction specifying target position and orientation
        :param auto_recover: Whether to automatically recover from errors
        :returns: True if movement succeeded, False otherwise
        """
        try:
            with self.control(control_types.cartesian) as ctrl:
                ctrl.update((action.position, action.quaternion))
        except ControlException as e:
            logger.warning("Exception while control robot %s", e)
            if auto_recover:
                self.robot.recover_from_errors()
                return False

        time.sleep(0.01)

        return True

    def control_hand(self, action: CartesianAction):
        if action.hand_open:
            self.gripper.open()
        else:
            self.gripper.close()
