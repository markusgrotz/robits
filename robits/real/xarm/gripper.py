from typing import Dict
from typing import Any
from typing import Optional

import time
import logging
import threading
from xarm.wrapper import XArmAPI

from robits.core.abc.gripper import GripperBase


logger = logging.getLogger(__name__)


class XArmGripper(GripperBase):

    def __init__(
        self,
        gripper_name: str,
        xarm_api: Optional[XArmAPI] = None,
        ip_addr: Optional[str] = None,
        **kwargs
    ):
        """
        Initializes the gripper

        :param gripper_name: name of the gripper
        :param xarm_api: Optional, an instance to connect to the robot
        :param ip_addr: Optional, the IP address of the robot
        """
        self.lock = threading.Lock()
        self.ip_addr = ip_addr
        self.gripper = xarm_api or self.connect_to_robot()
        self.last_cmd_timestamp = 0
        self._gripper_name = gripper_name

        self.config = {
            "speed": kwargs.get("speed", 500),
            "open_pos": kwargs.get("open_pos", 850),
            "close_pos": kwargs.get("close_pos", 0),
        }

        logger.info(
            "Initialized gripper %s. Gripper normalized width is %.2f (%.2f of %.2f)",
            self._gripper_name,
            self.normalized_width,
            self.gripper.width,
            self.max_joint_position,
        )

    def connect_to_robot(self) -> XArmAPI:
        robot = XArmAPI(self.ip_addr)
        robot.connect()
        robot.motion_enable(enable=True)
        robot.set_mode(0)  # Position control
        robot.set_state(0)  # Ready state
        logger.info("Connected to xArm at %s", self.ip_addr)
        return robot

    @property
    def gripper_name(self):
        return self._gripper_name

    @classmethod
    def get_gripper_type_name(cls):
        return "xarm"

    @property
    def is_active(self):
        return self.lock.locked()

    @property
    def max_joint_position(self):
        return self.config["open_pos"]

    @property
    def normalized_width(self):
        # Normalized between 0 (closed) and 1 (fully open)
        _code, angles = self.gripper.get_servo_angle(is_radian=True)
        width = angles[-1]
        return width / self.max_joint_position

    def open(self):
        with self.lock:
            self.last_cmd_timestamp = time.time()
            logger.info("Opening gripper to position %d", self.config["open_pos"])
            self.gripper.open_lite6_gripper(sync=True)

    def close(self):
        with self.lock:
            self.last_cmd_timestamp = time.time()
            logger.info("Closing gripper to position %d", self.config["close_pos"])
            self.gripper.close_lite6_gripper(sync=True)

    def get_obs(self):
        return {"finger_positions": [self.normalized_width], "timestamp": time.time()}

    def is_open(self) -> bool:
        return self.normalized_width > 0.5

    def get_info(self) -> Dict[str, Any]:
        return {
            "gripper_config": self.config,
            "max_joint_position": self.max_joint_position,
        }

    def set_pos(self, pos):
        with self.lock:
            self.last_cmd_timestamp = time.time()
            pos = max(0, min(pos, self.max_joint_position))
            logger.info("Setting gripper to position %d", pos)
            raise NotImplementedError("Not implemented yet.")
