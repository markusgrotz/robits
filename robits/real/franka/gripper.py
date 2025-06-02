from typing import Dict
from typing import Any

import time
import logging
import threading

from functools import lru_cache
from functools import wraps

from franky import Gripper as FrankyGripper

from robits.core.abc.gripper import GripperBase

from robits.real.franka import DEFAULT_ROBOT_IP_ADDR


logger = logging.getLogger(__name__)


def timed_cache(time_delta=0.5):

    def decorator(func):

        start_time = time.monotonic()
        previous_value = None

        @wraps(func)
        def wrapper(*args, **kwargs):

            nonlocal start_time
            nonlocal previous_value

            current_time = time.monotonic()

            if previous_value is None or (current_time - start_time) > time_delta:
                start_time = current_time
                previous_value = func(*args, **kwargs)

            return previous_value

        return wrapper

    return decorator


class FrankaGripper(GripperBase):

    def __init__(self, gripper_name: str, **kwargs):
        """
        Initializes the gripper
        """
        self.config = {"speed": 0.2, "force": 10.0, "epsilon_outer": 1.0}
        self.lock = threading.Lock()

        gripper_ip_addr = DEFAULT_ROBOT_IP_ADDR

        self.gripper = FrankyGripper(gripper_ip_addr)

        self.last_cmd_timestamp = 0

        max_joint_position = self.max_joint_position
        current_normalized_width = self.normalized_width

        logger.info(
            "Initialized gripper %s. Gripper normalized witdh is %.2f (%.2f of %.2f)",
            self.gripper,
            current_normalized_width,
            self.gripper.width,
            max_joint_position,
        )
        self._gripper_name = gripper_name

    @property
    def gripper_name(self):
        return self._gripper_name

    @classmethod
    def get_gripper_type_name(cls):
        return "franka"

    @property
    def is_active(self):
        return self.lock.locked()

    @property
    @timed_cache()
    def normalized_width(self):
        """
        Returns the normalized width of the gripper.
        Unfortunately, reading the gripper width is slow.
        It can take some milliseconds to read the gripper width making it impractical in loops.
        Hence, the last value is cached using a decorator:

        Reading the current gripper.width:

        46.8 ms ± 2.11 ms per loop (mean ± std. dev. of 7 runs, 100 loops each)

        """
        with self.lock:
            return self.gripper.width / self.max_joint_position

    @property
    @lru_cache(1)
    def max_joint_position(self):
        """
        Returns the maximum joint position of the gripper.
        Value is cached as reading the value takes a few milliseconds:

        gripper.max_width:

        45.8 ms ± 1.79 ms per loop (mean ± std. dev. of 7 runs, 100 loops each)

        """
        return self.gripper.max_width

    def open(self):
        speed = self.config["speed"]
        with self.lock:
            self.last_cmd_timestamp = time.time()
            self.gripper.open(speed)

    def close(self):
        with self.lock:
            self.last_cmd_timestamp = time.time()
            self.gripper.grasp(0.0, **self.config)

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
            self.gripper.move(pos, speed=self.config["speed"])
