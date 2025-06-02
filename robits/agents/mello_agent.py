from typing import Optional
import logging
import time
import requests

from dataclasses import dataclass

import numpy as np


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class MelloConfig:

    max_delta: float = 0.25

    initial_joint_positions: Optional[np.ndarray] = None


class MelloAgent:

    def __init__(self, device_addr, robot):
        self.device_addr = device_addr
        self.robot = robot
        self.max_delta = 0.25

    def get_mello_data(self):
        data = requests.get(self.device_addr).json()
        joint_positions = np.array(data["joint_positions"])
        joint_positions = np.deg2rad(joint_positions)
        joint_positions *= -1

        joint_velocities = np.array(data["joint_velocities"])
        joint_velocities = np.deg2rad(joint_velocities)
        joint_velocities *= -1
        return joint_positions, joint_velocities

    def get_mello_joint_positions(self):
        joint_positions = np.array(
            requests.get(self.device_addr).json()["joint_positions"]
        )
        joint_positions = np.deg2rad(joint_positions)
        joint_positions *= -1
        return joint_positions

    def get_action(self) -> np.ndarray:
        mello_joint_positions = self.get_mello_joint_positions()
        robot_joint_positions = self.robot.get_proprioception_data(False, False)[
            "joint_positions"
        ]
        delta = mello_joint_positions - robot_joint_positions
        delta = np.clip(delta, -self.max_delta, self.max_delta)
        new_joint_positions = robot_joint_positions + delta
        return new_joint_positions

    def wait_for_pose(self):
        delta = np.ones_like(self.get_mello_joint_positions()) * self.max_delta
        while np.any(delta >= self.max_delta):
            mello_joint_positions = self.get_mello_joint_positions()
            robot_joint_positions = self.robot.get_joint_obs(False, False)[
                "joint_positions"
            ]
            delta = np.abs(mello_joint_positions - robot_joint_positions)
            logger.info(
                "Delta between robot and mello is too large %s. Should be with %s",
                np.rad2deg(delta),
                np.rad2deg(self.max_delta),
            )
            time.sleep(0.25)
        logger.info("Done.")
