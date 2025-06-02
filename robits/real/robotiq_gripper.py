"""
Implementation for Robotiq grippers.

This module provides an implementation of the GripperBase interface for
Robotiq grippers, including the 2F85, 2F140, and Hand-E models.
It handles calibration, movement commands, and state monitoring.
"""

import time
import logging
import threading

import numpy as np


import pyrobotiqgripper


from robits.core.abc.gripper import GripperBase


logger = logging.getLogger(__name__)


class RobotiqGripper(GripperBase):
    """
    Implementation for Robotiq grippers (2F85, 2F140, and Hand-E).

    This class provides an interface for controlling Robotiq grippers
    using the pyRobotiqGripper library. It includes functionality for
    calibration, opening and closing the gripper, and monitoring the
    gripper state.

    Thread-safe implementation ensures proper operation in multi-threaded
    applications.
    """

    def __init__(self, gripper_name: str, activate=True, **kwargs):
        """
        Initialize a Robotiq gripper controller.

        Sets up communication with the gripper and optionally performs
        calibration and initial positioning.

        :param gripper_name: Identifier for this gripper instance
        :param activate: Whether to activate and calibrate the gripper (default: True)
        :param kwargs: Additional configuration parameters
        """
        self.gripper = pyrobotiqgripper.RobotiqGripper()
        self.lock = threading.Lock()

        # Status information from the gripper registers:
        # before calibrate
        # {'gOBJ': 0, 'gSTA': 0, 'gGTO': 0, 'gACT': 0, 'kFLT': 0, 'gFLT': 0, 'gPR': 100, 'gPO': 3, 'gCU': 0}
        # gripper: closed
        # {'gOBJ': 3, 'gSTA': 3, 'gGTO': 1, 'gACT': 1, 'kFLT': 0, 'gFLT': 0, 'gPR': 255, 'gPO': 230, 'gCU': 0}
        # gripper: open
        # {'gOBJ': 3, 'gSTA': 3, 'gGTO': 1, 'gACT': 1, 'kFLT': 0, 'gFLT': 0, 'gPR': 0, 'gPO': 3, 'gCU': 0}

        if activate:
            logger.debug("Calibrating gripper")
            self.gripper.calibrate(0, 40)
            self.gripper.open()

        self._gripper_name = gripper_name

    @property
    def gripper_name(self):
        return self._gripper_name

    @classmethod
    def get_gripper_type_name(cls):
        return "robotiq"

    def read_calibration(self):

        calibration_data = {
            "ratio_a": self.gripper._aCoef,
            "ratio_b": self.gripper._bCoef,
            "closemm": self.gripper.closemm,
            "openmm": self.gripper.openmm,
            "open_encoder": self.gripper.openbit,
            "encoder_close": self.gripper.closebit,
        }
        return calibration_data

    def get_info(self):
        self.gripper.readAll()
        gripper_info = self.gripper.paramDic
        return {
            "info": gripper_info,
            "is_open": self.is_open(),
            "position": self.normalized_width,
        }

    def load_calibration(self):

        # we still need to call gripper.activate()

        calibration_data = {
            "ratio_a": -0.1762114537444934,
            "ratio_b": 40.52863436123348,
            "closemm": 0,
            "openmm": 40.0,
            "open_encoder": 3,
            "encoder_close": 229,
        }

        self.gripper._aCoef = calibration_data["ratio_a"]
        self.gripper._bCoef = calibration_data["ratio_b"]
        self.gripper.openmm = calibration_data["openmm"]
        self.gripper.closemm = calibration_data["closemm"]
        self.gripper.closebit = calibration_data["encoder_close"]
        self.gripper.openbit = calibration_data["open_encoder"]

    def is_active(self):
        return self.lock.locked()

    def open(self):
        with self.lock:
            self.gripper.open()

    def close(self):
        with self.lock:
            self.gripper.close()

    @property
    def normalized_width(self) -> float:
        """
        Get the current gripper opening as a normalized value.

        Returns the current gripper opening as a value between 0.0 (fully closed)
        and 1.0 (fully open), representing the fraction of the maximum opening.

        :returns: Normalized gripper width (0.0-1.0)
        :raises Warning: If gripper is not calibrated
        """
        with self.lock:
            if not self.gripper.isCalibrated():
                logger.warning("gripper is not calibrated")
                return 0.0
            position_in_mm = self.gripper.getPositionmm()
        return (position_in_mm - self.gripper.closemm) / (
            self.gripper.openmm - self.gripper.closemm
        )

    def get_obs(self):
        return {
            "finger_positions": np.asarray([self.normalized_width]),
            "timestamp": time.time(),
        }

    def is_open(self) -> bool:
        return self.normalized_width > 0.5
