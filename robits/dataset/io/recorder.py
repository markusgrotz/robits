from typing import Dict
from typing import Any
from typing import Optional

import threading
import datetime
import time

import logging

from queue import Queue


import numpy as np

from robits.core import __version__
from robits.core.utils import FrequencyTimer
from robits.core.abc.robot import UnimanualRobot


logger = logging.getLogger(__name__)


class DatasetRecorder:
    """
    Handles recording of dataset entries from a robot.
    """

    def __init__(self, robot: UnimanualRobot):
        """
        Initializes the DatasetRecorder.

        .. seealso:: robotis.dataset.io.writer.DatasetWriter

        .. code-block:: python

            # Usage with-statement

            recorder = DatasetRecorder(robot)

            with recorder:

                # move the robot
                robot.control.move_home()


            # Alternative

            recorder = DatasetRecorder(robot)

            recorder.start_recording()

            # move the robot
            robot.control.move_home()

            recorder.stop_recording()
        :param robot: The robot instance from which data will be recorded.
        """
        self.metadata: Dict[str, Any] = {}
        self.robot = robot
        self.num_items = 0

        self.is_stopped = False
        self.timer = FrequencyTimer(frequency=10)
        self.data_queue = Queue()
        self.thread: Optional[threading.Thread] = None

    def start_recording(self) -> None:
        """
        Starts the recording process in a separate thread.
        """
        if self.thread:
            raise Exception("Data collection is already running.")

        self.thread = threading.Thread(target=self.do_recording)
        self.is_stopped = False
        self.metadata["recording_started"] = datetime.datetime.now().isoformat()
        self.metadata["time_started"] = time.time()
        self.metadata["frequency"] = self.timer.frequency
        self.metadata["camera_names"] = [c.camera_name for c in self.robot.cameras]
        self.metadata["num_items"] = 0
        self.metadata["robot_info"] = self.robot.get_info()

        self.thread.start()

    def stop_recording(self) -> None:
        """
        Stops the recording process and joins the recording thread.
        """
        self.is_stopped = True

        if not self.thread:
            logger.warning("Unable to stop recording if recording hasn't been started.")
            return

        logger.info("Joining data collecting thread")
        self.thread.join()

        self.metadata["recording_stopped"] = datetime.datetime.now().isoformat()
        self.metadata["time_stopped"] = time.time()

        time_elapsed = self.metadata["time_stopped"] - self.metadata["time_started"]
        self.metadata["time_elapsed"] = time_elapsed
        self.metadata["num_items"] = self.num_items
        self.metadata["version"] = __version__

        logging.info(
            "Data collection done. Collected %d entries. Took %.2f seconds",
            self.metadata["num_items"],
            time_elapsed,
        )

    def __enter__(self):
        """
        Enables the use of the DatasetRecorder as a context manager.

        :returns: The DatasetRecorder instance.
        """
        self.start_recording()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Ensures proper stopping of the recording process when used in a context manager.
        """
        self.stop_recording()

    def do_recording(self) -> None:
        """
        The main recording loop that collects data at a fixed frequency.
        """
        logging.info("Starting recording.")
        self.timer.reset()
        while not self.is_stopped:
            proprioception = self.robot.get_proprioception_data(True, True)
            perception = self.robot.get_vision_data()

            # Update wrist camera extrinsics.
            gripper_matrix = proprioception["gripper_matrix"]
            m = np.linalg.inv(gripper_matrix)

            for camera in self.robot.cameras:
                if camera.is_wrist_camera():
                    extrinsics = perception[f"{camera.camera_name}_camera_extrinsics"]
                    perception[f"{camera.camera_name}_camera_extrinsics"] = np.dot(
                        extrinsics, m
                    )

            self.data_queue.put(
                {
                    "proprioception": proprioception,
                    "perception": perception,
                    "seq": self.num_items,
                }
            )

            self.num_items += 1
            self.timer.wait_for_cycle()

        logging.info("Stopping recording.")
