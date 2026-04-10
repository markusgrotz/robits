import time
import threading
import json
import logging
from pathlib import Path

import numpy as np
from PIL import Image

from robits.core.utils import NumpyJSONEncoder

logger = logging.getLogger(__name__)


class DatasetWriter:
    """
    Class for writing dataset recordings asynchronously.
    """

    def __init__(self, output_path, recorder, rgb_format="jpg"):
        """
        Initializes the DatasetWriter.

        :param output_path: The path where the dataset will be stored.
        :param recorder: The DatasetRecorder instance responsible for collecting data.
        :param rgb_format: The format to save RGB images (default is "jpg").
        """
        self.output_path = self.get_demo_path(output_path)
        self.recorder = recorder
        self.thread = threading.Thread(target=self.do_write)
        self.thread.daemon = True
        self.is_stopped = False
        self.rgb_format = rgb_format

    def get_demo_path(self, output_path) -> Path:
        """
        Generates a new demo path within the given output directory.

        :param output_path: The base output path for storing datasets.
        :return: The formatted dataset path.
        """
        base_path = Path(output_path).expanduser().resolve()
        base_path.mkdir(parents=True, exist_ok=True)

        if not any(base_path.iterdir()):
            i = 0
        else:
            demo_folders = [
                int(p.name.split("_")[-1])
                for p in base_path.iterdir()
                if p.is_dir() and p.name.startswith("demo_")
            ]
            i = max(demo_folders, default=-1) + 1

        return base_path / f"demo_{i:04d}"

    def do_write(self) -> None:
        """
        Continuously writes recorded data until stopped.
        """
        while not self.is_stopped:
            while not self.recorder.data_queue.empty():
                logger.debug(
                    "Currently %s pending items", self.recorder.data_queue.qsize()
                )
                try:
                    obs = self.recorder.data_queue.get(timeout=0.5)
                    self.write_obs(obs)
                    self.recorder.data_queue.task_done()
                except Exception as e:
                    logger.warning("Exception %s", e)
            logger.debug("Queue is empty")
            time.sleep(0.05)

    def prepare_write(self) -> None:
        """
        Prepares the directory structure and starts the writing thread.
        """
        self.thread.start()
        logger.info("Demo path is %s", self.output_path)
        (self.output_path / "robot_data").mkdir(parents=True, exist_ok=True)
        for camera in self.recorder.robot.cameras:
            camera_name = camera.camera_name
            (self.output_path / f"{camera_name}_rgb").mkdir(parents=True, exist_ok=True)
            (self.output_path / f"{camera_name}_depth").mkdir(
                parents=True, exist_ok=True
            )
            (self.output_path / f"{camera_name}_info").mkdir(
                parents=True, exist_ok=True
            )

    def finalize_write(self) -> None:
        """
        Waits for the recorder to finish and finalizes dataset writing.
        """
        while not self.recorder.is_stopped:
            logger.info("Waiting for recorder to finish")
            print("Waiting for recorder.")
            time.sleep(0.5)

        self.is_stopped = True
        logger.info("Waiting for writing thread to join.")
        if self.thread:
            self.thread.join()

        logger.info("Done writing entries. Finalizing dataset.")
        metadata = self.recorder.metadata
        metadata["rgb_format"] = self.rgb_format
        metadata["depth_format"] = "npz"

        with (self.output_path / "metadata.json").open("w") as f:
            json.dump(metadata, f, indent=4, cls=NumpyJSONEncoder)

        logger.info("Done writing to %s", self.output_path)

    def write_obs(self, observation) -> None:
        """
        Writes a single observation to the dataset.

        :param observation: The observation data containing sequence number, perception, and proprioception.
        """
        seq = observation["seq"]
        perception = observation["perception"]
        proprioception = observation["proprioception"]

        logger.debug("Writing sequence %d to %s", seq, self.output_path)

        for camera in self.recorder.robot.cameras:
            camera_name = camera.camera_name
            depth_image = perception.pop(f"{camera_name}_depth")
            rgb_image = perception.pop(f"{camera_name}_rgb")

            np.savez_compressed(
                self.output_path / f"{camera_name}_depth" / f"depth_{seq:04d}.npz",
                depth=depth_image,
            )
            Image.fromarray(rgb_image).save(
                self.output_path
                / f"{camera_name}_rgb"
                / f"image_{seq:04d}.{self.rgb_format}"
            )

            with (
                self.output_path
                / f"{camera_name}_info"
                / f"camera_intrinsics_{seq:04d}.json"
            ).open("w") as f:
                json.dump(
                    perception[f"{camera_name}_camera_intrinsics"],
                    f,
                    indent=4,
                    cls=NumpyJSONEncoder,
                )

            with (
                self.output_path
                / f"{camera_name}_info"
                / f"camera_extrinsics_{seq:04d}.json"
            ).open("w") as f:
                json.dump(
                    perception[f"{camera_name}_camera_extrinsics"],
                    f,
                    indent=4,
                    cls=NumpyJSONEncoder,
                )

        with (self.output_path / "robot_data" / f"proprioception_{seq:04d}.json").open(
            "w"
        ) as f:
            json.dump(proprioception, f, indent=4, cls=NumpyJSONEncoder)

    def __enter__(self):
        """
        Starts recording and prepares the dataset writer.
        """
        self.recorder.start_recording()
        self.prepare_write()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Stops recording and finalizes dataset writing.
        """
        self.recorder.stop_recording()
        self.finalize_write()
