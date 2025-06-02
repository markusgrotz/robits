from typing import List
from typing import Optional
from typing import Callable

import json
import logging
from pathlib import Path

import numpy as np
from PIL import Image

from robits.core import __version__

from robits.core.data_model.dataset import Dataset
from robits.core.data_model.dataset import Entry
from robits.core.data_model.camera_capture import CameraData


logger = logging.getLogger(__name__)


class DatasetReader:
    """
    Reads a dataset from a file
    """

    def __init__(self, dataset_path: Path) -> None:
        """
        Initializes the dataset reader

        :param dataset_path: the path to the demo
        """
        self.dataset_path = dataset_path.expanduser().resolve()

        metadata_file = self.dataset_path / "metadata.json"
        with metadata_file.open() as f:
            self.metadata = json.load(f)

        if (dataset_version := self.metadata.get("version", None)) != __version__:
            logger.warning(
                "Version mismatch. Current version is %s. Dataset was recorded with %s",
                __version__,
                dataset_version,
            )

    @property
    def num_items(self) -> int:
        """
        :returns: the number of entries within the loaded dataset
        """
        return self.metadata["num_items"]

    def validate(self) -> bool:
        """
        Performs some sanity check on the dataset

        :returns: True if the sanity checks are passed
        """
        if "recording_stopped" not in self.metadata:
            logger.error("Missing entry recording_stopped in dataset metadata")
            return False

        robot_data_dir = self.dataset_path / "robot_data"
        if (actual_items := len(list(robot_data_dir.iterdir()))) != self.num_items:
            logger.error(
                "Inconsistent dataset. Missing proprioception. Expected %s, Actual: %s",
                self.num_items,
                actual_items,
            )
            return False

        for camera_name in self.metadata["camera_names"]:
            rgb_dir = self.dataset_path / f"{camera_name}_rgb"
            if (actual_items := len(list(rgb_dir.iterdir()))) != self.num_items:
                logger.error(
                    "Inconsistent dataset. Missing RGB images for camera %s. Expected %s, Actual: %s",
                    camera_name,
                    self.num_items,
                    actual_items,
                )
                return False

        return True

    def load(
        self, load_camera_images: bool = True, wrapper: Optional[Callable] = None
    ) -> Dataset:
        """
        Loads the dataset into memory.

        :param load_camera_images: Whether to load camera images (default: True).
        :param wrapper: Optional wrapper function for iterating over sequences.
        :returns: A Dataset object containing all entries.
        """
        num_items = self.num_items
        camera_names = self.metadata["camera_names"] if load_camera_images else []

        entries = []
        sequence = wrapper(range(num_items)) if wrapper else range(num_items)
        for seq in sequence:
            entries.append(self.read_entry(seq, camera_names))

        return Dataset(entries, self.metadata)

    def read_entry(self, seq: int, camera_names: List[str]) -> Entry:
        """
        Reads a dataset entry including proprioception and camera data.

        :param seq: The sequence index of the entry.
        :param camera_names: A list of camera names whose data should be loaded.
        :returns: An Entry object containing the loaded data.
        """
        proprio_path = (
            self.dataset_path / "robot_data" / f"proprioception_{seq:04d}.json"
        )
        with proprio_path.open() as f:
            proprioception = json.load(f)

        camera_data = {name: self.load_camera_data(seq, name) for name in camera_names}

        camera_info = {}
        for camera_name in camera_names:
            intrinsics_path = (
                self.dataset_path
                / f"{camera_name}_info"
                / f"camera_intrinsics_{seq:04d}.json"
            )
            extrinsics_path = (
                self.dataset_path
                / f"{camera_name}_info"
                / f"camera_extrinsics_{seq:04d}.json"
            )

            with intrinsics_path.open("r") as f:
                intrinsics = json.load(f)
            with extrinsics_path.open("r") as f:
                extrinsics = json.load(f)

            camera_info[f"{camera_name}_intrinsics"] = np.array(intrinsics)
            camera_info[f"{camera_name}_extrinsics"] = np.array(extrinsics)

        return Entry(seq, proprioception, camera_data, camera_info)

    def load_camera_data(self, seq: int, camera_name: str) -> CameraData:
        """
        Loads camera data for a specific sequence index.

        :param seq: The sequence index of the image.
        :param camera_name: The name of the camera.
        :returns: A CameraData object containing RGB and depth images.
        """
        rgb_format = self.metadata["rgb_format"]
        rgb_path = (
            self.dataset_path / f"{camera_name}_rgb" / f"image_{seq:04d}.{rgb_format}"
        )
        depth_path = self.dataset_path / f"{camera_name}_depth" / f"depth_{seq:04d}.npz"

        rgb_image = np.array(Image.open(rgb_path))
        depth_image = np.load(depth_path)["depth"]

        return CameraData(rgb_image, depth_image)
