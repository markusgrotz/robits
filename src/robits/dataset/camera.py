from typing import Any
from typing import Tuple
from typing import Dict
from typing import Union

from pathlib import Path

from robits.core.data_model.dataset import Dataset
from robits.core.data_model.camera_capture import CameraData
from robits.core.abc.camera import CameraBase

from robits.dataset.io.reader import DatasetReader
from robits.core.utils import FrequencyTimer


class DatasetCamera(CameraBase):
    """
    Replay camera images from a dataset.
    """

    def __init__(
        self, camera_name: str, dataset: Union[Dataset, str], **kwargs
    ) -> None:
        """
        Initialize the DatasetCamera.

        :param dataset: The dataset containing camera entries or the path to the dataset
        :param camera_name: The name of the camera within the dataset.
        """
        self._camera_name = camera_name
        if isinstance(dataset, Dataset):
            self.dataset = dataset
        else:
            self.dataset = DatasetReader(Path(dataset)).load()
        self.seq = 0
        self.len_obs = len(self.dataset.entries)
        self.frequency_timer = FrequencyTimer(self.dataset.metadata["frequency"])

    @property
    def camera_name(self) -> str:
        return self._camera_name

    @property
    def idx(self) -> int:
        """
        Get the current index in the dataset.

        :return: The index of the current entry.
        """
        return self.seq % self.len_obs

    @property
    def intrinsics(self) -> Any:
        """
        Retrieve the intrinsics of the camera.

        :return: The camera intrinsics.
        """
        entry = self.dataset.entries[self.idx]
        return entry.camera_info[f"{self.camera_name}_intrinsics"]

    @property
    def extrinsics(self) -> Any:
        """
        Retrieve the extrinsics of the camera.

        :return: The camera extrinsics.
        """
        entry = self.dataset.entries[self.idx]
        return entry.camera_info[f"{self.camera_name}_extrinsics"]

    def get_camera_data(self) -> Tuple[CameraData, Dict[str, Any]]:
        """
        Get the current camera data from the dataset.

        :return: A tuple containing the camera images and the metadata
        """
        entry = self.dataset.entries[self.idx]
        camera_data = entry.camera_data[self.camera_name]
        self.frequency_timer.wait_for_cycle()
        self.seq += 1
        return camera_data, dict()
