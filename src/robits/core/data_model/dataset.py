from typing import Dict
from typing import List
from typing import Any

from dataclasses import dataclass
from dataclasses import field

import numpy as np

from robits.core.data_model.camera_capture import CameraData


@dataclass(frozen=True)
class Entry:
    """
    Represents a single snapshot entry in a dataset.

    :param seq: Serial identifier of this snapshot. Should be ordered
    :param proprioception: Dictionary containing proprioceptive sensor data.
    :param camera_data: Dictionary containing captured camera data.
    :param camera_info: Dictionary containing camera metadata as NumPy arrays.
    """

    seq: int

    proprioception: Dict[str, Any] = field(default_factory=lambda: {})

    camera_data: Dict[str, CameraData] = field(default_factory=lambda: {})

    camera_info: Dict[str, np.ndarray] = field(default_factory=lambda: {})


@dataclass
class Dataset:
    """
    Represents a dataset containing multiple entries and metadata.

    :param entries: List of dataset entries.
    :param metadata: Metadata related to the dataset.
    """

    entries: List[Entry] = field(default_factory=lambda: [])

    metadata: Dict[str, Any] = field(default_factory=lambda: {})

    def __len__(self):
        return len(self.entries)

    def __getitem__(self, idx):
        return self.entries[idx]
