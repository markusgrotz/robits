import unittest

import numpy as np

from robits.core.data_model.camera_capture import CameraData

from robits.core.data_model.dataset import Entry
from robits.core.data_model.dataset import Dataset


class TestEntryAndDataset(unittest.TestCase):

    def test_entry_initialization(self):
        camera_data = {
            "front": CameraData(np.ones((3, 3), dtype=np.uint8), np.ones((3, 3)))
        }
        camera_info = {"front": np.array([1.0, 2.0, 3.0])}
        proprioception = {"joint_angle": 1.57}

        entry = Entry(
            seq=1,
            proprioception=proprioception,
            camera_data=camera_data,
            camera_info=camera_info,
        )

        self.assertEqual(entry.seq, 1)
        self.assertEqual(entry.proprioception, proprioception)
        self.assertEqual(entry.camera_data, camera_data)
        np.testing.assert_array_equal(
            entry.camera_info["front"], np.array([1.0, 2.0, 3.0])
        )

    def test_dataset_initialization_and_access(self):
        entry1 = Entry(seq=0)
        entry2 = Entry(seq=1)
        metadata = {"source": "test_sim"}

        dataset = Dataset(entries=[entry1, entry2], metadata=metadata)

        self.assertEqual(len(dataset), 2)
        self.assertEqual(dataset[0], entry1)
        self.assertEqual(dataset[1].seq, 1)
        self.assertEqual(dataset.metadata["source"], "test_sim")

    def test_dataset_default_values(self):
        dataset = Dataset()
        self.assertEqual(len(dataset.entries), 0)
        self.assertEqual(dataset.metadata, {})


if __name__ == "__main__":
    unittest.main()
