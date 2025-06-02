from typing import Any
from typing import Dict
from typing import Optional

import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
from pathlib import Path

from robits.core.utils import NumpyJSONEncoder
from robits.core.data_model.dataset import Dataset
from robits.dataset.io.reader import DatasetReader


class StatsWriter:
    def __init__(self, dataset_path: Path, dataset: Optional[Dataset] = None):
        """
        Initializes the StatsWriter.

        :param dataset: A Dataset object containing multiple entries.
        :param output_dir: Path to the dataset folder where stats should be saved.
        """
        self.dataset = dataset
        self.output_dir = dataset_path

    def compute_stats(self) -> Dict[str, Dict[str, Any]]:
        """
        Computes statistics for each proprioceptive key across all entries.

        :return: Dictionary of statistics per proprioception field.
        """

        if self.dataset is None:
            self.dataset = DatasetReader(self.output_dir).load()

        target_keys = [
            "joint_positions",
            "joint_velocities",
            "gripper_joint_positions",
            "gripper_open",
            "eef_position",
            "eef_euler_angles",
        ]

        data = []
        for entry in self.dataset.entries:

            row = {"seq": entry.seq}
            gripper_pose = entry.proprioception["gripper_pose"]
            if isinstance(gripper_pose, tuple):
                position, quaternion = gripper_pose
            else:
                position, quaternion = gripper_pose[:3], gripper_pose[3:]

            entry.proprioception["eef_position"] = position
            entry.proprioception["eef_euler_angles"] = R.from_quat(quaternion).as_euler(
                "xyz"
            )
            entry.proprioception["gripper_open"] = np.atleast_1d(
                np.array(entry.proprioception["gripper_open"])
            ).astype(float)

            for key in target_keys:
                arr = entry.proprioception.get(key)
                for i, val in enumerate(np.atleast_1d(np.array(arr))):
                    row[f"{key}_{i}"] = val
            data.append(row)
        df = pd.DataFrame(data)

        print(df.describe())

        stats_result = {}

        for col in df.columns:
            series = df[col]
            print(col, series)

            stats_result[col] = {
                "mean": series.mean(),
                "std": series.std(),
                "min": series.min(),
                "max": series.max(),
                "25%": series.quantile(0.25),
                "50%": series.quantile(0.50),
                "75%": series.quantile(0.75),
            }
        return stats_result

    def load_stats(self) -> Dict[str, Dict[str, Any]]:
        stats_path = self.output_dir / "proprioception_stats.json"
        with stats_path.open("r") as f:
            return json.load(f)

    def has_stats(self) -> bool:
        return (self.output_dir / "proprioception_stats.json").is_file()

    def write_stats(self) -> None:
        """
        Computes and writes statistics to a JSON file.
        """
        stats = self.compute_stats()
        stats_path = self.output_dir / "proprioception_stats.json"
        with stats_path.open("w") as f:
            json.dump(stats, f, indent=4, cls=NumpyJSONEncoder)
