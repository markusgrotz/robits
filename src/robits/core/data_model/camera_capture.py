from typing import Dict
from typing import Optional

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraData:
    """
    Data model for camera images. Mainly RGB-D

    - `rgb_image` is stored in (height, width, colors) format.
    - `depth_image` is stored in (height, width) format.
    - Data type for `rgb_image` is `np.uint8`.
    - Data type for `depth_image` is `float32`, with depth values in meters (m).
    - Units for `point_cloud` are in meters (m).

    :ivar rgb_image: The RGB image stored as a NumPy array (height, width, 3).
    :ivar depth_image: The depth image stored as a NumPy array (height, width).
    :ivar np_xyz_points: The 3D point cloud coordinates (optional).
    :ivar np_colors: The corresponding RGB colors for the point cloud (optional).
    """

    rgb_image: np.ndarray
    depth_image: np.ndarray

    np_xyz_points: Optional[np.ndarray] = None
    np_colors: Optional[np.ndarray] = None

    def camera_data(self) -> Dict[str, bool]:
        """
        Checks which types of camera data are available.

        :returns: A dictionary indicating the presence of RGB, depth, and point cloud data.
        """
        return {
            "rgb": self.rgb_image is not None,
            "depth": self.depth_image is not None,
            "point_cloud": self.has_point_cloud(),
        }

    def has_images(self) -> bool:
        """
        Checks if both RGB and depth images are available.

        :returns: True if both RGB and depth images exist, otherwise False.
        """
        return (self.rgb_image is not None) and (self.depth_image is not None)

    def has_point_cloud(self) -> bool:
        """
        Checks if the point cloud data is available.

        :returns: True if both `np_xyz_points` and `np_colors` are available, otherwise False.
        """
        return (self.np_xyz_points is not None) and (self.np_colors is not None)

    @property
    def point_cloud(self) -> np.ndarray:
        """
        Constructs the point cloud by concatenating `np_xyz_points` and `np_colors`.

        :returns: A NumPy array representing the point cloud.
        :raises ValueError: If the point cloud data is unavailable.
        """
        if not self.has_point_cloud():
            raise ValueError("Point cloud data is not available.")
        return np.concatenate([self.np_xyz_points, self.np_colors], axis=2)

    def to_dict(self) -> Dict[str, np.ndarray]:
        """
        Converts the CameraData instance into a dictionary.

        :returns: A dictionary containing `rgb_image`, `depth_image`, and optionally `point_cloud`.
        """
        data = {"rgb_image": self.rgb_image, "depth_image": self.depth_image}
        if self.has_point_cloud():
            data.update({"point_cloud": self.point_cloud})
        return data
