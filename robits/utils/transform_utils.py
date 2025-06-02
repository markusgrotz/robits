from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R


def transform_pose(
    transform: np.ndarray, position: np.ndarray, quaternion: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    :param transform: The transformation to apply
    :param position: The position to transform
    :param quaternion: The quaternion to transform
    """
    R_matrix = transform[:3, :3]
    T_vector = transform[:3, 3]
    position_transformed = R_matrix @ np.array(position) + T_vector
    rotation = R.from_matrix(R_matrix)
    quaternion_transformed = (rotation * R.from_quat(quaternion)).as_quat()
    return position_transformed, quaternion_transformed
