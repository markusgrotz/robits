from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R


def transform_pose(
    transform: np.ndarray, position: np.ndarray, quaternion: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    :param transform: The 4x4 matrix transformation to apply
    :param position: The position to transform
    :param quaternion: The quaternion to transform
    """
    R_matrix = transform[:3, :3]
    T_vector = transform[:3, 3]
    position_transformed = R_matrix @ np.array(position) + T_vector
    rotation = R.from_matrix(R_matrix)
    quaternion_transformed = (rotation * R.from_quat(quaternion)).as_quat()
    return position_transformed, quaternion_transformed


def pose2mat(pose: Tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    """
    Converts a pose tuple to a 4x4 matrix
    Quaternion format is xyzw

    :param pose: (position, quaternion)
    :returns: the transformation matrix
    """
    position, quaternion = pose
    mat = np.identity(4)
    mat[:3, :3] = R.from_quat(quaternion).as_matrix()
    mat[:3, 3] = position
    return mat


def mat2pose(mat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Converts a 4x4 matrix to a pose tuple

    :param mat: the transformation matrix
    :returns: (position, quaternion)
    """
    position = mat[:3, 3]
    quaternion = R.from_matrix(mat[:3, :3]).as_quat()
    return position, quaternion

