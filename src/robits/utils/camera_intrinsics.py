from typing import Tuple

import numpy as np


def resize_intrinsics(
    intrinsics, current_image_size: Tuple[int, int], target_image_size: Tuple[int, int]
):
    """
    Rescales camera intrinsics to match a new image size.

    :param intrinsics: The 3x3 intrinsic matrix to be resized.
    :param current_image_size: Tuple (width, height) of the original image.
    :param target_image_size: Tuple (width, height) of the target image.
    :returns: The resized intrinsic matrix.
    """
    scale_x = target_image_size[0] / current_image_size[0]
    scale_y = target_image_size[1] / current_image_size[1]

    intrinsics = intrinsics.copy()
    intrinsics[0, 0] *= scale_x
    intrinsics[1, 1] *= scale_y
    intrinsics[0, 2] *= scale_x
    intrinsics[1, 2] *= scale_y

    return intrinsics


def make_camera_intrinsics(fx: float, fy: float, width: int, height: int) -> np.ndarray:
    intrinsics = np.identity(3)
    intrinsics[0, 0] = fx
    intrinsics[1, 1] = fy
    intrinsics[0, 2] = width / 2.0
    intrinsics[1, 2] = height / 2.0
    return intrinsics


def intrinsics_from_fovy(fovy, width, height) -> np.ndarray:
    """
    :param fovy: field of view in y-axis direction
    :param width: image width
    :param height: image height
    :return: intrinsic parameters
    """
    focal_length = (height / 2) / np.tan(fovy / 2.0)
    return make_camera_intrinsics(focal_length, focal_length, width, height)
