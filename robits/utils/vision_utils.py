from typing import Tuple
from typing import Dict
from typing import Any

import numpy as np
import open3d as o3d
from PIL import Image

from robits.core.data_model.camera_capture import CameraData


def camera_parameters_to_open3d(
    w: int, h: int, intrinsics: np.ndarray
) -> o3d.camera.PinholeCameraParameters:
    """
    Converts camera intrinsic parameters to Open3D PinholeCameraIntrinsic format.

    :param w: The width of the image.
    :param h: The height of the image.
    :param intrinsics: The 3x3 intrinsic matrix.
    :returns: Open3D PinholeCameraIntrinsic object.
    """
    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    ppx = intrinsics[0, 2]
    ppy = intrinsics[1, 2]
    return o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, ppx, ppy)


def depth_to_pcd(
    camera_data, camera, apply_extrinsics: bool = False
) -> o3d.geometry.PointCloud:
    """
    Converts depth and RGB images to a point cloud.

    :param camera_data: Camera data containing RGB and depth images.
    :param camera: Camera object containing intrinsics and extrinsics.
    :param apply_extrinsics: Flag whether to apply extrinsic transformations.
    :returns: Open3D PointCloud object.
    """
    h, w = camera_data.depth_image.shape
    depth_image = camera_data.depth_image
    depth = o3d.geometry.Image(depth_image)

    rgb_image = np.ascontiguousarray(camera_data.rgb_image)
    color = o3d.geometry.Image(rgb_image)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_scale=1.0, convert_rgb_to_intensity=False
    )

    intrinsics = camera_parameters_to_open3d(w, h, camera.intrinsics)
    if apply_extrinsics:
        extrinsics = camera.extrinsics
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsics, extrinsics, project_valid_depth_only=True
        )
    else:
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsics, project_valid_depth_only=False
        )
    return pcd


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


def get_camera_data_resized(
    camera_data, target_image_size: Tuple[int, int]
) -> Tuple[CameraData, Dict[str, Any]]:
    """
    Resizes camera images while preserving metadata.

    :param camera_data: The original camera data containing RGB and depth images.
    :param target_image_size: The target image size (width, height).
    :returns: Tuple containing resized camera data and metadata.
    """
    rgb_image = camera_data.rgb_image
    depth_image = camera_data.depth_image

    rgb_image = Image.fromarray(rgb_image).resize(target_image_size)
    depth_image = Image.fromarray(depth_image).resize(target_image_size)

    new_camera_data = CameraData(np.array(rgb_image), np.array(depth_image))
    new_metadata: Dict[str, Any] = {}
    return new_camera_data, new_metadata
