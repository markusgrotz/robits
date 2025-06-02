"""
Implementation for Intel RealSense cameras.

This module provides an implementation of the CameraBase interface for
Intel RealSense depth cameras, supporting RGB and depth imaging. It handles
camera discovery, configuration, and streaming.
"""

from typing import Dict
from typing import Any
from typing import Tuple
from typing import List

import threading
from functools import lru_cache
import logging


import numpy as np

import pyrealsense2 as rs

from robits.core.abc.camera import CameraBase
from robits.core.data_model.camera_capture import CameraData

logger = logging.getLogger(__name__)


class RealsenseCamera(CameraBase):
    """
    Implementation for Intel RealSense depth cameras.

    Provides RGB and depth imaging from RealSense cameras (D400 series).
    Supports configurable resolution, frame rate, and imaging modes.
    Thread-safe implementation ensures proper operation in multi-threaded
    applications.
    """

    @classmethod
    def get_camera_type_name(cls) -> str:
        """
        Get the type name for this camera implementation.

        :returns: The string identifier for RealSense cameras
        """
        return "realsense"

    @classmethod
    def list_camera_info(cls) -> List[Tuple[str, str]]:
        """
        Get information about all connected RealSense cameras.

        :returns: List of tuples containing (camera_name, serial_number)
        """
        info = []
        ctx = rs.context()
        for device in ctx.devices:
            camera_name = device.get_info(rs.camera_info.name)
            serial_number = device.get_info(rs.camera_info.serial_number)
            info.append((camera_name, serial_number))
        return info

    def __init__(self, **kwargs):
        """
        Initialize a RealSense camera.

        Configures and starts the camera stream with specified parameters.

        :param kwargs: Camera configuration parameters including:
            - rgb: Enable RGB imaging (default: True)
            - depth: Enable depth imaging (default: True)
            - width: Image width in pixels (default: 640)
            - height: Image height in pixels (default: 480)
            - hz: Frame rate in Hz (default: 30)
            - camera_name: Identifier for this camera (default: "front")
            - serial_number: Specific camera to use (default: use first available)
        """

        self.config = {
            "rgb": True,
            "depth": True,
            "point_cloud": False,
            "width": 640,
            "height": 480,
            "hz": 30,
            "camera_name": "front",
        }
        self.config.update(kwargs)

        config = self.config

        logger.info("Using camera config %s", self.config)

        pipeline = rs.pipeline()
        rs_config = rs.config()

        if "rgb" in config and config["rgb"]:
            # ..todo:: we should use rs.format.rgb8
            rs_config.enable_stream(
                rs.stream.color,
                config["width"],
                config["height"],
                rs.format.bgr8,
                config["hz"],
            )
        if "depth" in config and config["depth"]:
            rs_config.enable_stream(
                rs.stream.depth,
                config["width"],
                config["height"],
                rs.format.z16,
                config["hz"],
            )
        if "serial_number" in config:
            rs_config.enable_device(config["serial_number"])
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)

        if not rs_config.can_resolve(pipeline_wrapper):
            logger.error("Unable to resolve camera profile")
            self.pipeline = None
            return

        try:
            profile = pipeline.start(rs_config)
        except RuntimeError as e:
            logger.error("Unable to connect to camera. Exception was %s", e)
            self.pipeline = None
            return

        align_to = rs.stream.color
        align = rs.align(align_to)
        self.align = align
        self.pipeline = pipeline

        self.profile = profile
        self.config = config

        self.lock = threading.Lock()

        logger.debug(
            "Camera parameters are intrinsics=%s, extrinsics=%s",
            self.intrinsics,
            self.extrinsics,
        )

        # get a test image to check if everything is working.
        # increase the timeout

        self.get_camera_data(1000 * 5)
        logger.info("Camera %s initialized.", self.camera_name)

    @property
    def camera_name(self):
        return self.config["camera_name"]

    @lru_cache()
    def is_wrist_camera(self) -> bool:
        if "is_wrist" not in self.config:
            return False
        return self.config["is_wrist"]

    def extract_intrinsics(self) -> np.ndarray:
        """
        Reads the intrinsics from the camera

        :returns: the intrinsics parameters
        """

        if not self.profile:
            logger.error("Camera is not initialized")
            raise Exception("Camera not initalized")

        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        profile = frames.get_profile()
        depth_intrinsics = profile.as_video_stream_profile().get_intrinsics()

        intrinsics = np.identity(3)
        intrinsics[0, 0] = depth_intrinsics.fx
        intrinsics[1, 1] = depth_intrinsics.fy
        intrinsics[0, 2] = depth_intrinsics.ppx
        intrinsics[1, 2] = depth_intrinsics.ppy

        return intrinsics

    def get_camera_data(self, timeout: int = 100) -> Tuple[CameraData, Dict[str, Any]]:
        """
        Capture images from the RealSense camera.

        Retrieves RGB and depth frames from the camera, aligns them,
        and returns them as a CameraData object with associated metadata.

        .. seealso:: robits.core.abc.camera.CameraBase.get_camera_data

        :param timeout: Timeout in milliseconds for waiting for frames
        :returns: Tuple of (CameraData, metadata_dict)
        :raises RuntimeError: If frames cannot be captured within timeout
        """
        new_camera_data = None
        metadata = {}
        try:
            frames = self.pipeline.wait_for_frames(timeout)
            metadata["timestamp"] = frames.timestamp / 1000.0
            metadata["seq"] = frames.frame_number
        except RuntimeError as e:
            logger.warning(
                "Unable to capture camera images for camera %s. Exception is %s",
                self.camera_name,
                e,
            )
            return new_camera_data, {}

        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            logger.error("Unable to align depth image")
            return new_camera_data, {}

        depth_image = np.asanyarray(aligned_depth_frame.get_data()).copy()
        depth_image = depth_image.astype(np.float32) * self.depth_scale
        color_image = np.asanyarray(color_frame.get_data()).copy()

        return CameraData(color_image[:, :, ::-1], depth_image), metadata

    @property
    def depth_scale(self) -> float:
        """
        ..todo:: read from camera
        """
        return 1.0 / 1000.0

    def __del__(self):
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
