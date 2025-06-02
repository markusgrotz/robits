#!/usr/bin/env python3
"""
Camera command module for interacting with camera devices.

This module provides commands for viewing camera feeds, capturing images,
working with point clouds, and managing camera calibration. Supports both
real and simulated cameras through a common interface.
"""

import time
import os
import subprocess
from datetime import datetime

import rich_click as click
from click_prompt import choice_option
from click_prompt import filepath_option

import numpy as np

from robits.core.factory import CameraFactory
from robits.core.config_manager import config_manager

from robits.cli.base_cli import console
from robits.cli.base_cli import cli


@cli.group()
def camera():
    """
    Commands for working with cameras and visual data.

    This command group provides operations for interacting with camera devices,
    including viewing camera feeds, capturing images, working with point clouds,
    and calibrating cameras. It supports both real hardware cameras (like RealSense)
    and simulated cameras through a common interface.

    Camera commands require specifying a camera configuration with --camera-name.
    """
    pass


@camera.group()
def point_cloud():
    """
    Commands for working with 3D point clouds from camera data.

    This subgroup provides operations for generating, visualizing, and
    saving point clouds derived from RGB-D camera data. Point clouds
    represent the 3D structure of the scene and can be used for
    object detection, manipulation planning, and scene understanding.
    """
    pass


@point_cloud.command(name="view")
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
def pc_view(camera_name):
    """
    View the current point cloud in an interactive 3D visualizer.

    Opens a visualization window displaying the 3D point cloud generated
    from the current RGB-D data of the specified camera. The visualizer
    allows rotating, panning, and zooming to inspect the point cloud.

    Press Ctrl+C in the terminal to close the visualization.

    :param camera_name: The configuration name of the camera to use.
    """

    from robits.core.abc.robot import DummyRobot as Robot
    from robits.vis.scene_visualizer import SceneVisualizer

    camera = CameraFactory(camera_name).build()
    robot = Robot(gripper=None, cameras=[camera])
    visualizer = SceneVisualizer(robot)
    visualizer.show()

    try:
        while True:
            visualizer.update_scene()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    visualizer.close()


@point_cloud.command(name="save")
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
@filepath_option("--output-file", default="/tmp/output.pcd")
@click.option("--is-global/--no-is-global", default=False, is_flag=True)
def pc_save(camera_name, output_file, is_global):
    """
    Save the current point cloud to file
    """

    import open3d as o3d
    from robits.utils import vision_utils

    camera = CameraFactory(camera_name).build()
    camera_data, info = camera.get_camera_data()
    pcd = vision_utils.depth_to_pcd(camera_data, camera)

    if is_global:
        pcd = pcd.transform(camera.extrinsics)
    o3d.io.write_point_cloud(output_file, pcd, write_ascii=False)


@camera.group()
def calibrate():
    """
    Camera calibration related commands
    """
    pass


@calibrate.command()
def extrinsics():
    """
    Calibrate the extrinsics parameter of a camera
    """
    package_folder = os.path.dirname(os.path.abspath(__file__))
    package_folder = os.path.join(os.path.dirname(package_folder), "..", "app")
    cmd = f"{package_folder}/extrinsics_app.py"
    subprocess.call(cmd, shell=True)


@calibrate.command()
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
def intrinsics(camera_name):
    """
    Updates the intrinsic parameters of a camera
    """
    camera = CameraFactory(camera_name).build()

    if not hasattr(camera, "extract_intrinsics") or not callable(
        camera.extract_intrinsics
    ):
        console.print("Camera does not support extraction of intrinsic parameters")
        return

    from robits.core.config import CameraCalibration

    intrinsics = camera.extract_intrinsics()
    config_dict = camera.calibration.to_dict()
    config_dict["intrinsics"] = intrinsics.tolist()
    config_dict["date_updated"] = datetime.now().isoformat()
    new_calibration = CameraCalibration(**config_dict)
    user_config_dir = config_manager.get_user_config_dir()
    config_path = os.path.join(
        user_config_dir, f"calibration_{camera.camera_name}_camera.json"
    )
    new_calibration.save_config(config_path)
    console.print(f"New calibration is now {new_calibration}")


@camera.command()
def list():
    """
    List available cameras on the system
    """
    from rich.table import Table

    table = Table(title="Cameras")

    table.add_column("Library", justify="right", style="cyan", no_wrap=True)
    table.add_column("Name", style="magenta")
    table.add_column("Serial number", justify="right", style="green")

    try:
        from robits.real.realsense_camera import RealsenseCamera

        for name, serial_number in RealsenseCamera.list_camera_info():
            table.add_row("pyrealsense2", name, serial_number)

    except ModuleNotFoundError:
        console.print("Unable to find realsense library. Please check the installation")

    console.print(table)


@camera.command()
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
def hz(camera_name):
    """
    Measure the frequency of the camera
    """
    camera = CameraFactory(camera_name).build()

    measured_times = []

    try:
        while True:

            start_time = time.time()
            camera.get_camera_data()
            elapsed_time = time.time() - start_time
            measured_times.append(elapsed_time)
            average = np.average(np.array(measured_times))
            hz = 1.0 / average

            console.print(
                f"Took {measured_times[-1]:.4f} seconds. Average is {average:.4f} seconds. {hz:.2f} fp/s"
            )

    except KeyboardInterrupt:
        pass


@camera.group()
def info():
    """
    Show information about a camera
    """
    pass


@info.command(name="extrinsics")
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
def info_extrinsics(camera_name):
    """
    Display extrinsic parameters
    """
    camera = CameraFactory(camera_name).build()
    console.print(camera.extrinsics)


@info.command(name="intrinsics")
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
def info_intrinsics(camera_name):
    """
    Display intrinsic parameters
    """
    camera = CameraFactory(camera_name).build()
    console.print(camera.intrinsics)


@camera.group()
def image():
    """
    Image related commands
    """
    pass


@image.command()
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
@filepath_option("--output-file", default="/tmp/output.png")
def save(camera_name, output_file):
    """
    Save an image to file
    """
    from PIL import Image

    camera = CameraFactory(camera_name).build()
    camera_data, _ = camera.get_camera_data()
    Image.fromarray(camera_data.rgb_image).save(output_file)


@image.command()
@choice_option("--camera-name", type=click.Choice(config_manager.available_cameras))
def view(camera_name):
    """
    View the current camera image
    """

    import matplotlib.pyplot as plt

    camera = CameraFactory(camera_name).build()

    # Open a matplotlib figure to display images.
    fig = plt.figure()
    ax = []
    ax.append(fig.add_subplot(1, 2, 1, label="Color"))
    ax.append(fig.add_subplot(1, 2, 2, label="Depth"))

    im = []
    camera_data, _meta_info = camera.get_camera_data()
    im.append(ax[0].imshow(camera_data.rgb_image))
    im.append(ax[1].imshow(camera_data.depth_image, cmap="jet"))

    ax[0].title.set_text("Color")
    ax[1].title.set_text("Depth")

    try:

        while fig is not None:

            # Draw the figure with the images.
            plt.pause(0.01)
            plt.draw()

            camera_data, _meta_info = camera.get_camera_data()

            # Update the images in the figures.
            im[0].set_data(camera_data.rgb_image)
            im[1].set_data(camera_data.depth_image)

    except KeyboardInterrupt:
        fig = None
        plt.close()


if __name__ == "__main__":
    camera()
