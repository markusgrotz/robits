#!/usr/bin/env python3

import time
import logging


import open3d as o3d

import rich_click as click
from click_prompt import filepath_option

from robits.core.abc.control import control_types

from robits.utils import vision_utils
from robits.dataset.io.recorder import DatasetRecorder
from robits.dataset.io.writer import DatasetWriter

from robits.cli import cli_utils
from robits.cli import cli_options


logger = logging.getLogger(__name__)


@click.command
@filepath_option(
    "--output-path", default="/tmp/data/stack_blocks", help="Where to save the ."
)
@cli_options.robot()
def cli(output_path, robot):

    console = cli_utils.console

    console.rule("Starting")
    console.print(
        "Select two blocks with shift + left click. Use shift + right click to undo."
    )

    default_robot_orientation = [0, -1, 0, 0]

    camera = robot.cameras[0]
    recorder = DatasetRecorder(robot)
    writer = DatasetWriter(output_path, recorder)

    logger.info("Moving home.")
    robot.control.move_home()
    robot.gripper.open()

    camera_data, _info = camera.get_camera_data()

    pcd = vision_utils.depth_to_pcd(camera_data, camera, apply_extrinsics=True)

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Select two blocks")
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    points = vis.get_picked_points()

    if len(points) != 2:
        console.print("Insufficient points selected")
        return

    with writer:

        time.sleep(0.1)

        with robot.control(control_types.cartesian) as ctrl:

            target_point = pcd.points[points[0]].copy()
            target_point[2] += 0.05
            ctrl.update((target_point, default_robot_orientation))

            target_point = pcd.points[points[0]].copy()
            target_point[2] -= 0.01
            ctrl.update((target_point, default_robot_orientation))
            robot.gripper.close()

            target_point = pcd.points[points[1]].copy()
            target_point[2] += 0.15
            ctrl.update((target_point, default_robot_orientation))

            target_point = pcd.points[points[1]].copy()
            target_point[2] += 0.05
            ctrl.update((target_point, default_robot_orientation))
            robot.gripper.open()

        robot.control.move_home()

        time.sleep(1)

    console.rule("Done")


if __name__ == "__main__":
    cli_utils.setup_cli()
    cli()
