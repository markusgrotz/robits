#!/usr/bin/env python3
import time
import sys

from functools import partial

import numpy as np

import rich_click as click
from click_prompt import filepath_option

from robits.core.abc.control import control_types
from robits.utils.transform_utils import transform_pose

from robits.dataset.io.recorder import DatasetRecorder
from robits.dataset.io.writer import DatasetWriter

from robits.cli import cli_utils
from robits.cli import cli_options

from robits.sim.env_design import env_designer

@click.command
@filepath_option(
    "--output-path", default="/tmp/data/stack_blocks", help="Where to save the ."
)
@cli_options.robot()
def cli(output_path, robot):

    env_designer.add_blocks()

    console = cli_utils.console
    console.rule("Starting")

    pose_to_robot = partial(transform_pose, robot.transform_world_to_robot)

    default_robot_orientation = np.array([0, -1, 0, 0])

    if not robot.gripper:
        console.print("Robot config should contain a gripper")
        sys.exit(-1)

    recorder = DatasetRecorder(robot)
    writer = DatasetWriter(output_path, recorder)

    console.print("Moving to default joint positions.")
    robot.control.move_home()
    robot.gripper.open()

    with writer:

        time.sleep(0.1)

        with robot.control(control_types.cartesian) as ctrl:

            grasp_pose = np.array([0.5, 0.2, 0.015])

            console.print("Moving to pregrasp pose.")
            pre_grasp_pose = grasp_pose.copy()
            pre_grasp_pose[2] = 0.15
            ctrl.update(pose_to_robot(pre_grasp_pose, default_robot_orientation))

            console.print("Moving to grasp pose.")
            ctrl.update(pose_to_robot(grasp_pose, default_robot_orientation))
            robot.gripper.close()
            time.sleep(1)

            console.print("Again moving to pregrasp pose.")
            ctrl.update(pose_to_robot(pre_grasp_pose, default_robot_orientation))

            console.print("Moving to the right.")
            target_point = np.array([0.5, -0.2, 0.1])
            ctrl.update(pose_to_robot(target_point, default_robot_orientation))
            robot.gripper.open()
            time.sleep(0.5)

        robot.control.move_home()

        time.sleep(1)

    console.rule("Done")


if __name__ == "__main__":
    cli_utils.setup_cli()
    cli()
