#!/usr/bin/env python3

import sys
import time

from scipy.spatial.transform import Rotation as R

import rich_click as click
from click_prompt import choice_option

from robits.core.abc.control import control_types
from robits.core.config_manager import config_manager
from robits.core.factory import RobotFactory
from robits.core.abc.robot import BimanualRobot

from robits.cli import cli_utils

from robits.cli.cli_utils import console


if not config_manager.available_bimanual_robots:
    console.print(
        "Unable to find any bimanual robot. Please add a bimanual robot configuration and set ROBITS_CONFIG_DIR."
    )
    sys.exit(-1)


@click.command
@choice_option(
    "--robot-name", type=click.Choice(config_manager.available_bimanual_robots)
)
def cli(robot_name):

    console = cli_utils.console

    console.rule("Starting")

    default_robot_orientation = [1, 0, 0, 0]

    robot: BimanualRobot = RobotFactory(robot_name).build()

    robot.right_robot.control.move_home()
    robot.left_robot.control.move_home()

    # robot.control.move_home()

    right, left = robot.right_robot, robot.left_robot

    right.gripper.open()
    left.gripper.open()

    with robot.right_robot.control(
        control_types.cartesian
    ) as right_ctrl, robot.left_robot.control(control_types.cartesian) as left_ctrl:

        console.print("Moving left robot")
        # move left arm to handover position. these are world coordinates for mujoco
        left_ctrl.update(([0.4, -0.2, 0.4], default_robot_orientation))
        left_ctrl.update(
            ([0, -0.2, 0], R.from_euler("XYZ", (270, 0, 90), degrees=True).as_quat()),
            relative=True,
        )

        console.print("Picking up object")
        right_ctrl.update(([0.52, -0.2, 0.2], default_robot_orientation))
        right_ctrl.update(([0.52, -0.2, 0.13], default_robot_orientation))
        right.gripper.close()
        time.sleep(1.0)

        # right_ctrl.update(([0.0, 0.0, 0.1], ), relative=True)

        console.print("Moving to hand over pose")
        # move right arm to handover position
        right_ctrl.update(([0.4, -0.2, 0.4], default_robot_orientation))
        right_ctrl.update(
            ([0, 0, 0], R.from_euler("XYZ", (90, 0, 0), degrees=True).as_quat()),
            relative=True,
        )

        time.sleep(1.0)

        console.print("Starting handover")

        # move right forward and do the hand over
        right_ctrl.update(([0, 0.4, 0.0], [0, 0, 0, 1]), relative=True)
        left.gripper.close()
        right.gripper.open()

        time.sleep(1.0)

        # move back
        right_ctrl.update(([0, -0.2, 0.0], [0, 0, 0, 1]), relative=True)
        left_ctrl.update(([0, 0.2, 0.0], [0, 0, 0, 1]), relative=True)

        time.sleep(1.0)


if __name__ == "__main__":
    cli()
