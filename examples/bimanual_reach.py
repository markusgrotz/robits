#!/usr/bin/env python3

import sys
import numpy as np

import rich_click as click
from click_prompt import choice_option

from robits.core.factory import RobotFactory
from robits.core.config_manager import config_manager
from robits.core.abc.control import control_types

from robits.cli.cli_utils import console

if not config_manager.available_bimanual_robots:
    console.print(
        "[orange]Warning[/orange] Unable to find any bimanual robot. Please add a bimanual robot configuration and set ROBITS_CONFIG_DIR."
    )
    sys.exit(-1)


@click.command
@choice_option(
    "--robot-name", type=click.Choice(config_manager.available_bimanual_robots)
)
def cli(robot_name):

    console.rule("Starting")
    default_robot_orientation = [1, 0, 0, 0]
    id_rot = [0, 0, 0, 1]
    z_rot_by_90_degrees = [0.0, 0.0, 0.71, 0.71]

    robot = RobotFactory(robot_name).build()

    right, left = robot.right_robot, robot.left_robot
    left_position = (left.transform_world_to_robot @ np.array([0.5, 0.65, 0.5, 1.0]))[
        :3
    ]
    right_position = (right.transform_world_to_robot @ np.array([0.5, 0.35, 0.5, 1.0]))[
        :3
    ]

    ctrl_cfg = {"controller_type": control_types.cartesian, "asynchronous": True}
    with robot.control(**ctrl_cfg) as ctrl:

        ctrl.update(
            right=(right_position, default_robot_orientation),
            left=(left_position, default_robot_orientation),
        )

        input("Press enter to update ctrl target.")

        ctrl.update(
            right=(np.zeros(3), id_rot),
            left=(np.zeros(3), z_rot_by_90_degrees),
            relative=True,
        )

        input("Press enter to interrupt again.")


if __name__ == "__main__":
    cli()
