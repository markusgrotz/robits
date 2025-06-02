#!/usr/bin/env python3

import rich_click as click

from robits.core.abc.control import control_types

from robits.cli import cli_utils
from robits.cli import cli_options


@click.command
@cli_options.robot()
def cli(robot):
    console = cli_utils.console
    console.rule("Initializing")

    default_robot_orientation = [0, -1, 0, 0]
    # robot.control.move_home()
    robot.gripper.open()

    console.rule("Starting")

    # If the robot pose is not identity then you to
    # transform the pose to the robot's coordinate system

    with robot.control(control_types.cartesian) as ctrl:

        ctrl.update(([0.52, -0.2, 0.15], default_robot_orientation))
        ctrl.update(([0.52, -0.2, 0.02], default_robot_orientation))
        robot.gripper.close()
        ctrl.update(([0.0, 0.0, 0.2], [0, 0, 0, 1]), relative=True)

    robot.control.move_home()

    console.rule("Done")


if __name__ == "__main__":
    cli_utils.setup_cli()

    cli()
