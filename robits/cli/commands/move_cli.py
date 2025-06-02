"""
Movement command module for controlling robot positioning.

This module provides commands for moving robots in various directions and
to predefined positions using Cartesian coordinates. All movements are
executed through the robot's controller interface.
"""

import numpy as np

from robits.core.abc.control import control_types

from robits.cli.base_cli import cli
from robits.cli import cli_options


@cli.group()
def move():
    """
    Commands for moving the robot in different directions.

    This command group provides operations for positioning the robot using
    different movement types:

    - Directional movements (up, down, left, right, forward, back)
    - Predefined positions (home)
    - Test movement patterns

    All movement commands require specifying a robot configuration with --robot-name.
    """
    pass


@move.command()
@cli_options.robot()
def left(robot):
    """
    Move the end-effector of the robot 0.1m to the left.

    Uses Cartesian control to move the robot's end-effector along the y-axis
    in the positive direction (left) by 0.1 meters while maintaining the
    current orientation.

    :param robot_name: The configuration name of the robot to control.
    """
    with robot.control(control_types.cartesian) as ctrl:
        ctrl.update(([0.0, 0.1, 0.0], [0, 0, 0, 1]), relative=True)


@move.command()
@cli_options.robot()
def right(robot):
    """
    Move the end-effector of the robot 0.1m right
    """
    with robot.control(control_types.cartesian) as ctrl:
        ctrl.update(([0.0, -0.1, 0.0], [0, 0, 0, 1]), relative=True)


@move.command()
@cli_options.robot()
def back(robot):
    """
    Move the end-effector of the robot 0.1m back
    """
    with robot.control(control_types.cartesian) as ctrl:
        ctrl.update(([-0.1, 0.0, 0.0], [0, 0, 0, 1]), relative=True)


@move.command()
@cli_options.robot()
def forward(robot):
    """
    Move the end-effector of the robot 0.1m forward
    """
    with robot.control(control_types.cartesian) as ctrl:
        ctrl.update(([0.1, 0.0, 0.0], [0, 0, 0, 1]), relative=True)


@move.command()
@cli_options.robot()
def up(robot):
    """
    Move the end-effector of the robot 0.1m up
    """
    with robot.control(control_types.cartesian) as ctrl:
        ctrl.update(([0.0, 0.0, 0.1], [0, 0, 0, 1]), relative=True)


@move.command()
@cli_options.robot()
def down(robot):
    """
    Move the end-effector of the robot 0.1m down
    """
    with robot.control(control_types.cartesian) as ctrl:
        ctrl.update(([0.0, 0.0, -0.1], [0, 0, 0, 1]), relative=True)


@move.command()
@cli_options.robot()
def home(robot):
    """
    Move the robot to its predefined home position.

    This command moves the robot to a safe, predefined home position in joint space.
    The home position is defined in the robot's configuration and is typically a
    position where the robot has good maneuverability and visibility.

    This is often a good starting position before performing other operations.

    :param robot_name: The configuration name of the robot to control.
    """
    robot.control.move_home()


@move.command()
@cli_options.robot()
def test(robot):
    """
    Execute a test movement pattern
    """
    robot.control.move_home()

    deltas = [
        np.array([0.0, 0.2, 0.0]),
        np.array([0.0, 0.0, -0.4]),
        np.array([0.0, -0.4, 0.0]),
        np.array([0.0, 0.0, 0.4]),
        np.array([0.0, 0.2, 0.0]),
    ]

    with robot.control(control_types.cartesian) as ctrl:

        ctrl.update((np.array([0.0, 0.0, 0.2]), [0, 0, 0, 1]), relative=True)

        for i in range(5):
            # delta = np.array([0.0, 0.20, 0.0])
            delta_position = deltas[i % len(deltas)]
            ctrl.update((delta_position, [0, 0, 0, 1]), relative=True)
