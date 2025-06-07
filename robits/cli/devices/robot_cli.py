#!/usr/bin/env python3

import rich_click as click

from robits.cli.base_cli import cli
from robits.cli.base_cli import console

from robits.cli import cli_options


@cli.command()
@cli_options.robot()
@click.option("-c", default=None, help="Command to execute before starting the shell")
def shell(robot, c: str):
    """
    Creates an interactive shell. Use robot variable to interact
    """
    from robits.core.abc.control import control_types  # noqa: F401

    if c:
        exec(c)

    from IPython import embed
    embed()


@cli.group()
def info():
    """
    Read various information about the current robot state
    """
    pass


@info.command()
@cli_options.robot()
def pose(robot):
    """
    prints the current pose
    """
    console.print()
    console.rule("EEF Pose")
    console.print(robot.eef_pose)
    console.print()


@info.command()
@cli_options.robot()
def matrix(robot):
    """
    prints the current pose
    """
    console.print()
    console.rule("EEF Matrix")
    console.print(robot.eef_matrix)
    console.print()


@info.command()
@cli_options.robot()
def joint_positions(robot):
    """
    prints the currnet joint positions of the robot
    """
    console.print()
    console.rule("robot joint positions")
    console.print(robot.get_proprioception_data(False, False)["joint_positions"])
    console.print()
