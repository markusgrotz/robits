#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
from functools import partial

import rich_click as click

from robits.core.abc.control import control_types
from robits.core.utils import FrequencyTimer

from robits.cli import cli_utils
from robits.cli import cli_options

from robits.utils.transform_utils import transform_pose


def generate_lemniscate_6d_trajectory(
    a=0.5, y_amplitude=0.5, z_amplitude=0.2, z_offset=0.5, steps=300, x_offset=0.5
):
    """Figure-8 motion in the YZ plane with pitch and Z offset."""
    t_vals = np.linspace(-np.pi, np.pi, steps)

    y_vals = (y_amplitude * np.cos(t_vals)) / (1 + np.sin(t_vals) ** 2)
    z_vals = z_amplitude * np.sin(2 * t_vals) + z_offset  # apply Z offset
    x_vals = np.full_like(t_vals, x_offset)

    dy = np.gradient(y_vals)
    dz = np.gradient(z_vals)
    pitch_vals = np.arctan2(dz, dy)  # pitch is angle of movement in YZ plane

    poses = []
    for i in range(steps):
        position = np.array([x_vals[i], y_vals[i], z_vals[i]])
        rot = R.from_euler("x", np.pi) * R.from_euler(
            "y", pitch_vals[i]
        )  # facing forward + pitch
        orientation = rot.as_quat()
        poses.append((position, orientation))
    return poses


@click.command
@cli_options.robot()
def cli(robot):
    console = cli_utils.console
    console.rule("Starting")

    robot.control.move_home()
    timer = FrequencyTimer(25)

    pose_to_robot = partial(transform_pose, robot.transform_world_to_robot)

    trajectory = generate_lemniscate_6d_trajectory()

    timer.reset()
    with robot.control(control_types.cartesian, asynchronous=True) as ctrl:

        for _ in range(10):
            for position, orientation in trajectory:
                orientation = np.array([0, -1, 0, 0])
                ctrl.update(pose_to_robot(position.tolist(), orientation))
                timer.wait_for_cycle()

    robot.control.move_home()
    console.rule("Done")


if __name__ == "__main__":
    cli_utils.setup_cli()
    cli()
