#!/usr/bin/env python3

import numpy as np

import rich_click as click
from click_prompt import filepath_option

from robits.core.abc.control import control_types

from robits.dataset.io.recorder import DatasetRecorder
from robits.dataset.io.writer import DatasetWriter

from robits.cli import cli_utils
from robits.cli import cli_options


@click.command
@filepath_option(
    "--output-path", default="/tmp/data/stack_blocks", help="Where to save the dataset."
)
@cli_options.robot()
def cli(output_path, robot):

    console = cli_utils.console
    writer = DatasetWriter(output_path, DatasetRecorder(robot))

    orientation_delta = np.array([0, -1, 0, 0])
    position_delta = np.array([0, 0, 0.05])

    console.rule("Moving robot to default pose")
    robot.control.move_home()

    console.rule("Starting data collection.")

    with writer:

        with robot.control(control_types.cartesian) as ctrl:

            console.print("moving up.")
            ctrl.update((position_delta, orientation_delta), relative=True)
            console.print("moving down.")
            ctrl.update((-position_delta, orientation_delta), relative=True)
            console.print("closing gripper.")
            robot.gripper.close()
            console.print("moving up again.")
            ctrl.update((position_delta, orientation_delta), relative=True)

    console.rule("Done collecting data.")


if __name__ == "__main__":
    cli_utils.setup_cli()
    cli()
