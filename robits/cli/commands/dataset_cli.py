import logging

import time
from functools import partial
import json
from pathlib import Path

import numpy as np

import rich_click as click
from rich.progress import track
from click_prompt import choice_option
from click_prompt import filepath_option


from robits.core.abc.control import control_types


from robits.cli.base_cli import cli
from robits.cli.base_cli import console
from robits.cli import cli_options

logger = logging.getLogger(__name__)


@cli.group()
def dataset():
    """
    Dataset related commands
    """
    pass


@dataset.command()
@filepath_option("--input-path", default="/tmp/dataset")
def validate(input_path):
    """
    Perform a sanity check on the dataset
    """
    from robits.dataset.io.reader import DatasetReader

    reader = DatasetReader(Path(input_path))
    if reader.num_items == 0:
        console.print("[orange]Empty.[/orange]")
    elif reader.validate():
        console.print("[green]Done.[/green]")
    else:
        console.print("[red]Invalid[/red]")


@dataset.command()
@cli_options.robot()
@filepath_option("--input-path", default="/tmp/dataset")
@choice_option(
    "--control-mode",
    type=click.Choice([control_types.position, control_types.cartesian]),
)
def replay(robot, input_path, control_mode):
    """
    Replay a dataset
    """
    from robits.dataset.io.reader import DatasetReader

    reader = DatasetReader(Path(input_path))

    wrapper = partial(
        track, console=console, total=reader.num_items, description="Loading dataset.."
    )

    dataset = reader.load(load_camera_images=False, wrapper=wrapper)

    frequency = dataset.metadata["frequency"]
    period = 1.0 / frequency

    logger.info("Moving robot to start joint position of dataset")
    # with robot.control(control_types.position) as ctrl:
    ##    ctrl.update(dataset.entries[0].proprioception["joint_positions"])
    # logger.info("Done moving robot to initial position.")

    if control_mode == control_types.position:
        key_name = "joint_positions"
    elif control_mode == control_types.cartesian:
        key_name = "gripper_pose"
        for entry in dataset.entries:
            position, orientation = entry.proprioception[key_name]
            entry.proprioception[key_name] = (
                np.asarray(position),
                np.asarray(orientation),
            )
    else:
        raise ValueError("invalid control mode")

    with robot.control(control_mode, asynchronous=True) as ctrl:

        for entry in track(
            dataset.entries, console=console, description="Replaying data"
        ):
            values = entry.proprioception[key_name]

            ctrl.update(values)

            # robot.gripper.set_pos(entry.proprioception["gripper_joint_positions"][0])

            time.sleep(period)


@dataset.command()
@filepath_option("--input-path", default="/tmp/demo_0000/")
def visualize(input_path):
    """
    Visualize a dataset
    """
    from robits.tools.visualize_dataset import cli as do_visualization

    do_visualization.callback(input_path, True, False)


@dataset.command()
@cli_options.robot()
@filepath_option("--output-path", default="/tmp/dataset")
def record(robot, output_path):
    """
    Record a dataset
    """
    # ..todo:: add kinesthetic teaching
    from robits.dataset.io.recorder import DatasetRecorder
    from robits.dataset.io.writer import DatasetWriter

    recorder = DatasetRecorder(robot)
    writer = DatasetWriter(output_path, recorder)

    try:
        with writer:
            input("Recording started. Please ENTER when done.")
            console.print(
                "Done recording dataset. Please wait until dataset is serialized."
            )
    except KeyboardInterrupt:
        logger.warning("Ctrl+C pressed. Please wait until dataset is serialized.")

    console.print("Done writing dataset.")


@dataset.command()
@filepath_option("--input-path", default="/tmp/demo_0000/")
def inspect(input_path):
    """
    Shows the metadata and other statstics of a dataset
    """
    from robits.dataset.io.reader import DatasetReader

    reader = DatasetReader(Path(input_path))
    dataset = reader.load(load_camera_images=False)

    console.print(json.dumps(dataset.metadata, indent=4))


@dataset.command()
@filepath_option("--input-path", default="/tmp/demo_0000/")
def stats(input_path):
    """
    Computes and shows statistics such as mean, std, min, max
    """
    from rich.table import Table

    from robits.dataset.io.stats_writer import StatsWriter

    stats_writer = StatsWriter(Path(input_path))

    if stats_writer.has_stats():
        console.print("Stats already computed loading existing stats")
    else:
        stats_writer.write_stats()
    stats = stats_writer.load_stats()

    console.rule("Stats")

    table = Table(title="Proprioception Stats")
    table.add_column("Name", justify="left", style="cyan", no_wrap=True)
    table.add_column("Mean", style="magenta")
    table.add_column("Std", style="magenta")
    table.add_column("Min", justify="right", style="green")
    table.add_column("Max", justify="right", style="green")
    table.add_column("25%", justify="right", style="yellow")
    table.add_column("50%", justify="right", style="yellow")
    table.add_column("75%", justify="right", style="yellow")

    for key, values in stats.items():
        table.add_row(
            key,
            f"{values.get('mean', ''):.3f}",
            f"{values.get('std', ''):.3f}",
            f"{values.get('min', ''):.3f}",
            f"{values.get('max', ''):.3f}",
            f"{values.get('25%', ''):.3f}",
            f"{values.get('50%', ''):.3f}",
            f"{values.get('75%', ''):.3f}",
        )
    console.print(table)
