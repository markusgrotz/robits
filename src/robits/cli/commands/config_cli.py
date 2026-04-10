"""
Configuration commands for managing RoBits configurations.

This module provides commands for listing and displaying configuration files
that define robots, grippers, cameras, and other system components. The configuration
system allows customizing component parameters through JSON files.
"""

import os
import json
import subprocess
from pathlib import Path

import rich_click as click
from click_prompt import choice_argument
import questionary

from robits.core.config_manager import config_manager

from robits.cli.base_cli import cli
from robits.cli.base_cli import console


@cli.group()
def config():
    """
    Commands for viewing and managing system configurations.

    This command group provides operations for listing available configurations
    and viewing their content. Configurations in RoBits are stored as JSON files
    and define properties for robots, grippers, cameras, and other components.

    Custom configurations can be placed in a directory specified by the
    ROBITS_CONFIG_DIR environment variable.
    """
    pass


@config.command()
def list():
    """
    List all available configurations organized by type.

    Displays a table of all available configuration files in the system,
    grouped by their type (robot, gripper, camera, etc.) and showing their
    file paths. This includes both built-in configurations and any user
    configurations defined in ROBITS_CONFIG_DIR.
    """
    from rich.table import Table

    config_types = [
        "robot",
        "gripper",
        "camera",
        "audio",
        "speech",
        "camera calibration",
    ]

    available_configs = [
        config_manager.available_robots,
        config_manager.available_grippers,
        config_manager.available_cameras,
        config_manager.available_audio_backends,
        config_manager.available_speech_backends,
        config_manager.available_camera_calibrations,
    ]

    table = Table(title="Configs")
    table.add_column("Name", justify="left", style="cyan", no_wrap=True)
    table.add_column("Type", style="magenta")
    table.add_column("Path", justify="left", style="green")

    for config_type, available_configs in zip(config_types, available_configs):
        for config_name in available_configs:
            config_path = str(config_manager.get_config_path(config_name))
            table.add_row(config_name, config_type, config_path)
        table.add_section()
    console.print(table)


@config.command()
@choice_argument("config-name", type=click.Choice(config_manager.list()))
def show(config_name):
    """
    Display the contents of a specific configuration file.

    Shows the full JSON content of the specified configuration file
    with proper formatting. Also displays the file path of the configuration.

    :param config_name: The name of the configuration to display.
    """
    dict = config_manager.load_dict(config_name)

    console.print()
    console.rule("Config")
    config_path = config_manager.get_config_path(config_name)
    console.print(f"[b]Path[/b]: {config_path}")

    console.print(json.dumps(dict, indent=4))


@config.command()
@choice_argument("config-name", type=click.Choice(config_manager.list()))
def copy(config_name):
    """
    Copy an existing config to a user configuration
    """
    import shutil

    if not (user_config_dir := os.environ.get("ROBITS_CONFIG_DIR")):
        raise EnvironmentError(
            "Unable to find user directory please set `ROBITS_CONFIG_DIR`. E.g. export ROBITS_CONFIG_DIR=~/robits_config/"
        )

    if not os.path.isdir(user_config_dir):
        raise ValueError(f"ROBITS_CONFIG_DIR={user_config_dir} is not a dir.")

    config_path = config_manager.get_config_path(config_name)
    config_type, *other = config_path.stem.split("_")
    config_name = "_".join(other)
    config_name = questionary.text(
        f"Name of the new config? (Don't include the prefix {config_type}_)",
        default=f"{config_name}_new",
    ).ask()

    if not config_name:
        raise ValueError("No config name provided")
    new_config_path = (
        Path(user_config_dir) / f"{config_type}_{config_name}{config_path.suffix}"
    )

    if new_config_path.exists():
        raise ValueError(f"New config path {new_config_path} already exists.")

    if questionary.confirm(f"Copy {config_path} to {new_config_path}?").ask():
        shutil.copyfile(config_path, new_config_path)


@config.command()
@choice_argument("config-name", type=click.Choice(config_manager.list()))
def edit(config_name):
    """
    Opens an existing configuration with the system editor

    $EDITOR variable must be set for this
    """
    config_path = config_manager.get_config_path(config_name)
    editor_cmd = os.environ.get("EDITOR") or "vim"
    subprocess.call(f"{editor_cmd} {config_path}", shell=True)
