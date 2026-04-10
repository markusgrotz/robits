"""
Main entry point for the RoBits command-line interface.

This module imports and assembles all CLI command groups, making them available
through the 'rb' entry point. New command modules should be imported here to
be included in the CLI.
"""

from robits.cli.base_cli import cli

# Import all command modules to register them with the CLI
import robits.cli.devices.robot_cli
import robits.cli.devices.gripper_cli
import robits.cli.devices.camera_cli
import robits.cli.commands.dataset_cli
import robits.cli.commands.speech_cli
import robits.cli.devices.panda_cli
import robits.cli.commands.move_cli
import robits.cli.commands.config_cli
import robits.cli.commands.service_cli  # noqa: F401

if __name__ == "__main__":
    cli()
