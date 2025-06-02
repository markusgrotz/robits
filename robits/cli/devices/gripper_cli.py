import rich_click as click
from click_prompt import choice_option


from robits.cli.base_cli import cli
from robits.cli.base_cli import console

from robits.core.factory import GripperFactory
from robits.core.config_manager import config_manager


@cli.group()
def gripper():
    """
    Gripper related commands
    """
    pass


@gripper.command()
@choice_option("--gripper-name", type=click.Choice(config_manager.available_grippers))
def close(gripper_name):
    """
    Closes the gripper
    """
    gripper = GripperFactory(gripper_name).build()
    gripper.close()


@gripper.command()
@choice_option("--gripper-name", type=click.Choice(config_manager.available_grippers))
def open(gripper_name):
    """
    Opens the gripper
    """
    gripper = GripperFactory(gripper_name).build()
    gripper.open()


@gripper.command()
@choice_option("--gripper-name", type=click.Choice(config_manager.available_grippers))
def info(gripper_name):
    """
    Displays info about the gripper
    """
    gripper = GripperFactory(gripper_name).build()
    console.print(gripper.get_info())


@gripper.command()
@choice_option("--gripper-name", type=click.Choice(config_manager.available_grippers))
def shell(gripper_name):
    """
    Launches an interactive shell with the gripper.

    Useful for debugging use the gripper variable to access the gripper
    """
    gripper = GripperFactory(gripper_name).build()  # noqa: F841
    from IPython import embed

    embed()
