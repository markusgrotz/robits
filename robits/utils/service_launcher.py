#!/usr/bin/env python3

import sys

from click_prompt import choice_argument
import rich_click as click

from robits.core.config_manager import config_manager

from robits.core.factory import CameraFactory
from robits.core.factory import RobotFactory
from robits.core.factory import GripperFactory

from robits.remote.server.server_base import ZMQServerBase
from robits.sim.env_client import MujocoEnvClient

available_services = config_manager.list()

MAGIC_CMD_ARG = "ROBITS_CMD_ED1hV3pHOIOoc"


@click.command()
@click.option("--magic-string")
@choice_argument("config-name", type=click.Choice(available_services))
def cli(magic_string, config_name):
    if magic_string != MAGIC_CMD_ARG:
        print("Please run with magic-string argument")
        sys.exit(-1)
    if "camera_" in config_name:
        service_instance = CameraFactory(config_name).build()
        port = 5060
    elif "robot_" in config_name:
        service_instance = RobotFactory(config_name).build()
        port = 5050
    elif "gripper_" in config_name:
        service_instance = GripperFactory(config_name).build()
        port = 5070
    else:
        raise NotImplementedError("Not implemented yet.")

    if isinstance(service_instance, MujocoEnvClient):
        # Access the enviroment and trigger a build of the environment
        service_instance.env

    service = ZMQServerBase(service_instance, port)
    service.listen()


if __name__ == "__main__":
    cli()
