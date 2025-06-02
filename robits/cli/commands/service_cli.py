"""
Service commands
"""

import subprocess

from click_prompt import choice_argument
import rich_click as click

from robits.core.config_manager import config_manager

from robits.cli.base_cli import cli
from robits.cli.base_cli import console


def get_running_services():
    """ """
    from robits.utils.process_utils import ProcessMonitor

    monitor = ProcessMonitor()
    services = [config_name for config_name in monitor.list()]
    return services or ["None"]


available_services = config_manager.list()


@cli.group()
def service():
    """
    Commands related to starting/stoping a service

    This command group provides operations for interacting with a service.
    A service is, for example, a remote instance running on a different machine.
    """
    pass


@service.command()
@choice_argument("config-name", type=click.Choice(available_services))
def start(config_name):
    """
    Start a configuration as service.
    """
    from robits.utils import service_launcher

    console.rule("Starting service")
    cmd = f"{service_launcher.__file__} --magic-string {service_launcher.MAGIC_CMD_ARG} {config_name}"

    subprocess.call(cmd, shell=True)
    console.rule("Done.")


@service.command()
@choice_argument("config-name", type=click.Choice(available_services))
def stop(config_name):
    """
    Stops a given service
    """
    from robits.utils.process_utils import ProcessMonitor

    monitor = ProcessMonitor()
    for p in monitor.terminate(config_name):
        console.print(p)
    for p in monitor.kill(config_name):
        console.print(p)


@service.command()
@choice_argument("config-name", type=click.Choice(available_services))
def restart(config_name):
    """
    Restarts a service
    """
    stop(config_name)
    start(config_name)


@service.command()
def status():
    """
    Lists running services and their status
    """
    from robits.utils.process_utils import ProcessMonitor

    monitor = ProcessMonitor()
    for p in monitor.status():
        console.print(p)
