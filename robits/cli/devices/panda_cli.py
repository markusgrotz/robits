from robits.cli.base_cli import cli
from robits.cli.cli_utils import console

from robits.core.config_manager import config_manager


@cli.group()
def panda():
    """
    Franka Panda related commands
    """
    pass


@panda.command()
def unlock():
    """
    Unlocks the breaks
    """
    from robits.real.franka.franka_web_client import FrankaWebClient

    main_config = config_manager.get_main_config()
    config = main_config.kwargs.get("franka_web", {})
    ip_addr = config.get("ip_addr", "172.16.0.2")
    user = config.get("user", "user")
    password = config.get("password", "password")

    console.print(f"Connecting to {ip_addr} with user {user}.")

    with FrankaWebClient(ip_addr, user=user, password=password) as robot:
        robot.unlock_brakes()

    console.rule("Done")


@panda.command()
def lock():
    """
    Locks the breaks
    """
    from robits.real.franka.franka_web_client import FrankaWebClient

    main_config = config_manager.get_main_config()
    config = main_config.kwargs.get("franka_web", {})
    ip_addr = config.get("ip_addr", "172.16.0.2")
    user = config.get("user", "user")
    password = config.get("password", "password")

    console.print(f"Connecting to {ip_addr} with user {user}.")

    with FrankaWebClient(ip_addr, user=user, password=password) as robot:
        robot.lock_brakes()

    console.rule("Done")


if __name__ == "__main__":
    cli()
