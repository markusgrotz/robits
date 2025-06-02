import rich_click as click
from click_prompt import choice_option

from robits.core.config_manager import config_manager


def build_robot_from_param(ctx, param, value):
    from robits.core.factory import RobotFactory

    return RobotFactory(value).build()


def robot(*args, **kwargs):
    def decorator(f):
        return choice_option(
            "--robot-name",
            "robot",
            envvar="ROBITS_DEFAULT_ROBOT",
            callback=build_robot_from_param,
            type=click.Choice(config_manager.available_robots),
            help="Name of the robot configuration",
            *args,
            **kwargs
        )(f)

    return decorator
