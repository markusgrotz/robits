from typing import Union

import sys
import logging

import numpy as np

import questionary

from rich.console import Console
from rich.logging import RichHandler

from robits.core import __version__


logger = logging.getLogger(__name__)

console = Console()

arm_actions = ["execute", "skip", "quit"]
hand_actions = ["execute", "skip", "quit"]


def prompt_for_action(robot, action, use_hands=True, num_attempts=10):
    # plan = robot.plan_arm_trajectory(action)

    action_mode = questionary.select(
        "What should I do with the arms ?",
        arm_actions,
        default="skip",
        use_shortcuts=True,
    ).ask()

    if action_mode == "quit":
        sys.exit(0)

    elif action_mode == "execute":
        robot.control_arm(action)
        robot.control_hand(action)

        if False and use_hands:
            action_mode = questionary.select(
                "What should I do with the hands?",
                hand_actions,
                default="execute",
                use_shortcuts=True,
            ).ask()
            if action_mode == "execute":
                robot.control_hand(action)
            if action_mode == "quit":
                sys.exit(0)

        return

    elif action_mode == "skip":
        return


def setup_cli(level: Union[str, int] = logging.INFO):
    log_config = {
        "handlers": [
            RichHandler(
                console=console, show_path=True, markup=True, rich_tracebacks=True
            ),
            # AudioHandler()
        ]
    }
    logging.basicConfig(level=level, **log_config)
    np.set_printoptions(precision=4, suppress=True)

    logger.info("Current package version is %s", __version__)
