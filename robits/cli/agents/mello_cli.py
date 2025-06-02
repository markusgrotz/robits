#!/usr/bin/env python3
import time

import rich_click as click
from click_prompt import filepath_option


from robits.cli import cli_options

from robits.cli.cli_utils import setup_cli

from robits.core.abc.control import control_types

from robits.agents.mello_agent import MelloAgent

import logging

logger = logging.getLogger(__name__)


@click.command()
@filepath_option("--device-addr", default="http://172.28.7.147:80/")
@cli_options.robot()
# @filepath_option("--output-path", default="/tmp/test")
def cli(device_addr, robot):

    agent = MelloAgent(device_addr, robot)

    robot.control.move_home()

    # agent.wait_for_pose()

    prev_joint_positions = agent.get_mello_joint_positions()
    with robot.control(control_types.position, asynchronous=False) as ctrl:
        try:
            while True:
                # agent.get_action()
                # joint_positions = agent.get_mello_joint_positions()
                joint_positions, joint_velocities = agent.get_mello_data()
                logger.info(
                    "Joint positions: %s, Joint velocities: %s",
                    joint_positions,
                    joint_velocities,
                )
                delta = joint_positions - prev_joint_positions
                # ctrl.update(joint_positions)
                ctrl.update(delta, relative=True)
                prev_joint_positions = joint_positions
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("keyboard interrupt")


if __name__ == "__main__":
    setup_cli()
    cli()
