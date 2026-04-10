#!/usr/bin/env python3


import time

import rich_click as click
from click_prompt import choice_option


from robits.core.factory import RobotFactory
from robits.core.config_manager import config_manager

from robits.vis.scene_visualizer import SceneVisualizer


@click.command()
@choice_option("--robot-name", type=click.Choice(config_manager.available_robots))
def cli(robot_name):
    robot = RobotFactory(robot_name).build()
    vis = SceneVisualizer(robot)

    vis.show()

    try:
        while vis.is_running:
            vis.update_scene()
            vis.update_pose(robot.eef_matrix)
            time.sleep(0.5)
    except KeyboardInterrupt:
        vis.close()


if __name__ == "__main__":
    cli()
