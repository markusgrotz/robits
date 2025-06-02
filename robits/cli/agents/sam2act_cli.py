#!/usr/bin/env python3

import sys
import logging
import time


import numpy as np

from click_prompt import choice_option
import rich_click as click
from click_prompt import filepath_option


import torch


from robits.cli import cli_options

from robits.core.data_model.action import CartesianAction
from robits.core.data_model.camera_capture import CameraData

from robits.vis.scene_visualizer import SceneVisualizer


logger = logging.getLogger(__name__)

execution_mode = ["step", "auto", "vis", "save"]


used_intructions = [
    "push the red button and then the blue button",
    "close and open the same drawer",
    "put the blue in drawer",
    "put the orange in drawer",
]


@torch.inference_mode()
@click.command()
@filepath_option("--model-path", default="/home/markus/sam2act_models/")
@choice_option("--execution-mode", type=click.Choice(execution_mode))
@cli_options.robot()
@click.option("--use-audio/--no-audio", default=False, is_flag=True)
@choice_option("--instruction", type=click.Choice(used_intructions))
def cli(model_path, execution_mode, robot, use_audio, instruction):

    from robits.cli import cli_utils

    cli_utils.setup_cli()

    from robits.agents.sam2act_agent import SAM2Act as Agent

    episode_length = 25

    agent = Agent(model_path)
    agent.lang_goal = "push the buttons in the following order: red, green, blue."

    if instruction:
        agent.lang_goal = instruction

    if use_audio:
        with robot.audio as t:
            input("Speak and then press enter when done")
        x = t.process()
        logger.info("Transcribed %s", x)
        agent.lang_goal = x
    elif False:
        print("language goal. Press enter to accept or type a new one:")
        x = input(f"{agent.lang_goal}")
        if x:
            agent.lang_goal = x

    logger.info("Language goal is %s", agent.lang_goal)

    vis = SceneVisualizer(robot)
    if execution_mode == "save":
        vis.set_output("/tmp/vis")
    else:
        vis.show()

    # while not robot.is_ready():
    #    time.sleep(1)

    def process_action(action) -> None:
        action = np.asarray(action)
        action = CartesianAction.parse(action)

        logger.info("Action from agent %s", action)
        vis.update_action(action)

        if execution_mode == "auto":
            if robot.control_arm(action):
                robot.control_hand(action)
        elif execution_mode == "step":
            cli_utils.prompt_for_action(robot, action)
        elif execution_mode == "vis":
            pass

    try:
        for i in range(episode_length):

            start_time = time.time()

            obs = robot.get_joint_obs()
            camera_data = robot.get_vision_data(
                include_point_cloud=True, swap_channels=True
            )
            obs.update(camera_data)

            logger.debug("getting scene")
            # vis.update_scene()
            camera = robot.cameras[0]
            camera_name = camera.camera_name
            vis.update_scene(
                CameraData(
                    rgb_image=camera_data[f"{camera_name}_rgb"].transpose((2, 1, 0)),
                    depth_image=camera_data[f"{camera_name}_depth"],
                ),
                camera,
            )
            logger.debug("done getting scene")

            for camera in robot.cameras:
                obs[f"{camera.camera_name}_camera_extrinsics"] = np.linalg.inv(
                    obs[f"{camera.camera_name}_camera_extrinsics"]
                )

            observation = agent.prepare_observation(obs, i, episode_length)
            action = agent.get_action(None, observation)
            process_action(action.action)

            elapsed_time = time.time() - start_time
            logger.info("Processing action took %.2f seconds", elapsed_time)

            time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        pass

    vis.close()

    logger.info("Done.")

    sys.exit(0)


if __name__ == "__main__":
    cli()
