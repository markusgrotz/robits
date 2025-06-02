#!/usr/bin/env python3

import json

import matplotlib.pyplot as plt

import rich_click as click
from click_prompt import choice_option


from robits.core.factory import RobotFactory
from robits.core.config_manager import config_manager


@click.group()
def cli():
    pass


@cli.command()
@choice_option("--robot-name", type=click.Choice(config_manager.available_robots))
def prompt(robot_name):

    robot = RobotFactory().build_robot(robot_name)

    camera = robot.cameras[0]

    for i in range(50):
        camera_data, meta_info = camera.get_camera_data()

    from robits.vlm.openai_vlm import ChatGPT
    from robits.vlm.openai_vlm import PromptBuilder

    prompt = PromptBuilder()

    prompt.add_instruction(
        """You are a robot. Your task is push the buttons in
            the following order: red, green, blue. Which object  do you want to
            grasp next?  Output a single action and a description. 
            The action should include the image coordinates for the gripper. This should be the location where the object is being grasped. So be previse.
            Format the output with json, the elements
            should be description, coordinates as a tuple.
            """
    )

    # and whether the gripper is open or closed.
    # , gripper_state
    # prompt.add_instruction("this is the output of your previous actions: {'desired_object': 'green button', 'coordinates': {'x': 200, 'y': 300}}, {'desired_object': 'blue button', 'coordinates': {'x': 300, 'y': 300}}, {'desired_object': 'red button', 'coordinates': {'x': 400, 'y': 300}}")

    image = prompt.add_image(camera_data.rgb_image, resize=False)

    vlm = ChatGPT()
    result = vlm.query(prompt)

    print(result)
    # print(result.__dict__)
    content = result.choices[0].message.content

    response = json.loads(content[7:-3])

    print(response)

    x, y = response["coordinates"]
    # y = response['coordinates']['y']

    plt.imshow(image)

    plt.plot(x, y, "ro")
    plt.show()


if __name__ == "__main__":
    cli()
