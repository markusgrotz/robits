from typing import Optional
from typing import Sequence

import os
from importlib import import_module
import logging

import numpy as np

import mujoco
from dm_control import mjcf

from robits.sim.blueprints import RobotDescriptionModel
from robits.sim.blueprints import Pose


logger = logging.getLogger(__name__)


def load_and_clean_model(blueprint: RobotDescriptionModel):
    model = load_model_from_blueprint(blueprint)
    remove_invalid_joints(model)
    remove_non_home_key(model)
    ensure_existing_home_key(model)
    return model

def reload_model_with_assets(model):
    return mujoco.MjModel.from_xml_string(model.to_xml_string(), model.get_assets())

def load_model_from_path(path_name, escape_separators=False):
    return mjcf.from_path(path_name, escape_separators)


def load_model_from_blueprint(blueprint: RobotDescriptionModel):
    logger.info("Loading model %s", blueprint)
    model = load_model_from_robot_descriptions(
        blueprint.description_name, blueprint.variant_name
    )
    if blueprint.model_prefix_name:
        model.namescope.name = blueprint.model_prefix_name
    logger.info("namescope is %s", model.namescope.name)
    return model


def load_model_from_robot_descriptions(
    description_name: str, variant_name: Optional[str] = None
):
    """
    Loads a mujoco model from robot_description package

    :param description_name: the name of the description package
    :param variant_name: the name of the variant. Usually hand.xml
    """
    module = import_module(f"robot_descriptions.{description_name}")
    model_path = module.MJCF_PATH
    if variant_name:
        model_path = os.path.join(os.path.dirname(model_path), variant_name)

    logger.debug("Model path is %s", model_path)
    return mjcf.from_path(model_path, escape_separators=False)


def remove_invalid_joints(model):
    for j in model.find_all("joint"):
        if j.name is None or j.name == "floating_base_joint" or j.name == "freejoint":
            logger.warning("Removing invalid joint from model. Parent is %s", j.parent)
            for k in model.find_all("key"):
                k.qpos = k.qpos[7:]
            j.remove()

# can this be replaced?
def merge_home_keys(arm_model, gripper_model):
    # merges the home key of the arm and the gripper
    # see `https://github.com/google-deepmind/mujoco_menagerie/blob/main/FAQ.md`
    arm_key = arm_model.find("key", "home")
    if arm_key:
        gripper_key = gripper_model.find("key", "home")

        if gripper_key is None:
            physics = mjcf.Physics.from_mjcf_model(gripper_model)
            arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
            arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
        else:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, gripper_key.ctrl])
            arm_key.qpos = np.concatenate([arm_key.qpos, gripper_key.qpos])
            gripper_key.remove()
    else:
        logger.warning("No home key found.")


def ensure_existing_home_key(model):
    physics = mjcf.Physics.from_mjcf_model(model)
    for k in model.find_all("key"):
        if hasattr(k, "name") and k.name == "home":
            return model
    logger.error(
        "Unable to find a home key for model %s. Manually adding one", model
    )
    k = model.keyframe.add("key", name="home")
    k.ctrl = np.zeros(physics.model.nu)
    k.qpos = np.zeros(physics.model.nq)

    return model


def remove_non_home_key(model):
    for k in model.find_all("key"):
        if not hasattr(k, "name") or k.name is None:
            logger.warning("Removing invalid key. Reason key has no name")
            k.remove()
        elif "home" not in k.name:
            logger.warning("Removing non home key with name %s", k.name)
            k.remove()
    return model

def update_joint_position(model, new_joint_position: Sequence[float]):
    q_len = len(new_joint_position)
    for k in model.find_all("key"):
        if len(k.ctrl) != q_len:
            logger.warning("Joint has %s invalid length. Expected %d was %d.", k, q_len, len(k.ctrl))
        k.qpos[:q_len] = np.asarray(new_joint_position)
        k.ctrl[:q_len] = np.asarray(new_joint_position)

def set_pose(element, pose: Optional[Pose] = None):
    if pose is None:
        return
    if (
        element.quat is not None
        or element.euler is not None
        or element.axisangle is not None
    ):
        logger.error(
            "Element orientation already set for element %s. Discarding stored information.",
            element,
        )
        element.quat = None
        element.axisangle = None
        element.euler = None
    element.pos = pose.position
    element.quat = pose.quaternion_wxyz