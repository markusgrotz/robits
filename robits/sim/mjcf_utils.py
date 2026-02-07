from typing import Optional
from typing import Sequence
from typing import List

import os
from importlib import import_module
import logging

import numpy as np

import mujoco
from dm_control import mjcf

from robits.sim.blueprints import RobotDescriptionModel
from robits.sim.blueprints import Pose


logger = logging.getLogger(__name__)


DUMMY_MODEL = mjcf.from_xml_string(
    '<mujoco><worldbody><body><geom type="box" rgba="1 0 0 1" size="0.05 0.05 0.05"/></body></worldbody></mujoco>'
)


def load_and_clean_model(blueprint: RobotDescriptionModel):
    model = load_model_from_blueprint(blueprint)
    remove_invalid_joints(model)
    remove_non_home_key(model)
    try:
        ensure_existing_home_key(model)
    except ValueError as e:
        logger.error(
            "Something went wrong while adding a homekey. Using simple heuristic.",
            exc_info=e,
        )
        k = model.keyframe.add("key", name="home")
        k.ctrl = np.zeros(len(model.find_all("actuator")))
        k.qpos = np.zeros(len(model.find_all("joint")))
    return model


def reload_model_with_assets(model):
    try:
        return mujoco.MjModel.from_xml_string(model.to_xml_string(), model.get_assets())
    except ValueError as ex:
        logger.error("Unable to reload model: %s", model, exc_info=ex)
        return model


def load_model_from_path(path_name, escape_separators=False):
    try:
        return mjcf.from_path(path_name, escape_separators)
    except Exception as ex:  # (FileNotFoundError, AttributeError, ValueError) as ex:
        logger.error("Unable to load model from path %s", path_name, exc_info=ex)
        return DUMMY_MODEL


def load_model_from_blueprint(blueprint: RobotDescriptionModel):
    logger.info("Loading model %s", blueprint)
    model = load_model_from_robot_descriptions(
        blueprint.description_name, blueprint.variant_name
    )
    if blueprint.model_prefix_name:
        model.namescope.name = blueprint.model_prefix_name
    logger.info("Namescope is %s", model.namescope.name)
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
    return load_model_from_path(model_path, True)


def remove_invalid_joints(model):
    for j in model.find_all("joint"):
        # or j.name == "floating_base_joint" or j.name == "freejoint" or
        if j.name is None or is_freejoint(j):
            logger.warning("Removing joint %s. Parent is %s", j.name, j.parent)
            for k in model.find_all("key"):
                k.qpos = k.qpos[7:]
            j.remove()


def has_freejoint(element: mjcf.Element) -> bool:
    for j in element.find_all("joint", immediate_children_only=True):
        if is_freejoint(j):
            return True
    return False


def is_freejoint(j: mjcf.Element) -> bool:
    if getattr(j, "tag", None) == "freejoint" or getattr(j, "type", None) == "free":
        return True
    return False


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
    for k in model.find_all("key"):
        if hasattr(k, "name") and k.name == "home":
            return model
    logger.warning("Unable to find a home key for model %s. Manually adding one", model)
    # this might fail, e.g. for panda_nohand.xml
    physics = mjcf.Physics.from_mjcf_model(model)
    k = model.keyframe.add("key", name="home")
    k.ctrl = np.zeros(physics.model.nu)
    k.qpos = np.zeros(physics.model.nq)
    return model


def remove_non_home_key(model):
    for k in model.find_all("key"):
        if not hasattr(k, "name") or k.name is None:
            logger.warning("Removing invalid key. Reason key has no name")
            k.remove()
        elif k.name != "home":
            logger.warning("Removing non home key with name %s", k.name)
            k.remove()
    return model


def update_joint_position(model, new_joint_position: Sequence[float]):
    q_len = len(new_joint_position)
    for k in model.find_all("key"):
        max_len = min(len(k.ctrl), q_len)
        if len(k.ctrl) != q_len:
            logger.error(
                "Key %s has invalid length. Expected %d, but key has %d values.",
                k,
                q_len,
                len(k.ctrl),
            )
            logger.error("model is %s", model)
            # continue
        k.qpos[:max_len] = np.asarray(new_joint_position)[:max_len]
        k.ctrl[:max_len] = np.asarray(new_joint_position)[:max_len]


def set_pose(element: mjcf.Element, pose: Optional[Pose] = None):
    """
    .. seealso:: :func:`add_offset_pose`
    """
    if pose is None:
        return

    if (
        element.quat is not None
        or element.euler is not None
        or element.axisangle is not None
    ):
        logger.warning(
            "Element orientation already set for element %s. Overwriting previous orientation",
            element,
        )
        element.quat = None
        element.axisangle = None
        element.euler = None
    if hasattr(element, "pos") and element.pos is not None:
        logger.warning(
            "Element position already set for element %s. Overwriting previous position.",
            element,
        )

    element.pos = pose.position
    element.quat = pose.quaternion_wxyz


def add_offset_pose(element: mjcf.Element, offset: Optional[Pose] = None):
    if offset is None:
        return

    prev_pose = pose_from_element(element)
    new_pose = Pose(prev_pose.matrix.dot(offset.matrix))
    element.quat = None
    element.axisangle = None
    element.euler = None
    element.pos = new_pose.position
    element.quat = new_pose.quaternion_wxyz


def pose_from_element(element: mjcf.Element, use_degrees: bool = True) -> Pose:
    pose = Pose()
    if hasattr(element, "pos") and element.pos is not None:
        pose = pose.with_position(element.pos)
    if hasattr(element, "quat") and element.quat is not None:
        pose = pose.with_quat_wxyz(element.quat)
    elif hasattr(element, "xyaxes") and element.xyaxes is not None:
        x = element.xyaxes[:3]
        y = element.xyaxes[3:]
        x = x / np.linalg.norm(x)
        y = y / np.linalg.norm(y)
        m = np.identity(4)
        m[:3, 0] = x
        m[:3, 1] = y
        m[:3, 2] = np.cross(x, y)
        if abs(np.linalg.det(m) - 1.0) > 1e-3:
            logger.warning("Not a rotation matrix %s. Det is %f", m, np.linalg.det(m))
            u, _s, vt = np.linalg.svd(m[:3, :3], full_matrices=True)
            m[:3, :3] = u.dot(vt)
            if np.linalg.det(m) < 1e-3:
                u[:, -1] *= -1.0
                m[:3, :3] = u.dot(vt)
            logger.info("New rotation matrix is %s", m)
        pose = Pose(m).with_position(pose.position)
    elif hasattr(element, "euler") and element.euler is not None:
        logger.warning(
            "Not fully implemented. Check if there is a compile flag that specifies the units. Something like # <compiler angle=degree ..."
        )
        pose = pose.with_euler(element.euler, degrees=use_degrees)
    return pose


def set_object_pose(data: mujoco.MjData, object_name: str, pose: Pose, relative=False):
    joint_name = f"{object_name}_joint"

    joint_id = mujoco.mj_name2id(data.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint '{joint_name}' not found")

    qpos_adr = data.model.jnt_qposadr[joint_id]

    pose_vec = np.concatenate([pose.position, pose.quaternion_wxyz])

    if relative:
        data.qpos[qpos_adr : qpos_adr + 7] += pose_vec
    else:
        data.qpos[qpos_adr : qpos_adr + 7] = pose_vec


def get_home_key_qpos(root: mjcf.RootElement) -> List[float]:
    if not hasattr(root, "keyframe"):
        logger.warning("Unable to find a home keyframe.")
        return []
    for key in root.keyframe.find_all("key"):
        if key.name == "home":
            return key.ctrl.tolist()
    return []
