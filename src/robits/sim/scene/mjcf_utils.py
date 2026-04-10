from typing import Optional
from typing import Sequence
from typing import List

import os
import logging
from pathlib import Path
from importlib import import_module

import numpy as np

import mujoco

from robits.sim.blueprints import RobotDescriptionModel
from robits.sim.blueprints import Pose


logger = logging.getLogger(__name__)


DUMMY_MODEL_STR = """
<mujoco>
    <worldbody>
        <body>
            <geom type="box" rgba="1 0 0 1" size="0.05 0.05 0.05"/>
        </body>
    </worldbody>
</mujoco>"""

DEFAULT_WORLD_STR = """
<mujoco>
    <visual>
        <headlight diffuse=".5 .5 .5" specular="1 1 1"/>
    </visual>

    <asset>
        <texture type="skybox" builtin="gradient" rgb1=".5 .5 .5" rgb2="0 0 0" width="10" height="10"/>
        <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="1 1 1" rgb2="1 1 1" markrgb="0 0 0" width="300" height="300"/>
        <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.3"/>
    </asset>

    <worldbody>
        <geom name="floor" size="5 5 0.01" type="plane" material="groundplane"/>
        <light pos="0 0 3" diffuse="1 1 1" specular="1 1 1"/>
    </worldbody>
</mujoco>"""


def str_to_mujoco_geom_type(type_name: str) -> int:
    str_to_geom_type = {
        "box": mujoco.mjtGeom.mjGEOM_BOX,
        "ellipsoid": mujoco.mjtGeom.mjGEOM_ELLIPSOID,
        "mesh": mujoco.mjtGeom.mjGEOM_MESH,
        "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE,
        "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
        "plane": mujoco.mjtGeom.mjGEOM_PLANE,
        "sphere": mujoco.mjtGeom.mjGEOM_SPHERE,
    }
    return str_to_geom_type[type_name]


def load_and_clean_spec(blueprint: RobotDescriptionModel):
    spec = load_spec_from_blueprint(blueprint)
    remove_invalid_joints(spec)
    load_assets_into_spec(spec, blueprint)

    spec = mujoco.MjSpec.from_string(spec.to_xml(), assets=spec.assets)
    return spec


def load_assets_into_spec(spec: mujoco.MjSpec, blueprint: RobotDescriptionModel):
    model_path = get_robot_descriptions_model_path(blueprint)
    asset_dir = Path(model_path).parent / spec.meshdir

    for mesh in spec.meshes:
        if mesh.file and mesh.file not in spec.assets:
            p = asset_dir / mesh.file
            spec.assets[mesh.file] = p.read_bytes()

    # TODO use sepc.texturedir
    for texture in spec.textures:
        if texture.file and texture.file not in spec.assets:
            p = asset_dir / texture.file
            spec.assets[texture.file] = p.read_bytes()


def get_robot_descriptions_model_path(blueprint: RobotDescriptionModel) -> str:
    description_name = blueprint.description_name
    variant_name = blueprint.variant_name

    module = import_module(f"robot_descriptions.{description_name}")
    model_path = module.MJCF_PATH
    if variant_name:
        model_path = os.path.join(os.path.dirname(model_path), variant_name)
    logger.debug("Model path is %s", model_path)

    return model_path


def load_spec_from_blueprint(blueprint: RobotDescriptionModel):
    logger.info("Loading model %s", blueprint)

    model_path = get_robot_descriptions_model_path(blueprint)

    if blueprint.model_prefix_name:
        logger.warning("Not implemented yet.")

    return mujoco.MjSpec.from_file(model_path)


def load_model_from_path(path_name, escape_separators=False):
    try:
        spec = mujoco.MjSpec.from_file(path_name)
        if escape_separators:
            spec = fix_element_names(spec)
        return spec.compile()
    except Exception as ex:  # (FileNotFoundError, AttributeError, ValueError) as ex:
        logger.error("Unable to load model from path %s", path_name, exc_info=ex)
        return mujoco.MjSpec.from_string(DUMMY_MODEL_STR)


def fix_element_names(spec: mujoco.MjSpec) -> mujoco.MjSpec:
    def remove_invalid_chars(s: str) -> str:
        s = s.replace(" ", "_")
        s = s.replace("//", "_")
        return s

    for obj_list in [
        spec.bodies,
        spec.geoms,
        spec.joints,
        spec.sites,
        spec.cameras,
        spec.lights,
        spec.meshes,
        spec.materials,
        spec.sensors,
        # spec.excludes,
        # spec.equalities,
        # spec.tendons,
        # spec.actuators
    ]:
        for obj in obj_list:
            if obj.name:
                obj.name = remove_invalid_chars(obj.name)
            if hasattr(obj, "material") and obj.material is not None:
                obj.material = remove_invalid_chars(obj.material)
            if hasattr(obj, "meshname") and obj.meshname is not None:
                obj.meshname = remove_invalid_chars(obj.meshname)
    return spec


def remove_invalid_joints(spec):
    for j in spec.joints:
        if is_freejoint(j):  # j.name is None or
            logger.warning("Removing joint %s. Parent is %s", j.name, j.parent)
            spec.delete(j)
            for k in spec.keys:
                k.qpos = np.asarray(k.qpos)[7:]
            spec.compile()


def has_freejoint(element: mujoco.MjsElement) -> bool:
    for j in element.joints:
        if is_freejoint(j):
            return True
    return False


def is_freejoint(j: mujoco._specs.MjsJoint) -> bool:
    return j.type == mujoco.mjtJoint.mjJNT_FREE


def update_home_joint_position(spec, new_joint_position: Sequence[float]) -> None:

    k = spec.key("home")
    if not k:
        logger.warning("Model does not have a home key. Manually adding one")
        model = spec.compile()
        k = spec.add_key(name="home", qpos=np.zeros(model.nq), ctrl=np.zeros(model.nu))

    q_len = len(new_joint_position)
    qpos = np.asarray(k.qpos)
    ctrl = np.asarray(k.ctrl)

    if len(qpos) != q_len:
        logger.error(
            "Key %s has invalid length. Expected %d, but key has %d values.",
            k.name,
            q_len,
            len(qpos),
        )
    max_len = min(len(k.ctrl), q_len)
    # FIXME this is actually not correct as we need the mapping between actuators and joints
    qpos[:max_len] = np.asarray(new_joint_position)[:max_len]
    ctrl[:max_len] = np.asarray(new_joint_position)[:max_len]

    k.qpos = qpos
    k.ctrl = ctrl

    spec.compile()


def set_pose(element: mujoco.MjsFrame, pose: Optional[Pose] = None):
    """
    .. seealso:: :func:`add_offset_pose`
    """
    if pose is None:
        return

    if not np.allclose(element.quat, [1, 0, 0, 0], rtol=0.0):
        logger.warning(
            "Element orientation already set for element %s. Overwriting previous orientation",
            element,
        )
    if not np.allclose(element.pos, [0, 0, 0], rtol=0.0):
        logger.warning(
            "Element position already set for element %s. Overwriting previous position.",
            element,
        )

    element.pos = pose.position
    element.quat = pose.quaternion_wxyz


def add_offset_pose(element: mujoco.MjsFrame, offset: Optional[Pose] = None):
    if offset is None:
        return

    prev_pose = pose_from_element(element)
    new_pose = Pose(prev_pose.matrix.dot(offset.matrix))
    element.pos = new_pose.position
    element.quat = new_pose.quaternion_wxyz


def pose_from_element(
    element: mujoco.MjsElement, compiler: mujoco.MjsCompiler = None
) -> Pose:

    if element.alt.type != mujoco.mjtOrientation.mjORIENTATION_QUAT:
        logger.warning(
            "Not fully supported yet. Element %s has %s ",
            element.name,
            element.alt.type,
        )

        spec = mujoco.MjSpec()
        compiler = spec.compiler

        quat_wxyz = mujoco.MjSpec.resolve_orientation(
            degree=compiler.degree,
            sequence=compiler.eulerseq,
            orientation=element.alt,
        )
    else:
        quat_wxyz = element.quat
    return Pose().with_position(element.pos).with_quat_wxyz(quat_wxyz)


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


def get_home_key_qpos(spec: mujoco.MjSpec) -> List[float]:
    k = spec.key("home")
    if k:
        return list(k.ctrl)
    return []
