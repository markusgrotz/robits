from typing import Tuple
from typing import List
from typing import Optional
from typing import Union
from typing import Dict


import logging
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

import mujoco

from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import MeshBlueprint
from robits.sim.blueprints import Pose
from robits.sim.blueprints import Attachment
from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import BlueprintGroup

from robits.utils import camera_intrinsics
from robits.sim import mjcf_utils

from robits.sim.converters.robot_heuristics import get_all_heuristics_classes
from robits.sim.converters.gripper_heuristics import get_all_gripper_heuristics_classes


logger = logging.getLogger(__name__)

FREE_JOINT_LEN = 7


def _normalize_full_name(name: str) -> str:
    if not name:
        return "/"
    name = name.replace("\\", "")
    if not name.startswith("/"):
        name = f"/{name}"
    name = name.replace("//", "/")
    if len(name) > 1 and name.endswith("/"):
        name = name[:-1]
    return name


def _join_full_name(parent: Optional[str], child: str) -> str:
    parent = parent or ""
    combined = f"{parent}/{child}"
    return _normalize_full_name(combined)


def load_mjcf_as_blueprints(model_path: Union[str, Path]) -> List[Blueprint]:
    return MujocoXMLImporter(model_path).parse()


class MujocoXMLImporter:
    def __init__(self, model_path: Union[str, Path]) -> None:
        model_path = Path(model_path).expanduser().resolve()

        self.spec = mujoco.MjSpec.from_file(str(model_path))
        self.asset_dir = model_path.parent / self.spec.meshdir

        # TODO recompile so that we get rid of euler and other tags, such as fromto
        self.spec = mujoco.MjSpec.from_string(
            self.spec.to_xml(), assets=self.spec.assets
        )

        logger.info("Using heuristic to detect a robot. This is not fully implemented.")
        self.robot_heuristic_classes = get_all_heuristics_classes()
        self.gripper_heuristic_classes = get_all_gripper_heuristics_classes()

        self.seq = 0

    def get_top_level_parent(
        self, element: mujoco.MjsElement
    ) -> Optional[mujoco.MjsElement]:

        if element is self.spec.worldbody:
            return None

        current = element
        while current.parent:
            if current.parent is self.spec.worldbody:
                return current
            current = current.parent
        return None

    def parse(self) -> List[Blueprint]:

        joint_positions = mjcf_utils.get_home_key_qpos(self.spec)

        blueprints = []
        for element_tags in self.spec.cameras:
            blueprints.append(self.parse_camera_tag(element_tags))

        all_body_elements: List[mujoco.MjsElement] = [self.spec.worldbody]
        visited_body_elements = set()

        # Track hierarchical full names for each body to support nested groups.
        body_to_fullname: Dict[mujoco.MjsElement, Optional[str]] = {
            self.spec.worldbody: None
        }
        free_joint_body_elements: List[mujoco.MjsElement] = []

        for body_element in all_body_elements:
            if body_element in visited_body_elements:
                logger.info("element already visited: %s", body_element.name)
                continue
            else:
                visited_body_elements.add(body_element)
                all_body_elements.extend(body_element.bodies)

            pose = mjcf_utils.pose_from_element(body_element)
            if mjcf_utils.has_freejoint(body_element):  # we have a top level element
                is_static = False
                free_joint_body_elements.append(body_element)
                if len(joint_positions) >= FREE_JOINT_LEN:
                    # keyframe_pose = pose_from_home_key(joint_positions[:FREE_JOINT_LEN])
                    # we need to include the joint position pose into the current pose as the blueprint does not cover this concept
                    # pose = Pose(pose.matrix.dot(keyframe_pose.matrix))
                    # joint_positions = joint_positions[FREE_JOINT_LEN:]
                    pass
            else:
                # we need to do it here.
                top_level_parent = self.get_top_level_parent(body_element)
                is_static = top_level_parent not in free_joint_body_elements

            if bp := self.extract_robot_bp(body_element, joint_positions):
                for b in body_element.find_all("body"):
                    visited_body_elements.add(b)
                num_joints = len(bp.default_joint_positions)
                joint_positions = joint_positions[num_joints:]
                gripper_info = self.extract_gripper_bp(body_element, joint_positions)

                if gripper_info is not None:
                    attachment, gripper_bp = gripper_info
                    # Ensure gripper has a normalized full name at root for now
                    gripper_path = _join_full_name(bp.path, gripper_bp.path)
                    gripper_path = _normalize_full_name(gripper_path)
                    num_joints = len(gripper_bp.default_joint_positions)
                    joint_positions = joint_positions[num_joints:]
                    gripper_bp = replace(gripper_bp, path=gripper_path)
                    blueprints.append(gripper_bp)
                    attachment = replace(attachment, gripper_path=gripper_path)
                    bp = replace(bp, attachment=attachment)

                bp = replace(bp, path=_normalize_full_name(bp.path))
                blueprints.append(bp)
            else:
                if body_element != self.spec.worldbody:
                    name = getattr(
                        body_element, "name", f"body_{len(visited_body_elements)}"
                    )
                    parent_fullname = body_to_fullname.get(body_element.parent, None)
                    group_name = _join_full_name(parent_fullname, name)
                    body_to_fullname[body_element] = group_name
                    blueprints.append(BlueprintGroup(group_name, pose=pose))

                for geom in body_element.geoms:
                    logger.debug(
                        "Parsing geom tag %s of %s", geom.name, body_element.name
                    )
                    if geom_bp := self.parse_geom_tag(geom, is_static):
                        parent_name = body_to_fullname.get(body_element, None)
                        full_name = _join_full_name(parent_name, geom_bp.path)
                        geom_bp = replace(geom_bp, path=full_name)
                        blueprints.append(geom_bp)

        return blueprints

    def parse_camera_tag(self, element: mujoco.MjsElement) -> CameraBlueprint:
        name = getattr(element, "name", None)

        if name is None:
            logger.warning("Camera element does not have a name.")
            name = "unknown_camera"

        name = _normalize_full_name(name)

        logger.warning(
            "Not fully supported yet. Intrinsics and other camera parameters are not specified."
        )

        width, height = 640, 480

        if fovy := getattr(element, "fovy", None):
            intrinsics = camera_intrinsics.intrinsics_from_fovy(fovy, width, height)
        else:
            intrinsics = np.array(
                [[385.0, 0.0, 320.0], [0.0, 385.0, 240.0], [0.0, 0.0, 1.0]]
            )
        pose = mjcf_utils.pose_from_element(element)
        return CameraBlueprint(name, width, height, intrinsics, pose)

    def _mujoco_geom_type_to_str(self, geom_type: int) -> str:
        geom_type_to_str = {
            mujoco.mjtGeom.mjGEOM_BOX: "box",
            mujoco.mjtGeom.mjGEOM_ELLIPSOID: "ellipsoid",
            mujoco.mjtGeom.mjGEOM_MESH: "mesh",
            mujoco.mjtGeom.mjGEOM_CAPSULE: "capsule",
            mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
            mujoco.mjtGeom.mjGEOM_PLANE: "plane",
            mujoco.mjtGeom.mjGEOM_SPHERE: "sphere",
        }
        return geom_type_to_str[geom_type]

    def parse_geom_tag(
        self,
        geom: mujoco.MjsElement,
        is_static: bool,
    ) -> Optional[Union[GeomBlueprint, MeshBlueprint]]:

        if not geom.name:
            name = f"geom_{geom.type}_{self.seq}"
            self.seq += 1
        else:
            name = f"{geom.name}"

        spec = self.spec
        asset_dir = self.asset_dir

        if not asset_dir.is_dir():
            logger.warning("Asset dir is not a folder or does not exist.")

        geom_type = self._mujoco_geom_type_to_str(geom.type)
        pose = mjcf_utils.pose_from_element(geom)
        mass = geom.mass

        if geom_type == "mesh":
            mesh = spec.mesh(geom.meshname)

            if mesh.scale[0] != mesh.scale[1] or mesh.scale[0] != mesh.scale[2]:
                logger.warning("Different scale is not supported")

            scale = mesh.scale[0]

            mesh_path = str(asset_dir / mesh.file)

            logger.debug("Mesh path: %s, mesh: %s", mesh_path, geom.meshname)
            logger.warning("Not fully implemented yet.")
            if not Path(mesh_path).is_file():
                logger.warning("Mesh path does not exist.")

            return MeshBlueprint(name, mesh_path, pose, is_static, scale)

        elif geom_type in (
            "plane",
            "box",
            "sphere",
            "cylinder",
            "capsule",
            "ellipsoid",
        ):
            logger.debug("Geom: %s", geom)

            size = geom.size.tolist()

            if not np.isnan(geom.fromto[0]):
                from_point, to_point = geom.fromto[:3], geom.fromto[3:]
                mid_vec = (to_point - from_point) / 2.0
                mid_point = from_point + mid_vec
                m = np.identity(4)
                m[:3, :3] = rotation_from_z_to_vector(mid_vec)
                m[:3, 3] = mid_point
                pose = Pose(pose.matrix.dot(m))
                extend = np.linalg.norm(mid_vec)
                size[1] = float(extend)

            rgba = geom.rgba.tolist()

            logger.debug("Size: %s, RGBA: %s", size, rgba)
            return GeomBlueprint(name, geom_type, pose, size, rgba, is_static, mass)
        else:
            logger.error("Unable to parse %s", geom)
            return None

    def extract_gripper_bp(
        self,
        element: mujoco.MjsElement,
        joint_positions: List[float],
    ) -> Optional[Tuple[Attachment, GripperBlueprint]]:
        logger.warning("Gripper parsing is not fully implemented yet.")
        for heuristic in self.gripper_heuristic_classes:
            if (h := heuristic(element, joint_positions)) and h.search():
                return h.to_blueprint()

    def extract_robot_bp(
        self, element: mujoco.MjsElement, joint_positions: List[float]
    ) -> Optional[RobotBlueprint]:
        if not element.name:
            return None
        for heuristic in self.robot_heuristic_classes:
            if (h := heuristic(element, joint_positions)) and h.search():
                return h.to_blueprint()


def pose_from_home_key(joint_pose) -> Pose:
    if len(joint_pose) != FREE_JOINT_LEN:
        logger.warning(
            "Free joint found, but remaining keyframe has insufficient length"
        )
        return Pose()
    position = joint_pose[:3]
    wxyz = joint_pose[3:FREE_JOINT_LEN]
    return Pose().with_position(position).with_quat_wxyz(wxyz)


def rotation_from_z_to_vector(v: np.ndarray, eps=1e-12):
    normalized_v = v / np.linalg.norm(v)
    z = np.array([0.0, 0.0, 1.0])

    c = np.dot(z, normalized_v)
    if c > 1.0 - eps:
        return np.identity(3)
    elif c < -1.0 + eps:
        return R.from_rotvec(np.pi * np.array([1.0, 0.0, 0.0])).as_matrix()
    axis = np.cross(z, normalized_v)
    normalized_axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.clip(c, -1.0, 1.0))
    return R.from_rotvec(normalized_axis * angle).as_matrix()
