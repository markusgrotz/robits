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

from dm_control import mjcf

from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import MeshBlueprint
from robits.sim.blueprints import Pose
from robits.sim.blueprints import Attachment
from robits.sim.blueprints import RobotDescriptionModel
from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import BlueprintGroup

from robits.sim import mjcf_utils
from robits.utils import vision_utils

from robits.sim.converters.robot_heuristics import get_all_heuristics_classes
from robits.sim.converters.gripper_heuristics import get_all_gripper_heuristics_classes


logger = logging.getLogger(__name__)

DEFAULT_RGBA = (1, 0.5, 1, 1.0)
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


def load_mjcf_as_blueprints(xml_path: Union[str, Path]) -> List[Blueprint]:
    return MujocoXMLImporter(xml_path).parse()


class MujocoXMLImporter:
    def __init__(self, xml_path: Union[str, Path]) -> None:
        xml_path = Path(xml_path).expanduser().resolve()

        self.asset_dir = xml_path.parent
        self.model = mjcf.from_path(str(xml_path), escape_separators=True)

        logger.info("Using heuristic to detect a robot. This is not fully implemented.")
        self.robot_heuristic_classes = get_all_heuristics_classes()
        self.gripper_heuristic_classes = get_all_gripper_heuristics_classes()

        self.seq = 0

    def get_top_level_parent(self, element: mjcf.Element) -> Optional[mjcf.Element]:
        current = element
        while current.parent:
            if current.parent is self.model.worldbody:
                return current
            current = current.parent
        return None

    def parse(self) -> List[Blueprint]:
        model = self.model

        joint_positions = mjcf_utils.get_home_key_qpos(model)

        blueprints = []
        for element_tags in model.find_all("camera"):
            blueprints.append(self.parse_camera_tag(element_tags))

        all_body_elements: List[mjcf.Element] = [model.worldbody]
        all_body_elements.extend(model.find_all("body", immediate_children_only=False))

        visited_body_elements = set()

        # Track hierarchical full names for each body to support nested groups.
        body_to_fullname: Dict[mjcf.Element, Optional[str]] = {model.worldbody: None}
        free_joint_body_elements: List[mjcf.Element] = []

        for body_element in all_body_elements:
            if body_element in visited_body_elements:
                logger.info("element already visited: %s", body_element)
                continue
            else:
                visited_body_elements.add(body_element)

            pose = mjcf_utils.pose_from_element(body_element)
            if mjcf_utils.has_freejoint(body_element):  # we have a top level element
                is_static = False
                free_joint_body_elements.append(body_element)
                if len(joint_positions) >= FREE_JOINT_LEN:
                    keyframe_pose = pose_from_home_key(joint_positions[:FREE_JOINT_LEN])
                    # we need to include the joint position pose into the current pose as the blueprint does not cover this concept
                    # pose = Pose(pose.matrix.dot(keyframe_pose.matrix))
                    joint_positions = joint_positions[FREE_JOINT_LEN:]
            else:
                # we need to do it here.
                top_level_parent = self.get_top_level_parent(body_element)
                is_static = top_level_parent not in free_joint_body_elements

            if body_element != model.worldbody:
                name = getattr(
                    body_element, "name", f"body_{len(visited_body_elements)}"
                )
                parent_fullname = body_to_fullname.get(body_element.parent, None)
                group_name = _join_full_name(parent_fullname, name)
                body_to_fullname[body_element] = group_name
                blueprints.append(BlueprintGroup(group_name, pose=pose))

            logger.info("Parsing geom/mesh tags")

            for geom in body_element.find_all("geom", immediate_children_only=True):
                logger.info("Parsing %s", geom)
                geom_bp = self.parse_geom_tag(geom, is_static, self.asset_dir)
                parent_name = body_to_fullname.get(body_element, None)
                full_name = _join_full_name(parent_name, geom_bp.path)
                geom_bp = replace(geom_bp, name=full_name)
                blueprints.append(geom_bp)

            if bp := self.extract_robot_bp(body_element, joint_positions):
                for b in body_element.find_all("body"):
                    visited_body_elements.add(b)
                num_joints = len(bp.default_joint_positions)
                joint_positions = joint_positions[num_joints:]
                gripper_info = self.extract_gripper_bp(body_element)

                if gripper_info is not None:
                    attachment, gripper_bp = gripper_info
                    # Ensure gripper has a normalized full name at root for now
                    gripper_path = _join_full_name(bp.path, gripper_bp.path)
                    gripper_path = _normalize_full_name(gripper_path)
                    gripper_bp = replace(gripper_bp, name=gripper_path)
                    blueprints.append(gripper_bp)
                    attachment = replace(attachment, gripper_path=gripper_path)
                    bp = replace(bp, attachment=attachment)

                bp = replace(bp, name=_normalize_full_name(bp.path))
                blueprints.append(bp)

        return blueprints

    def parse_camera_tag(self, element: mjcf.Element) -> CameraBlueprint:
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
            intrinsics = vision_utils.intrinsics_from_fovy(fovy, width, height)
        else:
            intrinsics = np.array(
                [[385.0, 0.0, 320.0], [0.0, 385.0, 240.0], [0.0, 0.0, 1.0]]
            )
        pose = mjcf_utils.pose_from_element(element)
        return CameraBlueprint(name, width, height, intrinsics, pose)

    def parse_geom_tag(
        self, geom: mjcf.Element, is_static: bool, asset_dir: Path
    ) -> Union[GeomBlueprint, MeshBlueprint]:
        if geom.name is None:
            name = f"geom_{geom.type}_{self.seq}"
            self.seq += 1
        else:
            name = f"{geom.name}"
        geom_type = geom.type
        pose = mjcf_utils.pose_from_element(geom)

        if geom_type == "mesh":
            scale = getattr(geom, "scale", 1.0)
            mesh_path = geom.mesh.file.get_vfs_filename(False)
            if asset_dir.is_dir():
                mesh_path = asset_dir / mesh_path
            else:
                logger.warning("Asset dir is not a folder or does not exist.")

            logger.debug("Mesh path: %s, mesh: %s", mesh_path, geom.mesh)
            logger.warning("Not fully implemented yet.")
            if not Path(mesh_path).is_file():
                logger.warning("Mesh path does not exist.")

            return MeshBlueprint(name, mesh_path, pose, is_static, scale)

        else:
            logger.debug("Geom: %s", geom)

            if geom.size is not None:
                size = geom.size.tolist() * 3
                size = size[:3]
            else:
                logger.warning("No size information.")
                size = [0, 0, 0]

            if geom.rgba is not None:
                rgba = geom.rgba.tolist()
            else:
                rgba = DEFAULT_RGBA

            if hasattr(geom, "fromto") and geom.fromto is not None:
                from_point, to_point = geom.fromto[:3], geom.fromto[3:]
                mid_vec = (to_point - from_point) / 2.0
                mid_point = from_point + mid_vec
                m = np.identity(4)
                m[:3, :3] = rotation_from_z_to_vector(mid_vec)
                m[:3, 3] = mid_point
                pose = Pose(pose.matrix.dot(m))
                extend = np.linalg.norm(mid_vec)
                size[1] = extend

            logger.debug("Size: %s, RGBA: %s", size, rgba)
            return GeomBlueprint(name, geom_type, pose, size, rgba, is_static)

    def extract_gripper_bp(
        self,
        element: mjcf.Element,
    ) -> Optional[Tuple[Attachment, GripperBlueprint]]:
        logger.warning("Gripper parsing is not fully implemented yet.")

        joint_positions = [0]
        for heuristic in self.gripper_heuristic_classes:
            if (h := heuristic(element, joint_positions)) and h.search():
                return h.to_blueprint()

    def extract_robot_bp(
        self, element: mjcf.Element, joint_positions: List[float]
    ) -> Optional[RobotBlueprint]:
        name = getattr(element, "name", None)
        if not name:
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
