from typing import Optional
from typing import Sequence
from typing import Dict
from typing import List
# from typing import cast

import logging
from pathlib import Path
from functools import singledispatchmethod

import numpy as np

import mujoco
from dm_control import mjcf

from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import MeshBlueprint
from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import Attachment
from robits.sim.blueprints import Pose

from robits.sim.blueprints import ObjectBlueprint
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import BlueprintGroup

from robits.sim import mjcf_utils

logger = logging.getLogger(__name__)

# xyz + wxyz
DEFAULT_FREE_JOINT_QPOS = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])


class SceneBuilder:

    def __init__(self, add_floor: bool = True):
        self.scene = mjcf.RootElement()
        #self.scene.worldbody.add("body", name="box_body", pos="0 0 0.5")
        self.scene.worldbody.add("light", pos="0 0 5")
        self.key = self.scene.keyframe.add("key", name="home", qpos="", ctrl="")
        if add_floor:
            self.add_default_assets()
        # Map full blueprint path -> created MJCF element
        self.mapping: Dict[str, mjcf.Element] = {}

    def export_with_assets(self, out_path: Path) -> None:
        mjcf.export_with_assets(self.scene, out_path.parent, out_path.name)

    def get_parent(self, blueprint: Blueprint) -> mjcf.Element:
        if blueprint.parent_path == "/":
            return self.scene.worldbody
        if parent := self.mapping.get(blueprint.parent_path, None):
            return parent
        logger.warning(
            "Parent not found for blueprint %s. Assuming worldbody", blueprint
        )
        return self.scene.worldbody

    def get_top_level_parent(self, blueprint: Blueprint) -> mjcf.Element:
        """
        Returns the top parent element below the worldbod
        """
        parts = blueprint.path.strip("/").split("/")
        if len(parts) <= 1:
            return self.scene.worldbody
        return self.mapping.get("/" + parts[0])

    def add_default_assets(self):
        self.scene.asset.add("material", name="groundplane")
        self.scene.worldbody.add(
            "geom",
            name="floor",
            size="0 0 0.05",
            type="plane",
            material="groundplane",
            rgba=[0.5, 0.5, 0.5, 1],
        )
        return self

    def build_from_blueprints(self, blueprints: Sequence[Blueprint]) -> mujoco.MjModel:
        """
        Creates a mujoco model from blueprints
        """
        logger.info("Building environment model: %s", blueprints)

        bp_name_to_bp: Dict[str, Blueprint] = {bp.path: bp for bp in blueprints}

        other_bps: List[Blueprint] = []
        robot_bps: List[RobotBlueprint] = []
        gripper_bps: List[GripperBlueprint] = []

        for b in blueprints:
            if not b.path.startswith("/"):
                logger.warning("Invalid blueprint name %s. Skipping blueprint.", b.path)
                continue
            if isinstance(b, RobotBlueprint):
                robot_bps.append(b)
            elif isinstance(b, GripperBlueprint):
                gripper_bps.append(b)
            else:
                other_bps.append(b)

        other_bps.sort(key=lambda bp: bp.path)
        for obp in other_bps:
            self.add(obp)

        for robot_bp in robot_bps:
            gripper_bp: Optional[GripperBlueprint] = None
            if robot_bp.attachment:
                if gripper_bp := bp_name_to_bp.get(
                    robot_bp.attachment.gripper_path, None
                ):
                    gripper_bps.remove(gripper_bp)
                else:
                    logger.warning(
                        "Unable to find gripper blueprint with id %s.",
                        robot_bp.attachment.gripper_path,
                    )
            self.add_robot(robot_bp, gripper_bp)

        # add remaining grippers.
        for bp in gripper_bps:
            self.add(bp)

        self.merge_all_keyframes_into_home()
        logger.debug("Model is %s", self.scene.to_xml_string())
        return mjcf_utils.reload_model_with_assets(self.scene)

    @singledispatchmethod
    def add(self, blueprint):
        raise NotImplementedError(f"Unsupported blueprint type: {type(blueprint)}")

    @add.register
    def add_mesh(self, blueprint: MeshBlueprint):
        mujoco_scale = f"{blueprint.scale} {blueprint.scale} {blueprint.scale}"
        self.scene.asset.add(
            "mesh",
            name=f"{blueprint.basename}_mesh",
            file=blueprint.mesh_path,
            scale=mujoco_scale,
        )
        parent_body = self.get_parent(blueprint)
        geom = parent_body.add(
            "geom",
            name=f"{blueprint.basename}",
            type="mesh",
            mass=1.0,
            mesh=f"{blueprint.basename}_mesh",
        )
        mjcf_utils.set_pose(geom, blueprint.pose)
        return self

    @add.register
    def add_object(self, blueprint: ObjectBlueprint):
        obj_model = mjcf_utils.load_model_from_path(
            blueprint.model_path, escape_separators=True
        )
        obj_model = self.scene.attach(obj_model)

        mjcf_utils.add_offset_pose(obj_model, blueprint.pose)
        return self

    @add.register
    def add_camera(self, blueprint: CameraBlueprint):
        camera_name = blueprint.basename
        parent = self.get_parent(blueprint)
        camera = parent.add(
            "camera",
            name=camera_name,
            mode="trackcom",  # , target="box_body"
        )
        mjcf_utils.set_pose(camera, blueprint.pose)
        return self

    @add.register
    def add_group(self, blueprint: BlueprintGroup):
        parent = self.get_parent(blueprint)
        name = blueprint.basename
        body = parent.add("body", name=name)
        self.mapping[blueprint.path] = body
        mjcf_utils.set_pose(body, blueprint.pose)
        return self

    @add.register
    def add_geom(self, blueprint: GeomBlueprint):
        parent = self.get_parent(blueprint)
        if not blueprint.is_static:
            top_level_parent = self.get_top_level_parent(blueprint)
            if top_level_parent == self.scene.worldbody:
                logger.warning(
                    "Non-static geom element without a group. To add a freejoint we need to wrap this in a group."
                )
                new_body_name = f"{blueprint.path}_body_{len(self.mapping)}"
                self.add_group(BlueprintGroup(path=new_body_name, pose=Pose()))
                top_level_parent = self.mapping[new_body_name]
                parent = top_level_parent

            if mjcf_utils.has_freejoint(top_level_parent):
                logger.debug("Free joint already exists for %s", blueprint.path)
            else:
                joint = top_level_parent.add("freejoint")
                joint.name = f"{top_level_parent.name}_joint"
                for k in self.scene.find_all("key"):
                    qpos = DEFAULT_FREE_JOINT_QPOS
                    if top_level_parent.pos is not None:
                        qpos[:3] = top_level_parent.pos
                    if top_level_parent.quat is not None:
                        qpos[-4:] = top_level_parent.quat
                    k.qpos = np.concatenate([k.qpos, qpos], axis=None)

        geom = parent.add(
            "geom",
            type=blueprint.geom_type,
            name=blueprint.basename,
            mass=blueprint.mass,
            size=blueprint.size,
            rgba=blueprint.rgba,
        )

        mjcf_utils.set_pose(geom, blueprint.pose)
        return self

    def add_mocap(self):
        _mocap_body = self.scene.worldbody.add("body", name="target", mocap="true")
        return self

    @add.register
    def add_gripper(self, blueprint: GripperBlueprint):
        gripper_model = mjcf_utils.load_model_from_blueprint(blueprint.model)

        _gripper = self.scene.attach(gripper_model)

        return self

    def add_robot(
        self,
        blueprint: RobotBlueprint,
        gripper_blueprint: Optional[GripperBlueprint] = None,
    ):
        logger.info("Building robot model for %s", blueprint.path)
        robot = mjcf_utils.load_and_clean_model(blueprint.model)

        robot.namescope.name = blueprint.path.rsplit("/", maxsplit=1)[-1]

        for s in robot.find_all("site"):
            logger.info("Found site %s", s.name)

        if gripper_blueprint:
            robot = self.attach_gripper(robot, blueprint.attachment, gripper_blueprint)

        if blueprint.default_joint_positions:
            mjcf_utils.update_joint_position(robot, blueprint.default_joint_positions)

        robot = self.scene.attach(robot)
        mjcf_utils.add_offset_pose(robot, blueprint.pose)
        return self


    def merge_all_keyframes_into_home(self):
        qpos = []
        ctrl = []
        for k in self.scene.find_all("key"):
            qpos.append(k.qpos)
            ctrl.append(k.ctrl)
            if k != self.key:
                k.remove()

        self.key.qpos = np.concatenate([*qpos], axis=None)
        self.key.ctrl = np.concatenate([*ctrl], axis=None)

    def attach_gripper(
        self,
        arm_model: mjcf.Element,
        attachment_blueprint: Attachment,
        gripper_blueprint: GripperBlueprint,
    ) -> mujoco.MjModel:
        logger.info("Attaching gripper %s to robot model", gripper_blueprint.path)

        gripper_model = mjcf_utils.load_and_clean_model(gripper_blueprint.model)
        logger.info(
            "Changing the namescope of the gripper to gripper. Previously: %s",
            gripper_model.namescope.name,
        )
        gripper_model.namescope.name = "gripper"

        attachment_site = arm_model.worldbody.find(
            "site", attachment_blueprint.attachment_site
        )
        if attachment_site is None:
            logger.error(
                "Unable to find an attachment site. Adding attachment site manually."
            )

        if attachment_site.name != "attachment_site":
            logger.warning(
                "Renaming attachment site for consistency. Previously: %s, now called attachment_site",
                attachment_site.name,
            )
            attachment_site.name = "attachment_site"

        frame = attachment_site.attach(gripper_model)
        mjcf_utils.add_offset_pose(frame, attachment_blueprint.attachment_offset)
        mjcf_utils.merge_home_keys(arm_model, gripper_model)

        return arm_model
