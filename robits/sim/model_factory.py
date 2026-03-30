from typing import Optional
from typing import Sequence
from typing import Dict
from typing import List
# from typing import cast

import logging
from pathlib import Path
from functools import singledispatchmethod
import zipfile
import tempfile

import numpy as np

import mujoco

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

from robits.sim import mjcf_utils as mjcf_utils

logger = logging.getLogger(__name__)


class SceneBuilder:
    def __init__(self, use_default_scene: bool = True):

        if use_default_scene:
            self.spec = mujoco.MjSpec.from_string(mjcf_utils.DEFAULT_WORLD_STR)
        else:
            self.spec = mujoco.MjSpec()
            self.spec.worldbody.add_light(pos=[0, 0, 5.0])

        # Map full blueprint path -> created MJCF element
        self.mapping: Dict[str, mujoco.MjsElement] = {}

    def export_with_assets(self, out_path: Path) -> None:

        self.spec.to_file(str(out_path))

        with tempfile.TemporaryDirectory() as tmpdir:
            zip_path = Path(tmpdir) / "model.zip"
            self.spec.to_zip(str(zip_path))
            with zipfile.ZipFile(zip_path, "r") as z:
                z.extractall(out_path.parent)

    def get_parent(self, blueprint: Blueprint) -> mujoco.MjsBody:
        """
        Searches for the parent body given a blueprint
        """
        if blueprint.parent_path == "/":
            return self.spec.worldbody
        if parent := self.mapping.get(blueprint.parent_path, None):
            return parent
        logger.warning(
            "Parent not found for blueprint %s. Assuming worldbody", blueprint
        )
        return self.spec.worldbody

    def get_top_level_parent(self, blueprint: Blueprint) -> mujoco.MjsBody:
        """
        Returns the top parent element below the worldbody
        """
        parts = blueprint.path.strip("/").split("/")
        if len(parts) <= 1:
            return self.spec.worldbody
        return self.mapping.get("/" + parts[0])

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

        logger.debug("Current spec is %s", self.spec.to_xml())

        return self.build()
        # return self.spec.compile()

    def merge_into_single_home_key(self):
        model = self.spec.compile()

        qpos = np.zeros(model.nq, dtype=float)
        ctrl = np.zeros(model.nu, dtype=float)

        for k in self.spec.keys:
            k_qpos = np.asarray(k.qpos)
            mask = k_qpos != 0
            qpos[mask] = k_qpos[mask]

            k_ctrl = np.asarray(k.ctrl)
            mask = k_ctrl != 0
            ctrl[mask] = k_ctrl[mask]

        self.spec.add_key(name="home", qpos=qpos, ctrl=ctrl)

    def build(self) -> mujoco.MjModel:

        self.merge_into_single_home_key()

        return self.spec.compile()

    @singledispatchmethod
    def add(self, blueprint):
        raise NotImplementedError(f"Unsupported blueprint type: {type(blueprint)}")

    @add.register
    def add_mesh(self, blueprint: MeshBlueprint):

        mesh = self.spec.add_mesh(
            name=f"{blueprint.basename}_mesh",
            file=blueprint.mesh_path,
            scale=[blueprint.scale] * 3,
        )

        texture_path = Path(blueprint.mesh_path).with_name("textured.png")

        if texture_path.is_file():
            tex = self.spec.add_texture(
                name=f"{blueprint.basename}_tex",
                file=str(texture_path),
                type=mujoco.mjtTexture.mjTEXTURE_2D,
            )
            mat = self.spec.add_material(
                name=f"{blueprint.basename}_mat", textures=[tex.name]
            )
            mat_name = mat.name
        else:
            mat_name = None

        parent_body = self.get_parent(blueprint)
        geom = parent_body.add_geom(
            name=f"{blueprint.basename}",
            type=mujoco.mjtGeom.mjGEOM_MESH,
            mass=1.0,
            meshname=mesh.name,
            material=mat_name,
        )
        mjcf_utils.set_pose(geom, blueprint.pose)
        return self

    @add.register
    def add_object(self, blueprint: ObjectBlueprint):
        obj_spec = mujoco.MjSpec.from_file(blueprint.model_path)

        obj_mount = self.spec.worldbody.add_frame()
        frame = self.spec.attach(
            obj_spec, frame=obj_mount, prefix=f"{blueprint.basename}/"
        )

        mjcf_utils.add_offset_pose(frame, blueprint.pose)
        return self

    @add.register
    def add_camera(self, blueprint: CameraBlueprint):
        camera_name = blueprint.basename
        parent = self.get_parent(blueprint)
        camera = parent.add_camera(name=camera_name)
        mjcf_utils.set_pose(camera, blueprint.pose)
        return self

    @add.register
    def add_group(self, blueprint: BlueprintGroup):
        parent = self.get_parent(blueprint)
        body = parent.add_body(name=blueprint.basename)
        self.mapping[blueprint.path] = body
        mjcf_utils.set_pose(body, blueprint.pose)
        return self

    @add.register
    def add_geom(self, blueprint: GeomBlueprint):
        parent = self.get_parent(blueprint)
        if not blueprint.is_static:
            top_level_parent = self.get_top_level_parent(blueprint)
            if top_level_parent == self.spec.worldbody:
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
                top_level_parent.add_freejoint()

        geom = parent.add_geom(
            type=mjcf_utils.str_to_mujoco_geom_type(blueprint.geom_type),
            name=blueprint.basename,
            mass=blueprint.mass if blueprint.mass else None,
            size=blueprint.size,
            rgba=blueprint.rgba,
        )

        mjcf_utils.set_pose(geom, blueprint.pose)
        return self

    def add_mocap(self):
        _mocap_body = self.spec.worldbody.add_body(name="target", mocap=1)
        return self

    @add.register
    def add_gripper(self, blueprint: GripperBlueprint):

        gripper_spec = mjcf_utils.load_and_clean_spec(blueprint.model)

        gripper_mount = self.spec.worldbody.add_frame()
        _frame = self.spec.attach(
            gripper_spec, frame=gripper_mount, prefix=f"{blueprint.basename}/"
        )

        # mjcf_utils.add_offset_pose(frame, blueprint.pose)

        self.spec.compile()

        return self

    def add_robot(
        self,
        blueprint: RobotBlueprint,
        gripper_blueprint: Optional[GripperBlueprint] = None,
    ):
        logger.info("Building robot model for %s", blueprint.path)
        robot_spec = mjcf_utils.load_and_clean_spec(blueprint.model)
        robot_spec.modelname = blueprint.basename

        if blueprint.default_joint_positions:
            mjcf_utils.update_home_joint_position(
                robot_spec, blueprint.default_joint_positions
            )

        for s in robot_spec.sites:
            logger.info("Found site %s", s.name)

        if blueprint.attachment and gripper_blueprint:
            robot_spec = self.attach_gripper(
                robot_spec, blueprint.attachment, gripper_blueprint
            )

        self.spec.compile()

        robot_mount = self.spec.worldbody.add_frame()
        robot_frame = self.spec.attach(
            robot_spec, frame=robot_mount, prefix=f"{blueprint.basename}/"
        )


        mjcf_utils.add_offset_pose(robot_frame, blueprint.pose)

        self.spec.compile()

        return self

    def attach_gripper(
        self,
        arm_model: mujoco.MjSpec,
        attachment_blueprint: Attachment,
        gripper_blueprint: GripperBlueprint,
    ) -> mujoco.MjSpec:

        logger.info("Attaching gripper %s to robot model", gripper_blueprint.path)

        gripper_spec = mjcf_utils.load_and_clean_spec(gripper_blueprint.model)
        gripper_spec.modelname = gripper_blueprint.basename

        if gripper_blueprint.default_joint_positions:
            mjcf_utils.update_home_joint_position(
                gripper_spec, gripper_blueprint.default_joint_positions
            )

        attachment_site = arm_model.site(attachment_blueprint.attachment_site)
        if attachment_site is None:
            logger.error("Unable to find an attachment site. Unable to attach gripper.")
            return arm_model

        frame = arm_model.attach(
            gripper_spec, site=attachment_site, prefix=f"{gripper_blueprint.basename}/"
        )

        mjcf_utils.add_offset_pose(frame, attachment_blueprint.attachment_offset)

        arm_model.compile()

        return arm_model
