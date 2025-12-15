from typing import Optional
from typing import Sequence

from functools import singledispatchmethod

import logging

import numpy as np

import mujoco
from dm_control import mjcf

from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import ObjectBlueprint
from robits.sim.blueprints import MeshBlueprint
from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import Blueprint

from robits.sim import utils

logger = logging.getLogger(__name__)

DEFAULT_FREE_JOINT_QPOS = np.array([0., 0., 0., 1., 0., 0., 0.])

class SceneBuilder:

    def __init__(self, add_floor: bool=True):
        self.scene = mjcf.RootElement()
        #self.scene.worldbody.add("body", name="box_body", pos="0 0 0.5")
        self.scene.worldbody.add("light", pos="0 0 5")
        self.key = self.scene.keyframe.add("key", name="home", qpos="", ctrl="")
        if add_floor:
            self.add_default_assets()

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
        Creates a mujoco model from the blueprints
        """
        logger.info("Building environment model: %s", blueprints)
        bp_id_to_bp = {bp.id: bp for bp in blueprints}

        #  this ensures that the free joints are first
        for b in blueprints:
            if isinstance(
                b, (MeshBlueprint, CameraBlueprint, ObjectBlueprint, GeomBlueprint)
            ):
                self.add(b)

        for b in blueprints:
            if isinstance(b, RobotBlueprint):
                gripper_blueprint: Optional[GripperBlueprint] = None
                if b.attachment:
                    gripper_blueprint = bp_id_to_bp.get(b.attachment.blueprint_id, None)
                    if gripper_blueprint is None:
                        logger.warning("Unable to find gripper blueprint with id %s.", b.attachment.blueprint_id)
                self.add_robot(b, gripper_blueprint)

        self.merge_all_keyframes_into_home()
        logger.debug("Model is %s", self.scene.to_xml_string())
        return utils.reload_model_with_assets(self.scene)

    @singledispatchmethod
    def add(self, blueprint):
        raise NotImplementedError(f"Unsupported blueprint type: {type(blueprint)}")

    @add.register
    def add_mesh(self, blueprint: MeshBlueprint):
        mujoco_scale = f"{blueprint.scale} {blueprint.scale} {blueprint.scale}"
        self.scene.asset.add(
            "mesh", name=f"{blueprint.name}_mesh", file=blueprint.mesh_path, scale=mujoco_scale
        )
        if  blueprint.is_static:
            body = self.scene.worldbody
        else:
            body = self.scene.worldbody.add("body", name=f"{blueprint.name}_body")
            joint = body.add("freejoint")
            joint.name = f"{blueprint.name}_joint"
            for k in self.scene.find_all("key"):
                k.qpos = np.concatenate([k.qpos, DEFAULT_FREE_JOINT_QPOS], axis=None)
    
        geom = body.add("geom", type="mesh", mass=1.0, mesh=f"{blueprint.name}_mesh")
        utils.set_pose(geom, blueprint.pose)
        return self

    @add.register
    def add_object(self, blueprint: ObjectBlueprint):
        object = utils.load_model_from_path(
            blueprint.model_path, escape_separators=True
        )
        object = self.scene.attach(object)
        utils.set_pose(object, blueprint.pose)
        return self

    @add.register
    def add_camera(self, blueprint: CameraBlueprint):
        camera_name = blueprint.name
        if self.scene.find("camera", camera_name):
            logger.warning("Camera %s already added", camera_name)
            return self
        camera = self.scene.worldbody.add(
            "camera", name=camera_name, mode="trackcom"# , target="box_body"
        )
        utils.set_pose(camera, blueprint.pose)
        return self

    @add.register
    def add_geom(self, blueprint: GeomBlueprint):
        if not blueprint.is_static:
            body = self.scene.worldbody.add("body", name=f"{blueprint.name}_body")
            joint = body.add("freejoint")
            joint.name = f"{blueprint.name}_joint"
            for k in self.scene.find_all("key"):
                k.qpos = np.concatenate([k.qpos, DEFAULT_FREE_JOINT_QPOS], axis=None)
        else:
            body = self.scene.worldbody

        geom = body.add(
            "geom",
            type=blueprint.geom_type,
            name=blueprint.name,
            #mass=10.02,
            mass=0.02,
            size=blueprint.size,
            rgba=blueprint.rgba,
        )

        utils.set_pose(geom, blueprint.pose)
        return self    

    def add_mocap(
            self
    ):
        _mocap_body = self.scene.worldbody.add("body", name="target", mocap="true")
        return self

    def add_robot(
        self,
        blueprint: RobotBlueprint,
        gripper_blueprint: Optional[GripperBlueprint] = None,
    ):
        logger.info("Building robot model for %s", blueprint.name)
        robot = utils.load_and_clean_model(blueprint.model)
        
        if blueprint.default_joint_positions:
            utils.update_joint_position(robot, blueprint.default_joint_positions)

        for s in robot.find_all("site"):
            logger.info("Found site %s", s.name)

        if gripper_blueprint:
            robot = self.attach_gripper(robot, blueprint.attachment, gripper_blueprint)

        robot = self.scene.attach(robot)
        utils.set_pose(robot, blueprint.pose)
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
        self, arm_model, attachment_blueprint, gripper_blueprint: GripperBlueprint
    ) -> mujoco.MjModel:
        logger.info("Attaching gripper to robot model.")

        wrist = arm_model.find("body", attachment_blueprint.wrist_name)
        if not wrist:
            logger.error("I was looking for %s", attachment_blueprint.wrist_name)
            for b in arm_model.find_all("body"):
                print(b.name)
            raise ValueError("Unable to find wrist please update the config")

        # remove unncessary information
        # for e in ["geom", "joint", "body", "camera", "site"]:
        for k in wrist.find_all("geom"):
            logger.debug("Removing %s", k.name)
            k.remove()

        gripper_model = utils.load_model_from_blueprint(gripper_blueprint.model)
        logger.debug(
            "Changing the namescope of the gripper to gripper. Previously: %s",
            gripper_model.namescope.name,
        )
        gripper_model.namescope.name = "gripper"

        #utils.merge_home_keys(arm_model, gripper_model)

        attachment_site = arm_model.worldbody.find(
            "site", attachment_blueprint.attachment_site
        )
        if attachment_site is None:
            logger.info(
                "Unable to find an attachment site. Adding attachment site manually."
            )
            attachment_site = wrist.add("site")

        if attachment_site.name != "attachment_site":
            logger.warning(
                "Renaming attachment site for consistency. Previously: %s, now called attachment_site",
                attachment_site.name,
            )
            attachment_site.name = "attachment_site"

        frame = attachment_site.attach(gripper_model)
        utils.set_pose(frame, attachment_blueprint.wrist_pose)

        return arm_model
