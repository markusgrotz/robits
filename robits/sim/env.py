from typing import Dict
from typing import Any
from typing import List
from typing import Set

import logging

import time
import threading
from functools import lru_cache

# import numpy as np
from scipy.spatial.transform import Rotation as R


import mujoco
import mujoco.viewer

from robits.sim.env_design import env_designer
from robits.sim.model_factory import SceneBuilder

logger = logging.getLogger(__name__)


class MujocoEnv:
    """
    Mujoco environment
    """

    def __init__(self) -> None:
        """
        Initializes the mujoco environment and launches the simulation control loop
        """

        self.camera_names = env_designer.get_camera_names()

        blueprints = env_designer.finalize()
        self.model = SceneBuilder().build_from_blueprints(blueprints.values())

        self.data = mujoco.MjData(self.model)

        logger.info("Model timestep is %s", self.model.opt.timestep)

        logger.info("resetting to first keyframe")
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

        self.last_step = 0.0
        self.seq = 0

        self.image_lock = threading.Lock()
        self.camera_data: Dict[str, Any] = {}

        with mujoco.Renderer(self.model, height=480, width=640) as renderer:
            self.render_cameras(renderer)

        self.joint_id_to_actuator_id = self.get_joint_to_actuator_mapping(self.model)

        threading.Thread(target=self.sim_control_loop, daemon=True).start()

    def get_joint_to_actuator_mapping(self, model) -> Dict[int, int]:
        """
        Generates a mapping from joint id to actuator id

        :param model: the mujoco model to build the mapping from
        """
        mapping: Dict[int, int] = {}
        for i in range(model.nu):
            actuator = model.actuator(i)
            joint = model.joint(actuator.trnid[0])
            logger.debug(
                f"Actuator {model.actuator(i).name}: Connected to Joint {joint.name}"
            )
            if joint.id in mapping:
                logger.warning(
                    "Inconsistent model detected while generating joint / actuator mapping"
                )
                logger.warning(
                    "Joint %s maps already to Actuator %s, but Actuator %s does as well",
                    joint.name,
                    model.actuator(mapping[joint.id]).name,
                    actuator.name,
                )
            else:
                mapping[joint.id] = i
        return mapping

    def sim_control_loop(self) -> None:
        """
        The actual control loop over the simulation
        """
        model = self.model
        data = self.data

        logger.info("Starting sim control loop")

        with mujoco.viewer.launch_passive(model, data) as viewer, mujoco.Renderer(
            model, height=480, width=640
        ) as renderer:
            self.viewer = viewer
            while viewer.is_running():
                mujoco.mj_step(model, data)
                self.last_step = time.monotonic()
                viewer.sync()
                self.render_cameras(renderer)
                # time.sleep(0.002)

                # reset the cache
                self.get_collisions.cache_clear()
                self.get_scene_info.cache_clear()

                self.seq += 1

    def render_cameras(self, renderer) -> None:
        """
        Renders the camera images

        :param renderer: the renderer to use
        """
        # We already called mj_step
        # mujoco.mj_forward(self.model, self.data)

        new_camera_data = {}
        # since we control the step function in the simulation we can set it to a single timestamp.
        metadata = {"timestamp": time.time(), "seq": self.seq}
        for camera_name in self.camera_names:

            renderer.disable_depth_rendering()
            renderer.update_scene(self.data, camera=camera_name)
            rgb_image = renderer.render()

            renderer.enable_depth_rendering()
            depth_image = renderer.render()

            new_camera_data[f"{camera_name}_rgb"] = rgb_image.copy()
            new_camera_data[f"{camera_name}_depth"] = depth_image.copy()
            new_camera_data[f"{camera_name}_metadata"] = metadata.copy()

        with self.image_lock:
            self.camera_data = new_camera_data

    @property
    def num_free_joints(self) -> int:
        """
        Returns the number of free joint in the model
        """
        return sum(self.model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
    

    @lru_cache(maxsize=1)
    def get_scene_info(self) -> Dict[str, Dict[str, Any]]:
        """
        Gets the current information about objects in the scene. 
        An object is defined as geom element, whose parent bodies have a free
        joint.
        The result of this function is cached but resetted after each
        simulation step

        :returns: information, such as pose, about objects in the scene
        """
        data = {}
        for i in range(self.model.ngeom):
            geom_body_id = self.model.geom_bodyid[i]
            # Check if the body has a freejoint
            has_freejoint = False
            for j in range(self.model.njnt):
                if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE and self.model.jnt_bodyid[j] == geom_body_id:
                    has_freejoint = True
                    break

            if not has_freejoint:
                continue

            # ..todo:: extract..
            obj_model = self.model.geom(i)
            obj_data = self.data.geom(i)
            name = obj_data.name
            mat = obj_data.xmat.reshape(3, 3)
            q = R.from_matrix(mat).as_quat()

            data[name] = {
                "id": obj_data.id,
                "name": name,
                "position": obj_data.xpos,
                "quaternion": q,
                "size": obj_model.size,
                "friction": obj_model.friction,
                "rgba": obj_model.rgba,
            }
        return data


    @lru_cache(maxsize=1)
    def get_collisions(self) -> List[Set[str]]:
        """
        Object names that are in collision. Result of this function is cached,
        but the cache is resetted after each simulation step.

        :returns: The object that are in collisions
        """
        colliding_objects = []

        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1 = c.geom1
            g2 = c.geom2
            dist = c.dist
            name1 = self.model.geom(g1).name
            name2 = self.model.geom(g2).name
            if dist < 0:
                colliding_objects.append({name1, name2})

        return colliding_objects

    @classmethod
    @lru_cache(maxsize=1)
    def get(cls) -> "MujocoEnv":
        """
        Singleton instance
        """
        return MujocoEnv()
