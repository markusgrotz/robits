from typing import List
from typing import Optional

from functools import lru_cache

import numpy as np

import mujoco

from robits.sim.env import MujocoEnv


class MujocoEnvClient:
    """
    Convenience class to access data models

    Also allows the lazy initialize the mujoco environment.
    """

    @property
    def env(self) -> "MujocoEnv":
        """
        Access the current environment, which triggers a build of the environment
        """
        return MujocoEnv.get()

    @property
    def data(self) -> mujoco.MjData:
        """
        Convenience property to access the MuJoCo simulation data.
        """
        return self.env.data

    @property
    def model(self) -> mujoco.MjModel:
        """
        Convenience property to access the MuJoCo model.
        """
        return self.env.model

    @property
    def viewer(self):
        """
        Convenience property to access the current MuJoCo viewer.
        """
        return self.env.viewer


class MujocoJointControlClient(MujocoEnvClient):
    def __init__(
        self, joint_names: List[str], actuator_names: Optional[List[str]] = None
    ):
        """
        :param joint_names: name of the joints in the model
        :param actuator names: (optional) name of the actuators in the model
        """
        self.joint_names = joint_names
        self.actuator_names = actuator_names or []

    @property
    @lru_cache(1)
    def joint_ids(self) -> np.ndarray:
        """
        :returns: the joint ids
        """
        return np.array(
            [self.model.joint(name).id for name in self.joint_names], dtype=int
        )

    @property
    @lru_cache(1)
    def actuator_ids(self) -> np.ndarray:
        """
        :returns: the actuator ids
        """
        if self.actuator_names:
            return np.array(
                [self.env.model.actuator(n).id for n in self.actuator_names]
            )
        return np.array(
            [self.env.joint_id_to_actuator_id[i] for i in self.joint_ids], dtype=int
        )

    @property
    @lru_cache(1)
    def qpos_indices(self) -> np.ndarray:
        """
        :returns: the qpos indicies for each actuated joints
        """
        return np.array(
            [self.model.jnt_qposadr[i] for i in self.joint_ids],
            dtype=int,
        )

    @property
    @lru_cache(1)
    def qvel_indices(self) -> np.ndarray:
        """
        qvel, qacc indices for each actuated joint, using `jnt_dofadr`.

        see https://github.com/google-deepmind/mujoco/issues/1564
        """
        return np.array(
            [self.model.jnt_dofadr[i] for i in self.joint_ids],
            dtype=int,
        )

    @property
    @lru_cache(1)
    def ctrl_min(self) -> np.ndarray:
        """
        Minimum control range for the actuators
        """
        return self.model.actuator_ctrlrange[self.actuator_ids][:, 0]

    @property
    @lru_cache(1)
    def ctrl_max(self) -> np.ndarray:
        """
        Maximum control range for the actuators
        """
        return self.model.actuator_ctrlrange[self.actuator_ids][:, 1]

    def get_current_joint_positions(self) -> np.ndarray:
        """
        Removes the free joints from the model.

        :returns: A numpy array of the current joint positions
        """
        return self.data.qpos[self.qpos_indices].copy()
