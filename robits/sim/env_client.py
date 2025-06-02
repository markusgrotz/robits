from typing import List
from typing import Optional

from functools import lru_cache

import numpy as np

import mujoco

from robits.sim.env import MujocoEnv

"""
@dataclass
class ControlGroup:

    joint_ids: np.ndarray

    actuator_ids: np.ndarray

    site_id: int
"""


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
        return np.array([self.model.joint(name).id for name in self.joint_names])

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
        return np.array([self.env.joint_id_to_actuator_id[i] for i in self.joint_ids])

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
        current_joint_positions = self.data.qpos.copy()
        current_joint_positions = current_joint_positions[
            self.env.num_free_joints * 6 :
        ]
        return current_joint_positions[self.joint_ids]
