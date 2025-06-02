from typing import Dict
from typing import Any
from typing import Optional
from typing import List

import time
import logging

from robits.core.abc.gripper import GripperBase
from robits.sim.env_client import MujocoJointControlClient
from robits.sim.env_design import env_designer

from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import RobotDescriptionModel

logger = logging.getLogger(__name__)


class MujocoGripper(MujocoJointControlClient, GripperBase):
    """
    Implements a gripper in Mujoco
    """

    def __init__(
        self,
        gripper_name: str,
        joint_names: List[str],
        actuator_names: Optional[List[str]] = None,
        invert: bool = False,
        description_name: str = "",
        variant_name: Optional[str] = None,
        **kwargs
    ):
        """
        :param gripper_name: name of the gripper in the model
        :param joint_names: name of the joints
        :param actuator_names: name of the actuators
        :param invert: invert the joint positions
        """
        super().__init__(joint_names, actuator_names)

        self.invert = invert
        self._gripper_name = gripper_name

        description = RobotDescriptionModel(description_name, variant_name)
        env_designer.add(GripperBlueprint(gripper_name, model=description))

    @property
    def gripper_name(self):
        return self._gripper_name

    def open(self):
        if self.invert:
            self.data.ctrl[self.actuator_ids] = self.ctrl_min
        else:
            self.data.ctrl[self.actuator_ids] = self.ctrl_max

    def close(self):
        if self.invert:
            self.data.ctrl[self.actuator_ids] = self.ctrl_max
        else:
            self.data.ctrl[self.actuator_ids] = self.ctrl_min

    def get_obs(self) -> Dict[str, Any]:
        qpos = self.data.qpos[self.joint_ids].copy()
        qpos = self._normalize(qpos)
        return {"finger_positions": qpos, "timestamp": time.time()}

    def is_open(self) -> bool:
        return bool(self.get_obs()["finger_positions"][0] > 0.5)

    def _normalize(self, qpos):
        return (qpos - self.ctrl_min) / (self.ctrl_max - self.ctrl_min)

    def _unnormalize(self, qpos):
        return qpos * (self.ctrl_max - self.ctrl_min) + self.ctrl_min

    # def set_pos(self, pos):
    #    self.data.ctrl[self.actuator_ids] = self._unnormalize(pos)

    def get_info(self):
        return {
            "gripper_name": self.gripper_name,
            "joint_names": self.joint_names,
            "actuator_names": self.actuator_names,
            "invert_joint_positions": self.invert,
        }
