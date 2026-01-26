from typing import Tuple
from typing import List
from abc import ABC
from abc import abstractmethod

import logging

from dm_control import mjcf

from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import Attachment
from robits.sim.blueprints import RobotDescriptionModel

logger = logging.getLogger(__name__)


class GripperHeuristic(ABC):
    def __init__(self, element: mjcf.Element, joint_positions):
        self.element = element
        self.joint_positions = joint_positions

    def to_blueprint(self) -> Tuple[Attachment, GripperBlueprint]:
        default_joint_positions = self.joint_positions[: self.num_joints]
        gripper_bp = GripperBlueprint(
            f"/{self.prefix}",
            self.model(),
            default_joint_positions=default_joint_positions,
        )
        attachment = Attachment(f"/{self.prefix}", "")
        return (attachment, gripper_bp)

    @abstractmethod
    def model(self) -> RobotDescriptionModel:
        pass

    @abstractmethod
    def search(self) -> bool:
        pass

    @property
    @abstractmethod
    def num_joints(self) -> int:
        pass

    @property
    @abstractmethod
    def prefix(self) -> str:
        pass


class PandaHeuristic(GripperHeuristic):
    @property
    def num_joints(self) -> int:
        return 1

    @property
    def prefix(self) -> str:
        return "panda hand"

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("panda_mj_description", "hand.xml")

    def search(self) -> bool:
        for b in self.element.find_all("joint"):
            if hasattr(b, "name") and "panda hand\\finger_joint1" in b.name:
                return True
        return False


class RobotiqHeuristic(GripperHeuristic):
    @property
    def num_joints(self) -> int:
        return 1

    @property
    def prefix(self) -> str:
        return "robotiq_2f85"

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("robotiq_2f85_mj_description")

    def search(self) -> bool:
        for b in self.element.find_all("joint"):
            if hasattr(b, "name") and "robotiq_2f85\\right_coupler_joint" in b.name:
                return True
        return False


def get_all_gripper_heuristics_classes() -> List[type[GripperHeuristic]]:
    return [
        RobotiqHeuristic,
        PandaHeuristic,
    ]
