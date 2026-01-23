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
            "/gripper",  # will be fixed later
            self.model(),
            default_joint_positions=default_joint_positions,
        )
        attachment = Attachment("/gripper", "")
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


class PandaHeuristic(GripperHeuristic):
    @property
    def num_joints(self) -> int:
        return 1

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("panda_mj_description", "hand.xml")

    def search(self) -> bool:
        for b in self.element.find_all("body"):
            if hasattr(b, "name") and "gripper" in b.name:
                return True
        return False


class RobotiqHeuristic(GripperHeuristic):
    @property
    def num_joints(self) -> int:
        return 1

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("robotiq_2f85_mj_description")

    def search(self) -> bool:
        for b in self.element.find_all("body"):
            if hasattr(b, "name") and "gripper" in b.name:
                return True
        return False


def get_all_gripper_heuristics_classes() -> List[type[GripperHeuristic]]:
    logger.warning("Not fully implemented yet. Only Robotiq grippers are supported.")

    return [RobotiqHeuristic]
