from typing import Tuple
from typing import List
from abc import ABC
from abc import abstractmethod

import logging

import mujoco

from robits.sim.blueprints import GripperBlueprint
from robits.sim.blueprints import Attachment
from robits.sim.blueprints import RobotDescriptionModel

logger = logging.getLogger(__name__)


class GripperHeuristic(ABC):
    def __init__(self, element: mujoco.MjsElement, joint_positions):
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
            if "panda hand/finger_joint1" in b.name:
                return True
        return False


class AllegroHeuristic(GripperHeuristic):
    @property
    def num_joints(self) -> int:
        return 16

    @property
    def prefix(self) -> str:
        return "allegro_right"

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("allegro_hand_mj_description")

    def search(self) -> bool:
        for b in self.element.find_all("joint"):
            if "allegro_right/ffj0" in b.name:
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
            if "robotiq_2f85/right_coupler_joint" in b.name:
                return True
        return False


class AbilityHand(GripperHeuristic):
    @property
    def num_joints(self) -> int:
        return 10

    @property
    def prefix(self) -> str:
        return "ability_hand"

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel(
            "ability_hand_mj_description", variant_name="./hands/abh_right_large.xml"
        )

    def search(self) -> bool:
        for b in self.element.find_all("joint"):
            if "ability_hand/index_mcp" in b.name:
                return True
        return False


class XArmGripper(GripperHeuristic):
    @property
    def num_joints(self) -> int:
        return 1

    @property
    def prefix(self) -> str:
        return "xarm7 hand"

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("xarm7_mj_description", variant_name="hand.xml")

    def search(self) -> bool:
        for b in self.element.find_all("joint"):
            if "xarm7 hand/left_finger_joint" in b.name:
                return True
        return False


def get_all_gripper_heuristics_classes() -> List[type[GripperHeuristic]]:
    return [
        RobotiqHeuristic,
        PandaHeuristic,
        XArmGripper,
        AbilityHand,
        AllegroHeuristic,
    ]
