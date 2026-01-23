from typing import List

from abc import ABC
from abc import abstractmethod

from dm_control import mjcf

from robits.sim.blueprints import RobotBlueprint
from robits.sim.blueprints import RobotDescriptionModel

from robits.sim import mjcf_utils


class RobotHeuristic(ABC):
    def __init__(self, element: mjcf.Element, joint_positions):
        self.element = element
        self.joint_positions = joint_positions

    def to_blueprint(self) -> RobotBlueprint:
        pose = mjcf_utils.pose_from_element(self.element)
        name = f"/{self.element.name}"

        attachment = None
        default_joint_positions = self.joint_positions[: self.num_joints]
        return RobotBlueprint(
            name,
            self.model(),
            pose,
            attachment,
            default_joint_positions=default_joint_positions,
        )

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


class PandaHeuristic(RobotHeuristic):
    @property
    def num_joints(self) -> int:
        return 7

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("panda_mj_description", "panda_nohand.xml")

    def search(self) -> bool:
        return bool("panda" in getattr(self.element, "name", ""))


class UR10eHeuristic(RobotHeuristic):
    @property
    def num_joints(self) -> int:
        return 6

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("ur10e_mj_description")

    def search(self) -> bool:
        return bool("ur10e" in getattr(self.element, "name", ""))


class IIWAHeuristic(RobotHeuristic):
    @property
    def num_joints(self) -> int:
        return 7

    def model(self) -> RobotDescriptionModel:
        return RobotDescriptionModel("iiwa14_mj_description")

    def search(self) -> bool:
        return bool("iiwa" in getattr(self.element, "name", ""))


def get_all_heuristics_classes() -> List[type[RobotHeuristic]]:
    return [PandaHeuristic, UR10eHeuristic, IIWAHeuristic]
