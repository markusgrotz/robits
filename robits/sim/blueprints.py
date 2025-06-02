"""
Class that contains blueprints to the scene model and objects
"""

from typing import Optional
from typing import Sequence

from abc import ABC

from dataclasses import dataclass
from dataclasses import field

import numpy as np
from scipy.spatial.transform import Rotation as R


@dataclass(frozen=True)
class Blueprint(ABC):
    """
    Simple data class to model elements in the environment
    """

    name: str

    @property
    def id(self) -> str:
        return f"{self.__class__.__name__.lower()}_{self.name}"


@dataclass(frozen=True)
class Pose:

    matrix: np.ndarray = field(default_factory=lambda: np.identity(4))

    # is_relative: bool = False

    @property
    def position(self):
        return self.matrix[:3, 3]

    def with_position(self, new_position: Sequence[float]):
        new_matrix = self.matrix.copy()
        new_matrix[:3, 3] = np.asarray(new_position, dtype=float)
        return Pose(matrix=new_matrix)

    def with_quat(self, new_quat: Sequence[float]):
        new_matrix = self.matrix.copy()
        new_matrix[:3, :3] = R.from_quat(new_quat).as_matrix()
        return Pose(matrix=new_matrix)

    @property
    def quaternion(self):
        return R.from_matrix(self.matrix[:3, :3]).as_quat()

    @property
    def quaternion_wxyz(self):
        # scalar_first is only available in SciPy >= 1.4. Which does not work for Python 3.9
        q = R.from_matrix(self.matrix[:3, :3]).as_quat()
        return np.concatenate((q[-1:], q[:-1]))

    @property
    def euler(self):
        return R.from_matrix(self.matrix[:3, :3]).as_euler(seq="XYZ")

    def __post_init__(self):
        if self.matrix.shape != (4, 4):
            raise ValueError("pose must be a 4x4 transformation matrix")


@dataclass(frozen=True)
class CameraBlueprint(Blueprint):
    pass


@dataclass(frozen=True)
class GeomBlueprint(Blueprint):

    geom_type: str = "box"

    pose: Optional[Pose] = None

    size: Sequence[float] = field(default_factory=lambda: [0.02, 0.02, 0.02])

    rgba: Optional[Sequence[float]] = None

    is_static: bool = False


@dataclass(frozen=True)
class ObjectBlueprint(Blueprint):

    model_path: str

    pose: Optional[Pose] = None

    is_static: bool = True

    model_prefix_name: Optional[str] = None


@dataclass(frozen=True)
class RobotDescriptionModel:

    description_name: str

    variant_name: Optional[str] = None

    model_prefix_name: Optional[str] = None


@dataclass(frozen=True)
class Attachment:

    blueprint_id: str

    wrist_name: str

    wrist_pose: Optional[Pose] = None

    attachment_site: str = "attachment_site"


@dataclass(frozen=True)
class RobotBlueprint(Blueprint):

    model: RobotDescriptionModel

    pose: Optional[Pose] = None

    attachment: Optional[Attachment] = None


@dataclass(frozen=True)
class GripperBlueprint(Blueprint):

    model: RobotDescriptionModel

    # delta_offset: Optional[Pose] = None
