"""
Class that contains blueprints for the scene such as robots and objects
"""
from typing import Optional
from typing import Any
from typing import Sequence
from typing import Dict
from typing import List

from abc import ABC
import importlib
import json

from dataclasses import dataclass
from dataclasses import field
from dataclasses import fields as dc_fields
from dataclasses import is_dataclass
from dataclasses import replace

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

    def to_dict(self) -> Dict:
        """
        Serialize this blueprint (and nested dataclasses) to a plain dict.
        """
        def _convert(obj):
            if is_dataclass(obj):
                cls = obj.__class__
                out = {"class_path": f"{cls.__module__}.{cls.__name__}"}
                for f in dc_fields(obj):
                    out[f.name] = _convert(getattr(obj, f.name))
                return out
            if isinstance(obj, (list, tuple)):
                return [_convert(v) for v in obj]
            if isinstance(obj, dict):
                return {k: _convert(v) for k, v in obj.items()}
            return obj

        return _convert(self)


@dataclass(frozen=True)
class Pose:

    matrix: np.ndarray = field(default_factory=lambda: np.identity(4))

    def with_position(self, new_position: Sequence[float]):
        new_matrix = self.matrix.copy()
        new_matrix[:3, 3] = np.asarray(new_position, dtype=float)
        return replace(self, matrix=new_matrix)

    def with_quat(self, new_quat: Sequence[float]):
        new_matrix = self.matrix.copy()
        new_matrix[:3, :3] = R.from_quat(new_quat).as_matrix()
        return replace(self, matrix=new_matrix)
    
    def with_quat_wxyz(self, new_quat: Sequence[float]):
        return self.with_quat(np.concatenate((new_quat[1:], new_quat[:1])))
    
    def with_euler(self, new_euler: Sequence[float], degrees=False):
        new_matrix = self.matrix.copy()
        new_matrix[:3, :3] = R.from_euler('XYZ', new_euler, degrees).as_matrix()
        return replace(self, matrix=new_matrix)
    
    @property
    def position(self):
        return self.matrix[:3, 3]

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

    def to_dict(self) -> Dict:
        return self.matrix.tolist()

@dataclass(frozen=True)
class CameraBlueprint(Blueprint):

    width: int

    height: int

    intrinsics: np.ndarray

    pose: Optional[Pose] = None

    @property
    def extrinsics(self):
        """
        world to camera coordinates
        """
        if self.pose is None:
            return np.identity(4)
        return np.linalg.inv(self.pose.matrix)
    

@dataclass(frozen=True)
class BlueprintGroup(Blueprint):

    pose: Optional[Pose] = None
    

@dataclass(frozen=True)
class GeomBlueprint(Blueprint):

    geom_type: str = "box"

    pose: Optional[Pose] = None

    size: Sequence[float] = field(default_factory=lambda: [0.02, 0.02, 0.02])
    
    rgba: Optional[Sequence[float]] = None

    is_static: bool = False


@dataclass(frozen=True)
class MeshBlueprint(Blueprint):

    mesh_path: str

    pose: Optional[Pose] = None

    is_static: bool = False

    scale: float = 1.0


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

    """the id of the blueprint that we want to attach"""
    blueprint_id: str

    wrist_name: str

    wrist_pose: Optional[Pose] = None

    attachment_site: str = "attachment_site"


@dataclass(frozen=True)
class RobotBlueprint(Blueprint):

    model: RobotDescriptionModel

    pose: Optional[Pose] = None

    attachment: Optional[Attachment] = None

    default_joint_positions: Optional[Sequence[float]] = None


@dataclass(frozen=True)
class GripperBlueprint(Blueprint):

    model: RobotDescriptionModel

    # delta_offset: Optional[Pose] = None


def convert_json_to_bp(data: Any) -> Blueprint:

    def _build(obj: Any) -> Any:
        if isinstance(obj, list):
            return [_build(v) for v in obj]
        if isinstance(obj, dict) and "class_path" in obj:
            module_name, class_name = obj["class_path"].rsplit(".", 1)
            if not module_name == "robits.sim.blueprints":
                import logging
                logging.getLogger(__name__).error("Invalid module name %s in json data.", module_name)

            module = importlib.import_module(module_name)
            cls = getattr(module, class_name)
            kwargs = {}
            for f in dc_fields(cls):
                if f.name in obj:
                    val = _build(obj[f.name])
                    if cls is Pose and f.name == "matrix" and not isinstance(val, np.ndarray):
                        val = np.asarray(val, dtype=float)
                    kwargs[f.name] = val
            return cls(**kwargs)
        if isinstance(obj, dict):
            return {k: _build(v) for k, v in obj.items()}
        return obj

    return _build(data)


def blueprints_from_json(json_string: str) -> List[Blueprint]:
    blueprints = []
    json_data = json.loads(json_string)
    for item in json_data.get("blueprints", []):
        bp = convert_json_to_bp(item)
        blueprints.append(bp)
    return blueprints