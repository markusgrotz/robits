from typing import Tuple
from typing import Sequence
from typing import Union

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation as R


@dataclass(frozen=True)
class CartesianAction:
    """
    Defines a cartesian action consisting of position, orientation, and the gripper state.

    :ivar position: The position in meters.
    :ivar quaternion: The orientation represented as a quaternion in xyzw format.
    :ivar hand_open: A flag indicating whether the gripper is open (True) or closed (False).
    """

    position: np.ndarray
    quaternion: np.ndarray
    hand_open: bool

    @property
    def rot_matrix(self) -> np.ndarray:
        """
        Computes the orientation of the action as a 3x3 rotation matrix.

        :returns: A 3x3 rotation matrix representing the orientation.
        """
        q = self.quaternion.tolist()
        return R.from_quat(q).as_matrix()

    def to_matrix(self) -> np.ndarray:
        """
        Computes the action as a homogeneous 4x4 transformation matrix.

        :returns: A 4x4 homogeneous transformation matrix representing the pose.
        """
        pose = np.identity(4)
        pose[:3, :3] = self.rot_matrix
        pose[:3, 3] = self.position
        return pose

    @classmethod
    def from_matrix(cls, pose: np.ndarray, hand_open: bool = True) -> "CartesianAction":
        """
        Creates a CartesianAction instance from a given 4x4 pose matrix.

        :param pose: A 4x4 transformation matrix representing the pose.
        :param hand_open: A boolean flag indicating whether the gripper is open.
        :returns: A CartesianAction instance.
        """
        quaternion = R.from_matrix(pose[:3, :3]).as_quat()
        position = pose[:3, 3]
        return CartesianAction(position, quaternion, hand_open)

    @classmethod
    def parse(cls, action: Union[Sequence[float], np.ndarray]) -> "CartesianAction":
        """
        Parses a sequence of float values into a CartesianAction.

        :param action: A sequence containing position, quaternion, and gripper state.
        :returns: A CartesianAction instance.
        """
        if isinstance(action, (list, tuple)):
            action = np.asarray(action)
        return cls.from_numpy(action)

    @classmethod
    def from_numpy(cls, action: np.ndarray) -> "CartesianAction":
        """
        Creates a CartesianAction instance from a NumPy array.

        :param action: A NumPy array of length 9 containing position, quaternion, and gripper state.
        :returns: A CartesianAction instance.
        :raises ValueError: If the input array does not have exactly 9 elements.
        """

        if len(action) != 8:
            raise ValueError("Input action array must have exactly 9 elements.")

        position = action[:3]
        quaternion = action[3:7]
        hand_open = action[7] > 0.5

        return CartesianAction(position, quaternion, hand_open)

    @property
    def quaternion_as_tuple(self) -> Tuple[float]:
        """
        Converts the quaternion to a tuple format. Quaternion is stored in the xyzw format

        :returns: A tuple representation of the quaternion.
        """
        return tuple(self.quaternion.tolist())

    @property
    def position_as_tuple(self) -> Tuple[float]:
        """
        Converts the position to a tuple format.

        :returns: A tuple representation of the position.
        """
        return tuple(self.position.tolist())

    def to_numpy(self) -> np.ndarray:
        """
        Converts the CartesianAction instance into a NumPy array in the form (position, quaternion, gripper state)

        :returns: A NumPy array representing position, quaternion, and gripper state.
        """
        return np.concatenate(
            [
                self.position,
                self.quaternion,
                np.array([self.hand_open], dtype=np.float32),
            ],
            axis=0,
        )


@dataclass(frozen=True)
class BimanualAction:
    """
    Represents a bimanual action, which consists of separate actions for the right and left arms.

    :ivar right_action: The action for the right hand.
    :ivar left_action: The action for the left hand.
    """

    right_action: CartesianAction
    left_action: CartesianAction

    @classmethod
    def from_numpy(cls, action: np.ndarray) -> "BimanualAction":
        """
        Creates a BimanualAction instance from a NumPy array.

        :param action: A NumPy array containing both right and left actions.
        :returns: An instance of BimanualAction with parsed right and left actions.
        :raises ValueError: If the input array cannot be split into two equal parts.
        """
        if action.shape[0] % 2 != 0:
            raise ValueError(
                "The input action array must have an even number of elements."
            )

        np_right_action, np_left_action = np.split(action, 2)

        right_action = CartesianAction.from_numpy(np_right_action)
        left_action = CartesianAction.from_numpy(np_left_action)

        return BimanualAction(right_action=right_action, left_action=left_action)

    def to_numpy(self) -> np.ndarray:
        """
        Converts the BimanualAction instance into a NumPy array.

        :returns: A concatenated NumPy array representing both right and left actions.
        """
        return np.concatenate(
            [self.right_action.to_numpy(), self.left_action.to_numpy()], axis=0
        )
