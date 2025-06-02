from typing import Dict
from typing import Any
from typing import Tuple
from typing import List

from robits.core.data_model.action import CartesianAction

from robits.remote.client.client_base import ZMQClient

from robits.core.abc.control import ControlManager
from robits.core.abc.control import ControllerBase
from robits.core.abc.control import ControlTypes


class RemoteController(ControllerBase):

    def __init__(self, client, controller_type: ControlTypes):
        self.client = client
        self.controller_type = controller_type

    def update(self, *args, **kwargs):
        self.client.call(
            "control.active_controller.update", *args, **kwargs, no_return=True
        )


class RemoteControlManager(ControlManager):

    def __init__(self, client):
        self.client = client
        self.controller_type = None

    def __call__(
        self, controller_type: ControlTypes, *args, **kwargs
    ) -> "ControlManager":
        self.client.call(
            "control.__call__", controller_type, *args, **kwargs, no_return=True
        )
        self.controller_type = controller_type
        return self

    def __enter__(self):
        self.client.call("control.__enter__", no_return=True)
        if self.controller_type is None:
            raise Exception("Controller type should not be empty")
        return RemoteController(self.client, self.controller_type)

    def __exit__(self, *args, **kwargs) -> None:
        self.controller_type = None
        self.client.call("control.__exit__", *args, **kwargs)

    def move_home(self) -> None:
        self.client.call("control.move_home")


class RobotZMQClient:
    """
    Robot client using ZMQ
    """

    def __init__(self, address="localhost", port=5050, **kwargs):
        self.client = ZMQClient(address, port)
        self.control = RemoteControlManager(self.client)

    def get_robot_name(self) -> str:
        return self.client.call("robot_name")

    def get_proprioception_data(
        self, include_eef=True, include_gripper_obs=True
    ) -> Dict[str, Any]:
        return self.client.call(
            "get_proprioception_data",
            include_eef=include_eef,
            include_gripper_obs=include_gripper_obs,
        )

    @property
    def eef_pose(self) -> Tuple[List[float], List[float]]:
        return self.client.call("eef_pose")

    @property
    def eef_matrix(self) -> List[List[float]]:
        return self.client.call("eef_matrix")

    def get_info(self) -> Dict[str, Any]:
        return self.client.call("get_info")

    def control_arm(self, action: CartesianAction, auto_recover=True) -> bool:
        return self.client.call(
            "control_arm", action=action.__dict__, auto_recover=auto_recover
        )

    def control_hand(self, action: CartesianAction):
        return self.client.call("control_hand", action=action.__dict__)
