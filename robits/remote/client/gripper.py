from typing import Dict
from typing import Any

from robits.remote.client.client_base import ZMQClient


class GripperZMQClient:
    """
    Gripper client using ZMQ
    """

    def __init__(self, address="localhost", port=5070, **kwargs):
        self.client = ZMQClient(address, port)

    def open(self) -> None:
        self.client.call("open")

    def close(self) -> None:
        self.client.call("close")

    def get_obs(self) -> Dict[str, Any]:
        return self.client.call("get_obs")

    def is_open(self) -> bool:
        return self.client.call("is_open")

    def get_info(self) -> Dict[str, Any]:
        return self.client.call("get_info")

    def get_gripper_name(self) -> str:
        return self.client.call("gripper_name")
