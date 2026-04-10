from typing import Dict
from typing import Any
from typing import Optional

from robits.remote.client.client_base import ZMQClient


class GripperZMQClient:
    """
    Gripper client using ZMQ
    """

    def __init__(
        self,
        address: str = "localhost",
        port: int = 5070,
        client: Optional[Any] = None,
        **kwargs,
    ):
        self.client = client or ZMQClient(address, port)

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

    @property
    def gripper_name(self):
        return self.client.call("gripper_name")

    def set_pos(self, pos):
        return self.client.call("set_pos", pos)
