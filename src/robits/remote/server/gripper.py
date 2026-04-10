from robits.core.abc.gripper import GripperBase
from robits.remote.server.server_base import ZMQServerBase


class GripperZMQServer(ZMQServerBase):
    """
    Gripper client using for ZMQ
    """

    def __init__(self, gripper: GripperBase, port: int = 5070, **kwargs):
        super().__init__(gripper, port)
