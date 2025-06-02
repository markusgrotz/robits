from robits.core.abc.camera import CameraBase
from robits.remote.server.server_base import ZMQServerBase


class CameraZMQServer(ZMQServerBase):
    """
    Camera client using ZMQ.
    """

    def __init__(self, camera: CameraBase, port: int = 5060, **kwargs):
        super().__init__(camera, port)
