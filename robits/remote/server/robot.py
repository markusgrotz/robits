from robits.core.abc.robot import UnimanualRobot
from robits.remote.server.server_base import ZMQServerBase


class RobotZMQServer(ZMQServerBase):
    """
    Robot client using ZMQ
    """

    def __init__(self, robot: UnimanualRobot, port: int = 5050, **kwargs):
        super().__init__(robot, port)
