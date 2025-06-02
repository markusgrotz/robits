from typing import Dict
from typing import Any

import logging
import json

import zmq


logger = logging.getLogger(__name__)


class ZMQClient:
    """
    Base class for ZMQ client implementation
    """

    def __init__(self, address: str, port: int) -> None:
        """
        Connects to a ZMQ server

        :param address: the address to connect to
        :param port: the port to connect to
        """
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        logger.info("Connecting to %s:%d", address, port)
        self.socket.connect(f"tcp://{address}:{port}")
        logger.info("Connected to %s:%d", address, port)

    def call(self, method: str, *args, **kwargs: Dict[str, Any]):
        """
        Calls a remote method and returns the result

        :param method: name of the method to call
        """
        logger.debug("Calling method %s", method)
        self.socket.send_json({"method": method, "args": args, "kwargs": kwargs})
        reply = self.socket.recv_json()
        # reply = json.loads(reply)
        if reply["status"] != "ok":
            logger.error(
                "Unable to call method %s. Returned error is %s", method, reply["error"]
            )
            return reply

        return json.loads(reply["result"])
