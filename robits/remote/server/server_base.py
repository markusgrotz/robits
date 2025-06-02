from typing import Dict
from typing import Any

import traceback
import json
import logging

import zmq

from robits.core.utils import MiscJSONEncoder


logger = logging.getLogger(__name__)


class ZMQServerBase:
    """
    Base class for ZMQ server implementation
    """

    def __init__(self, target: Any, port: int) -> None:
        self.target = target
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{port}")
        logger.info("Wrapping target %s. Listening on *:%d", target, port)
        self.is_stopped = False

    def stop(self) -> None:
        self.is_stopped = True

    def listen(self) -> None:
        """
        Waits for a request, calls the requested method and returns the result
        """
        logger.warning("Listening for %s:%s", type(self.target), self.target)
        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)

        try:
            while not self.is_stopped:
                events = dict(poller.poll(1000))  # 1-second timeout
                if self.socket in events:
                    try:
                        msg: Dict[str, Any] = self.socket.recv_json()
                        logger.debug("Received message: %s", msg)
                        method = msg.get("method")
                        args = msg.get("args", [])
                        kwargs = msg.get("kwargs", {})

                        no_return = kwargs.pop("no_return", False)

                        func = self._resolve_method(method)

                        # func = getattr(self.target, method)
                        result = func(*args, **kwargs) if callable(func) else func

                        if not no_return:
                            self.socket.send_json(self._serialize_result(result))
                        else:
                            self.socket.send_json({"status": "ok", "result": "{}"})

                    except Exception as e:
                        logger.error(
                            "Error while handling request from client. Exception: %s", e
                        )
                        self.socket.send_json(
                            {
                                "status": "error",
                                "error": str(e),
                                "traceback": traceback.format_exc(),
                            }
                        )
            print("STOPPED")
        except KeyboardInterrupt:
            logger.info("ZMQ Server interrupted by user.")
        finally:
            self.socket.close()
            self.context.term()

    def _resolve_method(self, method_path: str):
        parts = method_path.split(".")
        attr = self.target
        for part in parts:
            attr = getattr(attr, part)
        return attr

    def _serialize_result(self, result: Any) -> Dict[str, Any]:
        """
        Serializes the result to json. Return an error if unable to serialize

        :returns: serialized result as json
        """
        try:
            json_result = json.dumps(result, cls=MiscJSONEncoder)
            return {"status": "ok", "result": json_result}
        except TypeError as e:
            logger.warning(
                "Unable to serialize data. Exception: %s Data: %s ", e, result
            )
            return {"status": "error", "error": "Unserializable result"}
