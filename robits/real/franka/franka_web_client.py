"""
copied from frankx.robot.Robot

.. seealso:: frankx.robot.Robot
"""

import base64
import json
import hashlib
from http.client import HTTPSConnection
import ssl


import logging

logger = logging.getLogger(__name__)


class FrankaWebClient:
    def __init__(self, fci_ip, user=None, password=None):
        self.hostname = fci_ip
        self.user = user
        self.password = password
        self.client = None
        self.token = None

    @staticmethod
    def _encode_password(user, password):
        bs = ",".join(
            [
                str(b)
                for b in hashlib.sha256(
                    (password + "#" + user + "@franka").encode("utf-8")
                ).digest()
            ]
        )
        return base64.encodebytes(bs.encode("utf-8")).decode("utf-8")

    def __enter__(self):
        self.client = HTTPSConnection(
            self.hostname, timeout=12, context=ssl._create_unverified_context()
        )  # [s]
        self.client.connect()
        self.client.request(
            "POST",
            "/admin/api/login",
            body=json.dumps(
                {
                    "login": self.user,
                    "password": self._encode_password(self.user, self.password),
                }
            ),
            headers={"content-type": "application/json"},
        )
        self.token = self.client.getresponse().read().decode("utf8")
        return self

    def __exit__(self, type, value, traceback):
        self.client.close()

    def unlock_brakes(self):
        logger.info("Unlocking robot breaks.")
        self.client.request(
            "POST",
            "/desk/api/robot/open-brakes",
            headers={
                "content-type": "application/x-www-form-urlencoded",
                "Cookie": f"authorization={self.token}",
            },
        )
        response = self.client.getresponse().read()

        logger.info("Reponse is %s", response)

        return response

    def lock_brakes(self):

        logger.info("Locking robot breaks.")

        self.client.request(
            "POST",
            "/desk/api/robot/close-brakes",
            headers={
                "content-type": "application/x-www-form-urlencoded",
                "Cookie": f"authorization={self.token}",
            },
        )

        response = self.client.getresponse().read()

        logger.info("Reponse is %s", response)

        return response
