from typing import List
from typing import Dict

from functools import lru_cache

import logging
import json

from robits.sim.blueprints import Blueprint
from robits.sim.blueprints import CameraBlueprint
from robits.sim.blueprints import GeomBlueprint
from robits.sim.blueprints import Pose


logger = logging.getLogger(__name__)


class EnvDesigner:
    """
    Collects all the necessary requirments to build the environemnt. Kind of like an architect.
    """

    def __init__(self) -> None:
        self.blueprints: Dict[str, Blueprint] = {}
        self.assembled = False

        self.add_defaults()

    def to_json(self) -> str:
        """
        Serializes all blueprints to json
        """
        return json.dumps({"blueprints": [vars(b) for b in self.blueprints.values()]})

    def from_json(self, json_string: str):
        """
        Load blueprints from json
        """
        json_data = json.loads(json_string)
        self.blueprints = {}
        for b in json_data["blueprints"]:
            raise NotImplementedError("not implemented yet")
            # self.blueprints[blueprint.id] = blueprint

    def add_floor(self):
        """
        .. todo:: material is missing.
        """

        self.add(
            GeomBlueprint(
                "floor",
                "plane",
                Pose(),
                size=[0, 0, 0.05],
                rgba=[0.5, 0.5, 0.5, 1],
                is_static=True,
            )
        )

    def add_defaults(self):
        self.add_blocks()
        # self.add_light()
        # self.add_floor()

    def finalize(self) -> Dict[str, Blueprint]:
        """
        Finalizes the layout of the environment

        :returns: all blueprints
        """
        self.assembled = True

        # blueprints, self.blueprints = self.blueprints, {}
        return self.blueprints

    def add(self, blueprint: Blueprint) -> "EnvDesigner":
        if self.assembled:
            raise RuntimeError("Environment already build.")

        if blueprint.id in self.blueprints:
            logger.error("Blueprint already added with id %s", blueprint.id)
            return self

        logger.debug("Adding gripper blueprint %s", blueprint)

        self.blueprints[blueprint.id] = blueprint
        return self

    def get_camera_names(self) -> List[str]:
        """
        Returns the name of all cameras
        """
        return [
            v.name for _, v in self.blueprints.items() if isinstance(v, CameraBlueprint)
        ]

    def add_blocks(self):
        blocks = [
            {
                "name": "red_block",
                "rgba": [1, 0, 0, 1],
                "pose": [0.5, -0.2, 0.02],
            },
            {
                "name": "green_block",
                "rgba": [0, 1, 0, 1],
                "pose": [0.5, 0.0, 0.02],
            },
            {
                "name": "blue_block",
                "rgba": [0, 0, 1, 1],
                "pose": [0.5, 0.2, 0.02],
            },
        ]
        for block in blocks:
            size = [0.02, 0.02, 0.02]
            pose = Pose().with_position(block["pose"])
            self.add(
                GeomBlueprint(
                    name=block["name"], rgba=block["rgba"], pose=pose, size=size
                )
            )

    @classmethod
    @lru_cache(maxsize=1)
    def get(cls) -> "EnvDesigner":
        """
        Singleton instance
        """
        return EnvDesigner()


env_designer = EnvDesigner.get()
