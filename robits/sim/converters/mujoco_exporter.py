from typing import Sequence

import logging

from pathlib import Path

from robits.sim.blueprints import Blueprint
from robits.sim.model_factory import SceneBuilder

logger = logging.getLogger(__name__)


class MujocoXMLExporter:
    def export_scene(self, out_path: Path, blueprints: Sequence[Blueprint]) -> None:
        logger.info("building mujoco model.")
        builder = SceneBuilder(add_floor=False)
        builder.build_from_blueprints(blueprints)
        builder.export_with_assets(out_path)

        logger.info("Exported MuJoCo scene to %s.", out_path)
