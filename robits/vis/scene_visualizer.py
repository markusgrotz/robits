import threading
import time
import logging
from collections import defaultdict


import numpy as np
import open3d as o3d

from robits.core.data_model.action import CartesianAction
from robits.utils import vision_utils


logger = logging.getLogger(__name__)


class SceneVisualizer:

    def __init__(self, robot):
        self.robot = robot

        self.elements = {}
        self.has_changed = defaultdict(lambda: False)
        self.transformations = defaultdict(lambda: np.identity(4))

        self.thread = None
        self.lock = threading.Lock()
        self.is_running = False
        self.visible = True
        self.output_path = ""

    def set_output(self, path):
        if self.is_running:
            logger.warning("Already running")
            return

        self.visible = False
        self.output_path = path

        self.show()

    def show(self):

        if self.is_running:
            logger.warning("Already running")
            return

        for camera in self.robot.cameras:
            camera_name = camera.camera_name
            camera_data, _info = camera.get_camera_data()
            pcd = vision_utils.depth_to_pcd(camera_data, camera, apply_extrinsics=True)
            self.elements[camera_name] = pcd

        pose = self.robot.eef_matrix

        action = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.3, origin=[0, 0, 0]
        )
        self.elements["action"] = action.transform(pose)
        self.transformations["action"] = pose

        add_scene_bounds = False
        if add_scene_bounds:
            min_scene_bounds = np.array([0.0, -0.5, 0.0])
            max_scene_bounds = np.array([1.0, 0.5, 1.0])
            diff = max_scene_bounds - min_scene_bounds
            box_dim = dict(zip(["width", "height", "depth"], diff.tolist()))
            scene_bounds = o3d.geometry.TriangleMesh.create_box(**box_dim)
            scene_bounds.translate(min_scene_bounds, relative=True)
            scene_bounds.paint_uniform_color([1, 0, 0])

            # o3d.geometry.AxisAlignedBoundingBox

            self.elements["scene_bounds"] = scene_bounds

        self.thread = threading.Thread(target=self.render)
        self.is_running = True
        self.thread.start()

    def update_scene(self, camera_data=None, camera=None):

        data = {}
        if camera_data:
            pcd = vision_utils.depth_to_pcd(camera_data, camera, apply_extrinsics=True)
            camera_name = camera.camera_name
            data[camera_name] = pcd
        else:
            for camera in self.robot.cameras:
                camera_data, info = camera.get_camera_data()
                camera_name = camera.camera_name
                pcd = vision_utils.depth_to_pcd(
                    camera_data, camera, apply_extrinsics=True
                )
                data[camera_name] = pcd

        with self.lock:
            for k, v in data.items():
                self.elements[k].points = v.points
                self.elements[k].colors = v.colors
                self.has_changed[k] = True

    def render(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Visualization", visible=self.visible)

        coordinate_system = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.4, origin=[0, 0, 0]
        )
        vis.add_geometry(coordinate_system)

        with self.lock:
            for _, geometry in self.elements.items():
                vis.add_geometry(geometry)

        while self.is_running:
            with self.lock:
                for k, v in self.has_changed.items():
                    if v:
                        self.has_changed[k] = False
                        vis.update_geometry(self.elements[k])

            vis.poll_events()
            vis.update_renderer()

            if not self.visible:

                view_control = vis.get_view_control()

                # copied from the viewer
                view_pose = {
                    "front": [
                        0.012339452449784705,
                        -0.90282996236235535,
                        0.42982065675584702,
                    ],
                    "lookat": [
                        -0.12694871669500873,
                        0.11746709358396157,
                        0.88075116287669197,
                    ],
                    "up": [
                        0.025976937544841906,
                        0.42999774206857738,
                        0.90245617097547559,
                    ],
                    "zoom": 0.33999999999999964,
                }

                if False and view_control:
                    view_control.set_front(view_pose["front"])
                    view_control.set_up(view_pose["up"])
                    view_control.set_lookat(view_pose["lookat"])
                    view_control.set_zoom(view_pose["zoom"])
                else:
                    print("unable to get view control")

                vis.capture_screen_image(
                    f"{self.output_path}/{time.time()}.png", do_render=True
                )
                time.sleep(0.10)
            else:
                time.sleep(0.01)

        logger.info("Closing window.")
        vis.destroy_window()

        logger.info("Done window.")

    def close(self):
        if self.is_running:
            self.is_running = False
            logger.info("Waiting for thread")
            self.thread.join()
            logger.info("Done waiting for thread")

    def update_pose(self, pose: np.ndarray):
        with self.lock:
            previous_pose = self.transformations["action"]
            # ..todo:: concatenate transformations
            self.elements["action"] = self.elements["action"].transform(
                np.linalg.inv(previous_pose)
            )
            self.elements["action"] = self.elements["action"].transform(pose)
            self.transformations["action"] = pose
            self.has_changed["action"] = True

    def update_action(self, robot_action: CartesianAction):
        self.update_pose(robot_action.to_matrix())
