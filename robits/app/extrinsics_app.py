#!/usr/bin/env python3

"""
App for extrinsic calibration
"""
import sys
import os

import time
from threading import Thread

from datetime import datetime


import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

from textual.app import App
from textual.app import ComposeResult
from textual.containers import Vertical
from textual.containers import Horizontal
from textual.widgets import Label
from textual.widgets import Button
from textual_slider import Slider
from textual.widgets import Select


from robits.core.config_manager import config_manager
from robits.core.factory import CameraFactory
from robits.core.config import CameraCalibration
from robits.utils import vision_utils


class SimplePCDViewer:

    def __init__(self) -> None:
        self.viewer_running = True
        self.prev_transform = np.identity(4)
        self.table = None

        self.point_cloud = o3d.geometry.PointCloud()
        # self.point_cloud.points = o3d.utility.Vector3dVector(np.zeros((10, 3)))
        self.prev_transform = np.identity(4)

    def apply_transformation(self, transformation: np.ndarray) -> None:
        """
        Apply a transformation matrix to the current point cloud.

        :param transformation: the transformation to apply
        """
        self.point_cloud.transform(np.linalg.inv(self.prev_transform))
        self.point_cloud.transform(transformation)
        self.prev_transform = transformation

    def update_point_cloud(self, camera) -> None:
        """
        Get a new point clodu from the camera
        :param camera: the camera to retrieve the point cloud from
        """
        camera_data, _ = camera.get_camera_data()
        pcd = vision_utils.depth_to_pcd(camera_data, camera)
        self.point_cloud.points = pcd.points
        self.point_cloud.colors = pcd.colors

    def show_table(self):
        pass

    def get_table(self):
        table = o3d.geometry.TriangleMesh.create_box(width=0.6, height=1.5, depth=0.01)
        table = table.translate(np.array([0.3, 0.0 - 1.5 / 2, 0.295 - 0.01 / 2.0]))
        return table

    def run(self) -> None:
        """
        Start the Open3D viewer in a separate thread.

        .. todo:: this breaks compatibility to macOS as the gui is supposed to run in the main thread.
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Point Cloud Viewer")

        coordinate_system = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.4, origin=[0, 0, 0]
        )
        vis.add_geometry(coordinate_system)
        vis.add_geometry(self.get_table())
        vis.add_geometry(self.point_cloud)

        vis.add_geometry(
            o3d.geometry.TriangleMesh.create_box(
                width=1.0, height=0.001, depth=1.0
            ).paint_uniform_color([1.0, 0.0, 0.0])
        )
        vis.add_geometry(
            o3d.geometry.TriangleMesh.create_box(
                width=1.0, height=1.0, depth=0.001
            ).paint_uniform_color([0.0, 1.0, 0.0])
        )
        vis.add_geometry(
            o3d.geometry.TriangleMesh.create_box(width=0.001, height=1.0, depth=1.0)
            .paint_uniform_color([0.0, 0.0, 1.0])
            .translate(np.array([0.4687, 0.0, 0.0]))
        )

        while self.viewer_running:
            vis.update_geometry(self.point_cloud)
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.05)

        vis.destroy_window()

    def stop(self) -> None:
        """
        Stop the Open3D viewer.
        """
        self.viewer_running = False


class CameraCalibrationApp(App):

    def __init__(self, viewer: SimplePCDViewer):
        super().__init__()
        self.viewer = viewer

    def compose(self) -> ComposeResult:
        yield Vertical(
            Label("current transform", id="label_transform"),
            Horizontal(
                Label("Select Camera"),
                Select.from_values(
                    values=config_manager.available_cameras, id="camera_select"
                ),
                Button(label="Connect", id="connect"),
            ),
            Label("Camera Calibration - Translation Sliders"),
            Horizontal(
                Vertical(
                    Label("Translation X"),
                    Slider(min=-3.0, max=3.0, value=0.0, step=0.0025, id="trans_x"),
                    Label(id="label_x"),
                ),
                Vertical(
                    Label("Translation Y"),
                    Slider(min=-3.0, max=3.0, value=0.0, step=0.0025, id="trans_y"),
                    Label(id="label_y"),
                ),
                Vertical(
                    Label("Translation Z"),
                    Slider(min=-3.0, max=3.0, value=0.0, step=0.0025, id="trans_z"),
                    Label(id="label_z"),
                ),
            ),
            Label("Camera Calibration - Rotation Sliders"),
            Horizontal(
                Vertical(
                    Label("Rotation Roll"),
                    Slider(min=-180, max=180, value=0, step=1, id="rot_roll"),
                    Label(id="label_roll"),
                ),
                Vertical(
                    Label("Rotation Pitch"),
                    Slider(min=-180, max=180, value=0, step=1, id="rot_pitch"),
                    Label(id="label_pitch"),
                ),
                Vertical(
                    Label("Rotation Yaw"),
                    Slider(min=-180, max=180, value=0, step=1, id="rot_yaw"),
                    Label(id="label_yaw"),
                ),
            ),
            Label("Actions"),
            Horizontal(
                Button(label="Save", id="save"),
                Button(label="Refine", id="refine"),
                Button(label="Exit", id="exit"),
            ),
        )

    def on_mount(self):
        # Start the Open3D viewer in a separate thread. Does not work for macOS
        Thread(target=self.viewer.run, daemon=True).start()

    def on_button_pressed(self, message: Button.Pressed) -> None:
        if message.button.id == "exit":
            sys.exit(0)
        elif message.button.id == "connect":
            selected_camera = self.query_one("#camera_select", Select).value
            if selected_camera:
                camera = CameraFactory(selected_camera).build()
                self.viewer.update_point_cloud(camera)
                transformation = np.linalg.inv(camera.calibration.extrinsics)
                self.update_sliders(transformation)
                self.viewer.apply_transformation(self.get_transformation())
        elif message.button.id == "save":
            selected_camera = self.query_one("#camera_select", Select).value
            self.save_camera_calibration(selected_camera)
        elif message.button.id == "refine":
            self.estimate_transformation()

    def update_sliders(self, extrinsics):
        rpy = R.from_matrix(extrinsics[:3, :3]).as_euler("ZYX", degrees=True)
        self.query_one("#trans_x", Slider).value = extrinsics[0, 3]
        self.query_one("#trans_y", Slider).value = extrinsics[1, 3]
        self.query_one("#trans_z", Slider).value = extrinsics[2, 3]
        self.query_one("#rot_roll", Slider).value = rpy[2]
        self.query_one("#rot_pitch", Slider).value = rpy[1]
        self.query_one("#rot_yaw", Slider).value = rpy[0]

    def draw_registration_result(self, source, target, transformation):
        import copy

        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        # source_temp.transform(transformation)
        coordinate_system = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.4, origin=[0, 0, 0]
        )
        o3d.visualization.draw_geometries([source_temp, target_temp, coordinate_system])

    def estimate_transformation(self):
        """
        .. todo:: not implemented yet
        """

        pcd = self.viewer.point_cloud
        """
        if False:
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=0.03, ransac_n=5000, num_iterations=1000
            )
            inlier_cloud = pcd.select_by_index(inliers)
            inlier_cloud.paint_uniform_color([1.0, 0, 0])
            outlier_cloud = pcd.select_by_index(inliers, invert=True)
            extracted_table_pcd = inlier_cloud
        
        """

        extracted_table_pcd = pcd

        table_height = 0.25
        x = np.linspace(0, 0.5, 100)
        y = np.linspace(-0.7, 0.7, 100)
        mesh_x, mesh_y = np.meshgrid(x, y)
        xyz = np.zeros((100 * 100, 3))
        xyz[:, 0] = mesh_x.reshape(-1)
        xyz[:, 1] = mesh_y.reshape(-1)
        xyz[:, 2] = table_height

        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(xyz)
        target.paint_uniform_color([1.0, 0.0, 0.0])

        trans_init = self.get_transformation()
        threshold = 0.02
        self.draw_registration_result(extracted_table_pcd, target, trans_init)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            extracted_table_pcd,
            target,
            threshold,
            trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        )
        # self.update_sliders(reg_p2p.transformation)
        self.draw_registration_result(
            extracted_table_pcd, target, reg_p2p.transformation
        )

    def get_transformation(self) -> np.ndarray:
        trans_x = self.query_one("#trans_x", Slider).value
        trans_y = self.query_one("#trans_y", Slider).value
        trans_z = self.query_one("#trans_z", Slider).value
        roll = self.query_one("#rot_roll", Slider).value
        pitch = self.query_one("#rot_pitch", Slider).value
        yaw = self.query_one("#rot_yaw", Slider).value

        self.query_one("#label_x", Label).update(content=f"{trans_x:.2f}")
        self.query_one("#label_y", Label).update(content=f"{trans_y:.2f}")
        self.query_one("#label_z", Label).update(content=f"{trans_z:.2f}")

        self.query_one("#label_roll", Label).update(content=f"{roll:.2f}")
        self.query_one("#label_pitch", Label).update(content=f"{pitch:.2f}")
        self.query_one("#label_yaw", Label).update(content=f"{yaw:.2f}")

        # Create transformation matrix
        transformation = np.identity(4)
        transformation[:3, 3] = [trans_x, trans_y, trans_z]
        transformation[:3, :3] = R.from_euler(
            "ZYX", [yaw, pitch, roll], degrees=True
        ).as_matrix()
        # transformation = np.linalg.inv(transformation)

        self.query_one("#label_transform", Label).update(content=f"{transformation}")

        return transformation

    def on_slider_changed(self, message: Slider.Changed) -> None:
        transformation = self.get_transformation()

        # Apply transformation to the point cloud
        self.viewer.apply_transformation(transformation)

    def on_shutdown(self):
        # Stop the Open3D viewer when the app is closed
        self.viewer.stop()

    def save_camera_calibration(self, selected_camera):

        if not selected_camera:
            return

        user_config_dir = config_manager.get_user_config_dir()
        extrinsics = np.linalg.inv(self.get_transformation())

        camera = CameraFactory(selected_camera).build()
        config_dict = camera.calibration.to_dict()
        config_dict["extrinsics"] = extrinsics.tolist()
        config_dict["date_updated"] = datetime.now().isoformat()
        new_calibration = CameraCalibration(**config_dict)
        config_path = os.path.join(
            user_config_dir, f"calibration_{camera.camera_name}_camera.json"
        )
        new_calibration.save_config(config_path)


if __name__ == "__main__":
    # import asyncio
    # loop = asyncio.new_event_loop()
    # asyncio.set_event_loop(loop)

    print("Viewer initialized")
    viewer = SimplePCDViewer()
    print("Creating app")
    app = CameraCalibrationApp(viewer)
    print("Running app")
    app.run()
