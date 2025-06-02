#!/usr/bin/env python3

import time
from pathlib import Path

import numpy as np
import open3d as o3d

# from rlbench.utils import get_stored_demos

import rich_click as click
from click_prompt import filepath_option

from robits.dataset.io.reader import DatasetReader
from robits.utils.vision_utils import depth_to_pcd


box_length = 0.01
box_dim = dict(zip(["width", "height", "depth"], [box_length] * 3))
box_offset = np.identity(4)
box_offset[:3, 3] = -box_length / 2


@click.command()
@filepath_option("--input-path", default="/tmp/test/")
@click.option("--show-visualization/--hide-visualization", is_flag=True, default=True)
@click.option("--add-trajectory/--hide-trajectory", is_flag=True, default=True)
def cli(input_path, show_visualization, add_trajectory):

    dataset = DatasetReader(Path(input_path)).load()

    camera_name = "front"

    visualization_data = []

    coordinate_system = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.1, origin=[0, 0, 0]
    )
    visualization_data.append(coordinate_system)

    # visualize trajectories
    if add_trajectory:

        for obs in dataset.entries:
            coordinate_system = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=0.01, origin=[0, 0, 0]
            )
            coordinate_system.transform(obs.proprioception["gripper_matrix"])
            visualization_data.append(coordinate_system)

    add_gripper_state = True
    if add_gripper_state:

        for obs in dataset.entries:
            gripper_box = o3d.geometry.TriangleMesh.create_box(**box_dim)

            gripper_state = obs.proprioception["gripper_joint_positions"][0]

            gripper_state = max(0, min(1.0, gripper_state))
            if gripper_state > 0.9:  # obs.proprioception["gripper_open"]:
                gripper_box.paint_uniform_color([0, 1, 0])
            else:
                gripper_box.paint_uniform_color([1, 0, 0])
            gripper_box.transform(
                np.array(obs.proprioception["gripper_matrix"]).dot(box_offset)
            )

            visualization_data.append(gripper_box)

    entry = dataset.entries[0]

    class DatasetCamera:

        def __init__(self, camera_name, entry):
            self.entry = entry
            self.camera_name = camera_name

        @property
        def extrinsics(self):
            return self.entry.camera_info[f"{self.camera_name}_extrinsics"]

        @property
        def intrinsics(self):
            return self.entry.camera_info[f"{self.camera_name}_intrinsics"]

    pcd = depth_to_pcd(
        entry.camera_data[camera_name],
        DatasetCamera(camera_name, entry),
        apply_extrinsics=True,
    )

    # pcd.colors = o3d.utility.Vector3dVector(rgb)

    visualization_data.append(pcd)

    if show_visualization:

        animate_pcd = True
        if animate_pcd:

            vis = o3d.visualization.Visualizer()
            vis.create_window(visible=True)
            for g in visualization_data:
                vis.add_geometry(g)
                vis.update_geometry(g)

            while True:

                for entry in dataset.entries:

                    pcd_update = depth_to_pcd(
                        entry.camera_data[camera_name],
                        DatasetCamera(camera_name, entry),
                        apply_extrinsics=True,
                    )
                    pcd.points = pcd_update.points
                    pcd.colors = pcd_update.colors
                    vis.update_geometry(pcd)

                    for i in range(10):
                        vis.poll_events()
                        vis.update_renderer()
                        time.sleep(0.01)

            vis.destroy_window()
        else:
            o3d.visualization.draw_geometries(visualization_data)

    else:

        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)
        for g in visualization_data:
            vis.add_geometry(g)
            vis.update_geometry(g)

        vis.poll_events()
        vis.update_renderer()

        view_control = vis.get_view_control()

        # copied from the viewer
        view_pose = {
            "front": [0.012339452449784705, -0.90282996236235535, 0.42982065675584702],
            "lookat": [-0.12694871669500873, 0.11746709358396157, 0.88075116287669197],
            "up": [0.025976937544841906, 0.42999774206857738, 0.90245617097547559],
            "zoom": 0.33999999999999964,
        }

        view_control.set_front(view_pose["front"])
        view_control.set_up(view_pose["up"])
        view_control.set_lookat(view_pose["lookat"])
        view_control.set_zoom(view_pose["zoom"])

        vis.capture_screen_image("/tmp/output.png", do_render=True)
        vis.destroy_window()


if __name__ == "__main__":
    cli()
