import unittest
from unittest.mock import MagicMock
import numpy as np

from robits.core.abc.robot import DummyRobot
from robits.core.abc.robot import Perception


class TestDummyRobot(unittest.TestCase):
    def setUp(self):
        # Create dummy camera mock
        self.mock_camera = MagicMock()
        self.mock_camera.camera_name = "test"
        self.mock_camera.get_camera_data.return_value = (
            MagicMock(depth_image=np.ones((2, 2)), rgb_image=np.ones((2, 2, 3))),
            {},
        )
        self.mock_camera.intrinsics = np.identity(3)
        self.mock_camera.extrinsics = np.identity(4)
        self.mock_camera.is_wrist_camera.return_value = False

        self.gripper = MagicMock()
        self.robot = DummyRobot(gripper=self.gripper, cameras=[self.mock_camera])

    def test_get_vision_data(self):
        perception = Perception(cameras=[self.mock_camera])
        data = perception.get_vision_data()

        self.assertIn("test_rgb", data)
        self.assertIn("test_depth", data)
        self.assertEqual(data["test_rgb"].shape, (2, 2, 3))
        self.assertEqual(data["test_depth"].shape, (2, 2))

    def test_dummy_robot_obs(self):
        obs = self.robot.get_obs()
        self.assertIn("test_rgb", obs)
        self.assertIn("test_depth", obs)

    def test_dummy_robot_eef_pose(self):
        pos, quat = self.robot.eef_pose
        np.testing.assert_array_equal(pos, np.zeros(3))
        np.testing.assert_array_equal(quat, np.array([0, 0, 0, 1]))

    def test_dummy_robot_eef_matrix(self):
        matrix = self.robot.eef_matrix
        np.testing.assert_array_equal(matrix, np.identity(4))

    def test_dummy_robot_name(self):
        self.assertEqual(self.robot.robot_name, "dummy")


if __name__ == "__main__":
    unittest.main()
