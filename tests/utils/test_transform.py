import unittest

import numpy as np

from scipy.spatial.transform import Rotation as R


from robits.utils.transform_utils import transform_pose


class TestTransform(unittest.TestCase):
    def test_identity(self):
        position = np.arange(3)
        quaternion = R.from_euler("xyz", [0, 0, 90], degrees=True).as_quat()

        transform = np.identity(4)

        actual_position, actual_quaternion = transform_pose(
            transform, position, quaternion
        )

        np.testing.assert_array_almost_equal(position, actual_position)
        np.testing.assert_array_almost_equal(quaternion, actual_quaternion)

    def test_translation(self):
        position = np.zeros(3)
        quaternion = R.from_euler("xyz", [90, 0, 0], degrees=True).as_quat()

        transform = np.identity(4)
        transform[:3, 3] = np.ones(3)

        actual_position, actual_quaternion = transform_pose(
            transform, position, quaternion
        )

        expected_position = np.ones(3)

        np.testing.assert_array_almost_equal(expected_position, actual_position)
        np.testing.assert_array_almost_equal(quaternion, actual_quaternion)

    def test_rotation(self):
        position = np.arange(3)
        quaternion = R.from_euler("xyz", [0, 0, 0], degrees=True).as_quat()

        transform = np.identity(4)
        transform[:3, :3] = R.from_euler("xyz", [0, 0, 180], degrees=True).as_matrix()

        actual_position, actual_quaternion = transform_pose(
            transform, position, quaternion
        )

        expected_quaternion = R.from_euler("xyz", [0, 0, 180], degrees=True).as_quat()
        expected_position = np.array([0, -1, 2])

        np.testing.assert_array_almost_equal(expected_position, actual_position)
        np.testing.assert_array_almost_equal(expected_quaternion, actual_quaternion)

    def test_example(self):
        expected_position = position = np.array([0.123, 0.456, 0.789])
        expected_quaternion = quaternion = R.from_euler(
            "xyz", [30, 30, 30], degrees=True
        ).as_quat()

        transform = np.identity(4)
        transform[:3, :3] = R.from_euler("xyz", [0, 0, 10], degrees=True).as_matrix()

        position, quaternion = transform_pose(transform, position, quaternion)
        np.testing.assert_array_almost_equal(
            [30, 30, 40], R.from_quat(quaternion).as_euler("xyz", degrees=True)
        )

        transform[:3, :3] = R.from_euler("xyz", [0, 0, -10], degrees=True).as_matrix()
        actual_position, actual_quaternion = transform_pose(
            transform, position, quaternion
        )

        np.testing.assert_array_almost_equal(expected_position, actual_position)
        np.testing.assert_array_almost_equal(expected_quaternion, actual_quaternion)


if __name__ == "__main__":
    unittest.main()
