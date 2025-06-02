import unittest

import numpy as np

from robits.core.data_model.action import CartesianAction
from robits.core.data_model.action import BimanualAction


class ActionTest(unittest.TestCase):

    def test_rot_matrix(self):
        action = CartesianAction(
            position=np.zeros(3), quaternion=np.array([0, 0, 0, 1]), hand_open=False
        )
        expected = np.identity(3)
        self.assertTrue(np.allclose(expected, action.rot_matrix))

    def test_to_matrix(self):
        action = CartesianAction(
            position=np.zeros(3), quaternion=np.array([0, 0, 0, 1]), hand_open=False
        )
        expected = np.identity(4)
        actual = action.to_matrix()
        self.assertTrue(np.allclose(expected, actual))

    def test_from_matrix(self):
        action = CartesianAction.from_matrix(np.identity(4))

        expected_position = np.zeros(3)
        self.assertTrue(np.allclose(expected_position, action.position))

        expected_quaternion = np.array([0, 0, 0, 1])
        self.assertTrue(np.allclose(expected_quaternion, action.quaternion))


class TestCartesianAction(unittest.TestCase):

    def setUp(self):
        self.position = np.array([0.5, 0.0, 0.2])
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.hand_open = True
        self.action = CartesianAction(self.position, self.quaternion, self.hand_open)

    def test_to_matrix_and_from_matrix(self):
        matrix = self.action.to_matrix()
        reconstructed = CartesianAction.from_matrix(matrix, hand_open=True)
        np.testing.assert_array_almost_equal(reconstructed.position, self.position)
        np.testing.assert_array_almost_equal(reconstructed.quaternion, self.quaternion)
        self.assertEqual(reconstructed.hand_open, self.hand_open)

    def test_to_numpy_and_from_numpy(self):
        arr = self.action.to_numpy()
        reconstructed = CartesianAction.from_numpy(arr)
        np.testing.assert_array_almost_equal(reconstructed.position, self.position)
        np.testing.assert_array_almost_equal(reconstructed.quaternion, self.quaternion)
        self.assertEqual(reconstructed.hand_open, self.hand_open)

    def test_parse(self):
        arr = np.concatenate([self.position, self.quaternion, [1.0]])
        parsed = CartesianAction.parse(arr)
        np.testing.assert_array_almost_equal(parsed.position, self.position)
        np.testing.assert_array_almost_equal(parsed.quaternion, self.quaternion)
        self.assertTrue(parsed.hand_open)

    def test_quaternion_and_position_as_tuple(self):
        self.assertEqual(self.action.position_as_tuple, tuple(self.position))
        self.assertEqual(self.action.quaternion_as_tuple, tuple(self.quaternion))

    def test_invalid_numpy_length(self):
        with self.assertRaises(ValueError):
            CartesianAction.from_numpy(np.zeros(5))


class TestBimanualAction(unittest.TestCase):

    def setUp(self):
        self.position = np.array([0.5, 0.0, 0.2])
        self.quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.right_action = CartesianAction(self.position, self.quaternion, True)
        self.left_action = CartesianAction(-self.position, self.quaternion, False)
        self.bimanual_action = BimanualAction(self.right_action, self.left_action)

    def test_to_numpy_and_from_numpy(self):
        arr = self.bimanual_action.to_numpy()
        reconstructed = BimanualAction.from_numpy(arr)
        np.testing.assert_array_almost_equal(
            reconstructed.right_action.position, self.right_action.position
        )
        np.testing.assert_array_almost_equal(
            reconstructed.left_action.position, self.left_action.position
        )
        self.assertTrue(reconstructed.right_action.hand_open)
        self.assertFalse(reconstructed.left_action.hand_open)

    def test_invalid_length(self):
        with self.assertRaises(ValueError):
            BimanualAction.from_numpy(np.zeros(13))
