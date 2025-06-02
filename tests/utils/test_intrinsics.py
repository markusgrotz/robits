#!/usr/bin/env python3

import unittest


import numpy as np


from robits.utils import vision_utils


class TestIntrinsics(unittest.TestCase):

    def test_scale(self):
        current_image_size = (640, 480)
        target_image_size = (320, 240)
        intrinsics = np.array(
            [
                [400.0, 0.0, 320],
                [0.0, 400, 240],
                [0.0, 0.0, 1.0],
            ]
        )

        resized_intrinsics = vision_utils.resize_intrinsics(
            intrinsics, current_image_size, target_image_size
        )

        expected = np.array(
            [
                [200, 0.0, 160],
                [0.0, 200, 120],
                [0.0, 0.0, 1.0],
            ]
        )
        np.testing.assert_almost_equal(expected, resized_intrinsics, decimal=6)


if __name__ == "__main__":
    unittest.main()
