import unittest
from unittest.mock import patch


import numpy as np
import json

from robits.core.utils import FrequencyTimer
from robits.core.utils import NumpyJSONEncoder


class NumpyJSONEncoderTest(unittest.TestCase):

    def test_encode(self):
        data = {
            "array": np.arange(3),
            "ones": np.ones((3, 3), dtype=np.uint8),
            "misc": [np.zeros(3), np.zeros(3)],
        }
        expected = '{"array": [0, 1, 2], "ones": [[1, 1, 1], [1, 1, 1], [1, 1, 1]], "misc": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]}'
        actual = json.dumps(data, cls=NumpyJSONEncoder)
        self.assertEqual(expected, actual)


class FrequencyTimerTest(unittest.TestCase):
    def test_period(self):
        timer = FrequencyTimer(1)
        self.assertEqual(1, timer.period)

        timer = FrequencyTimer(10)
        self.assertEqual(0.1, timer.period)

        timer = FrequencyTimer(30)
        self.assertEqual(1 / 30.0, timer.period)

    def test_reset(self):

        timer = FrequencyTimer(frequency=100)

        with patch("time.perf_counter") as mock_perf_counter, patch(
            "time.sleep"
        ) as mock_sleep:

            mock_perf_counter.return_value = 123.0

            timer.reset()
            mock_perf_counter.assert_called_once()
            mock_sleep.assert_not_called()
            self.assertEqual(timer.last_cycle, 123.0)

    def test_cycle(self):
        timer = FrequencyTimer(frequency=2)

        with patch("time.perf_counter") as mock_perf_counter, patch(
            "time.sleep"
        ) as mock_sleep:

            mock_perf_counter.side_effect = [100.0, 100.3, 100.5, 100.9, 101.1]

            # reset
            timer.reset()
            self.assertEqual(timer.last_cycle, 100.0)
            mock_perf_counter.assert_called_once()

            # wait for the first loop. This should trigger a sleep
            timer.wait_for_cycle()
            self.assertEqual(mock_perf_counter.call_count, 3)
            mock_sleep.assert_called_once()
            actual_sleep_duration = mock_sleep.call_args[0][0]
            self.assertAlmostEqual(0.2, actual_sleep_duration)
            self.assertEqual(timer.last_cycle, 100.5)

            # second loop
            timer.wait_for_cycle()
            self.assertEqual(mock_perf_counter.call_count, 5)
            self.assertEqual(mock_sleep.call_count, 2)
            actual_sleep_duration = mock_sleep.call_args_list[1][0][0]
            self.assertAlmostEqual(0.1, actual_sleep_duration)
            self.assertEqual(timer.last_cycle, 101.1)


if __name__ == "__main__":
    unittest.main()
