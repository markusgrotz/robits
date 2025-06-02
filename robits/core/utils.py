from typing import Callable

import time
import json

import importlib
import logging

from functools import wraps

import numpy as np

logger = logging.getLogger(__name__)


def check_bounds():
    def decorator(func: Callable):
        @wraps(func)
        def wrapper(self, action, **kwargs):

            # min_bounds = np.array([0.2, -0.4, 0.41])
            min_bounds = np.array([0.2, -0.4, 0.30])
            max_bounds = np.array([0.62, 0.4, 0.70])

            if np.any(action.position < min_bounds):
                logger.warning(
                    f"Action position {action.position} out of bounds (min: {min_bounds})"
                )
                return False
            if np.any(action.position > max_bounds):
                logger.warning(
                    f"Action position {action.position} out of bounds (max: {max_bounds})"
                )
                return False
            else:
                return func(self, action, **kwargs)

        return wrapper

    return decorator


class CustomDecoder(json.JSONDecoder):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(object_hook=self._object_hook_fn, *args, **kwargs)

    def _object_hook_fn(self, obj):
        if isinstance(obj, dict) and (class_path := obj.get("class_path", None)):
            try:
                module_name, class_name = class_path.rsplit(".", 1)
                module = importlib.import_module(module_name)
                cls = getattr(module, class_name)
                obj.pop("class_path")
                return cls(**obj)
            except (ImportError, AttributeError, TypeError) as e:
                raise ValueError(f"Could not instantiate class '{class_path}': {e}")
        return obj


class NumpyJSONEncoder(json.JSONEncoder):
    """Special json encoder for numpy types. Converts numpy array to a list."""

    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        # elif isinstance(obj, np.bool):
        #    return super().default(obj)
        return json.JSONEncoder.default(self, obj)


class MiscJSONEncoder(NumpyJSONEncoder):

    def default(self, obj):
        if hasattr(obj, "to_dict") and callable(obj.to_dict):
            return obj.to_dict()
        return super().default(obj)


class FrequencyTimer:
    """keeps a loop frequency"""

    def __init__(self, frequency):
        self.frequency = frequency
        self.period = 1.0 / frequency
        self.last_cycle = None

    def reset(self):
        """
        Resests the last cycle
        """
        self.last_cycle = time.perf_counter()

    def wait_for_cycle(self):
        """
        Waits for the remaining period time.
        First  cycle is skipped if ..func::reset hasn't been called.
        """
        if not self.last_cycle:
            self.last_cycle = time.perf_counter()
            return

        elapsed_time = time.perf_counter() - self.last_cycle
        time_remaining = self.period - elapsed_time

        if time_remaining > 0.001:
            time.sleep(time_remaining)
        self.last_cycle = time.perf_counter()
