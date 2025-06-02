from abc import ABC
from abc import abstractmethod

import logging

import numpy as np


logger = logging.getLogger(__name__)


class BaseAgent(ABC):
    """ """

    agent_name: str

    lang_goal: str

    def prepare_observation(self, obs, i, episode_length):

        from clip import tokenize
        import torch

        logger.info("Using language goal %s", self.lang_goal)

        obs["lang_goal_tokens"] = tokenize([self.lang_goal])[0].numpy()

        elapsed_time = (1.0 - (i / float(episode_length - 1))) * 2.0 - 1.0
        gripper_joint_positions = obs["gripper_joint_positions"][
            0:1
        ]  # gripper is normalized here
        logger.info("timestamp: %.2f", elapsed_time)

        obs["ignore_collisions"] = np.array([0])
        obs["low_dim_state"] = np.concatenate(
            [
                gripper_joint_positions,
                gripper_joint_positions,
                gripper_joint_positions,
                elapsed_time,
            ],
            axis=None,
        )

        for k, v in obs.items():
            if v is None:
                logger.info("No values for key %s", k)
                # obs[k] = torch.tensor([np.zeros(4)], device=self.device).unsqueeze(0)
                continue

            if isinstance(v, np.ndarray):
                v = v.astype(np.float32)
                logger.debug("Item %s has shape %s", k, v.shape)
            else:
                logger.debug("Key %s is not a numpy array", k)

            obs[k] = torch.tensor([v], device=self.device).unsqueeze(0)

        return obs

    @abstractmethod
    def get_action(self, step, observation):
        pass
