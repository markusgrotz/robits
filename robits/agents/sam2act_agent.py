import logging

import torch

from sam2act.eval import load_agent
from robits.agents.base_agent import BaseAgent

logger = logging.getLogger(__name__)


class SAM2Act(BaseAgent):

    def __init__(self, model_path):
        self.device = torch.device("cuda:0")

        self.agent = load_agent(model_path)
        self.agent.load_clip()
        self.agent.eval()
        self.lang_goal = "push the buttons in the following order: red, green, blue"

    def get_action(self, step, observation):

        if not observation:
            logger.error("Nothing todo.")
            return

        with torch.jit.optimized_execution(False):
            act_result = self.agent.act(step, observation, deterministic=True)
        return act_result
