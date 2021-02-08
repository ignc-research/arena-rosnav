import warnings
import rospy
import numpy as np
from typing import List
from stable_baselines3.common.callbacks import BaseCallback
from task_generator.task_generator.tasks import StagedRandomTask

class InitiateNewTrainStage(BaseCallback):
    """
    Introduces new training stage when threshhold reached.
    It must be used with "EvalCallback".

    :param TaskManagers (list, StagedRandomTask): holds task managers with specific methods and stages
    :param treshhold_type (str): checks threshhold for either percentage of successful episodes (succ) or mean reward (rew)
    :param StartAt (int): stage to start training with
    :param rew_threshold (int): mean reward threshold to trigger new stage
    :param succ_rate_threshold (float): threshold percentage of succesful episodes to trigger new stage
    :param task_mode (str): training task mode, if not 'staged' callback won't be called
    :param verbose:
    """
    def __init__(self, TaskManagers: List[StagedRandomTask], treshhold_type: str, rew_threshold: float = 10, succ_rate_threshold: float = 0.8, task_mode: str = "staged", verbose = 0):
        super(InitiateNewTrainStage, self).__init__(verbose = verbose)
        self.TaskManagers = TaskManagers
        self.threshhold_type = treshhold_type
        assert self.threshhold_type == "rew" or self.threshhold_type == "succ", "given theshhold type neither 'rew' or 'succ'"
        self.rew_threshold = rew_threshold
        self.succ_rate_threshold = succ_rate_threshold
        self.verbose = verbose
        self.activated = bool(task_mode == "staged")

    def _on_step(self) -> bool:
        assert self.parent is not None, "'InitiateNewTrainStage' callback must be used " "with an 'EvalCallback'"
        
        if self.activated:
            if self.parent.n_eval_episodes < 10:
                warnings.warn("Only %d evaluation episodes considered for threshold monitoring" % self.parent.n_eval_episodes)

            if (self.threshhold_type == "rew" and self.parent.best_mean_reward >= self.rew_threshold) or (self.threshhold_type == "succ" and self.parent.last_success_rate >= self.succ_rate_threshold):
                for task_manager in self.TaskManagers:
                    task_manager.next_stage()
                self.parent.best_mean_reward = -np.inf
                self.parent.last_success_rate = -np.inf
        return True