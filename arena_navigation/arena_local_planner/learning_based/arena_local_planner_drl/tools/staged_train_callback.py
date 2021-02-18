import warnings
import rospy
import numpy as np
from typing import List
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
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
    def __init__(self, 
                TaskManagers: List[StagedRandomTask], 
                treshhold_type: str, 
                upper_threshold: float = 0, 
                lower_threshold: float = 0, 
                task_mode: str = "staged", 
                verbose = 0):

        super(InitiateNewTrainStage, self).__init__(verbose = verbose)
        self.TaskManagers = TaskManagers
        self.threshhold_type = treshhold_type

        assert (self.threshhold_type == "rew" or self.threshhold_type == "succ"
        ), "given theshhold type neither 'rew' or 'succ'"
        
        # default values
        if self.threshhold_type == "rew" and upper_threshold == 0:
            self.upper_threshold = 13
            self.lower_threshold = 7
        elif self.threshhold_type == "succ" and upper_threshold == 0:
            self.upper_threshold = 0.85
            self.lower_threshold = 0.6
        else:
            self.upper_threshold = upper_threshold
            self.lower_threshold = lower_threshold

        self.verbose = verbose
        self.activated = bool(task_mode == "staged")

    def _on_step(self, EvalObject: EvalCallback) -> bool:
        assert (isinstance(EvalObject, EvalCallback)
        ), f"InitiateNewTrainStage must be called within EvalCallback"
    

        if self.activated:
            if EvalObject.n_eval_episodes < 20:
                warnings.warn("Only %d evaluation episodes considered for threshold monitoring," 
                    "results might not represent agent performance well" % EvalObject.n_eval_episodes)
            
            if ((self.threshhold_type == "rew" and EvalObject.best_mean_reward <= self.lower_threshold) or
                (self.threshhold_type == "succ" and EvalObject.last_success_rate <= self.lower_threshold)):
                for task_manager in self.TaskManagers:
                    task_manager.previous_stage()
                    
            if ((self.threshhold_type == "rew" and EvalObject.best_mean_reward >= self.upper_threshold) or
                (self.threshhold_type == "succ" and EvalObject.last_success_rate >= self.upper_threshold)):
                for task_manager in self.TaskManagers:
                    task_manager.next_stage()
                EvalObject.best_mean_reward = -np.inf
                EvalObject.last_success_rate = -np.inf
