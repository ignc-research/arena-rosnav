import warnings
import rospy
import numpy as np
import time

from typing import List
from std_msgs.msg import Bool
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from task_generator.task_generator.tasks import StagedRandomTask

class InitiateNewTrainStage(BaseCallback):
    """
    Introduces new training stage when threshhold reached.
    It must be used with "EvalCallback".

    :param treshhold_type (str): checks threshhold for either percentage of successful episodes (succ) or mean reward (rew)
    :param rew_threshold (int): mean reward threshold to trigger new stage
    :param succ_rate_threshold (float): threshold percentage of succesful episodes to trigger new stage
    :param task_mode (str): training task mode, if not 'staged' callback won't be called
    :param verbose:
    """
    def __init__(self, 
                n_envs: int = 1, 
                treshhold_type: str = "succ", 
                upper_threshold: float = 0, 
                lower_threshold: float = 0, 
                task_mode: str = "staged", 
                verbose = 0):

        super(InitiateNewTrainStage, self).__init__(verbose = verbose)
        self.n_envs = n_envs
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

        assert (self.upper_threshold > self.lower_threshold
        ), "upper threshold has to be bigger than lower threshold"
        assert (self.upper_threshold >= 0 and self.lower_threshold >= 0
        ), "upper/lower threshold have to be positive numbers"
        if self.threshhold_type == "succ":
            assert (self.upper_threshold <= 1 and self.lower_threshold >= 0
            ), "succ thresholds have to be between [1.0, 0.0]"

        self.verbose = verbose
        self.activated = bool(task_mode == "staged")

        if self.activated:
            rospy.set_param("/last_stage_reached", False)
            self._instantiate_publishers()

            self._trigger = Bool()
            self._trigger.data = True

    def _instantiate_publishers(self):
        self._publishers_next = []
        self._publishers_previous = []

        self._publishers_next.append(
            rospy.Publisher(f"/eval_sim/next_stage", Bool, queue_size=1))
        self._publishers_previous.append(
            rospy.Publisher(f"/eval_sim/previous_stage", Bool, queue_size=1))

        for env_num in range(self.n_envs):
            self._publishers_next.append(
                rospy.Publisher(f"/sim_{env_num+1}/next_stage", Bool, queue_size=1))
            self._publishers_previous.append(
                rospy.Publisher(f"/sim_{env_num+1}/previous_stage", Bool, queue_size=1))

    def _on_step(self, EvalObject: EvalCallback) -> bool:
        assert (isinstance(EvalObject, EvalCallback)
        ), f"InitiateNewTrainStage must be called within EvalCallback"

        if self.activated:
            if EvalObject.n_eval_episodes < 20:
                warnings.warn("Only %d evaluation episodes considered for threshold monitoring," 
                    "results might not represent agent performance well" % EvalObject.n_eval_episodes)
            
            if ((self.threshhold_type == "rew" and EvalObject.best_mean_reward <= self.lower_threshold) or
                (self.threshhold_type == "succ" and EvalObject.last_success_rate <= self.lower_threshold)):                
                for i, pub in enumerate(self._publishers_previous):
                    pub.publish(self._trigger)
                    if i == 0:
                       self.log_curr_stage(EvalObject.logger)
                    
            if ((self.threshhold_type == "rew" and EvalObject.best_mean_reward >= self.upper_threshold) or
                (self.threshhold_type == "succ" and EvalObject.last_success_rate >= self.upper_threshold)):
                if not rospy.get_param("/last_stage_reached"):
                    EvalObject.best_mean_reward = -np.inf
                    EvalObject.last_success_rate = -np.inf

                for i, pub in enumerate(self._publishers_next):
                    pub.publish(self._trigger)
                    if i == 0:
                        self.log_curr_stage(EvalObject.logger)

    def log_curr_stage(self, logger):
        time.sleep(1)
        curr_stage = rospy.get_param("/curr_stage", -1)
        logger.record("train_stage/stage_idx", curr_stage)
