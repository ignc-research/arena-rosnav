import enum
import warnings
import rospy
import numpy as np

from typing import List, Tuple
from std_msgs.msg import Bool
import time
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback


class TrainStageCallbackWP(BaseCallback):
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

    def __init__(
        self,
        namespaces: List[str],
        task_name: str,
        lower_threshold: float,
        upper_threshold: float,
        stage_idx_range: Tuple[int, int],
        init_stage_idx: int = 0,
        verbose=0,
    ):

        super().__init__(verbose=verbose)
        self.namespaces = namespaces
        self.verbose = verbose
        self.activated = task_name == "StagedRandomTask"
        self.upper_threshold = upper_threshold
        self.lower_threshold = lower_threshold
        self.stage_idx_range = stage_idx_range
        self.curr_stage_idx = init_stage_idx

        #DEBUG Only
        self.direction_m = False
        if self.activated:
            self.setup_stage_publishers()

            self._trigger = Bool()
            self._trigger.data = True
        print("TrainStageCallbackWP: TrainStageCallbackWP initialized!")
        print(f"TrainStageCallbackWP: Current Stage is : {self.curr_stage_idx}")

    def setup_stage_publishers(self):
        self._publishers_next = []
        self._publishers_previous = []

        for ns in self.namespaces:
            self._publishers_next.append(
                rospy.Publisher(f"/{ns}/next_stage", Bool, queue_size=1)
            )
            self._publishers_previous.append(
                rospy.Publisher(f"/{ns}/previous_stage", Bool, queue_size=1)
            )

    def _on_step(self, EvalObject: EvalCallback) -> bool:
        assert isinstance(
            EvalObject, EvalCallback
        ), f"InitiateNewTrainStage must be called within EvalCallback"

        if self.activated:

            if EvalObject.n_eval_episodes < 2:
                warnings.warn(
                    "Only %d evaluation episodes considered for threshold monitoring,"
                    "results might not represent agent performance well"
                    % EvalObject.n_eval_episodes
                )
                return 
            if EvalObject.last_success_rate <= self.lower_threshold and self.curr_stage_idx>self.stage_idx_range[0]:
                self.curr_stage_idx -= 1
                for i, pub in enumerate(self._publishers_previous):
                    pub.publish(self._trigger)
                    time.sleep(1)
                    if i == 0:
                        self.log_curr_stage(EvalObject.logger)

            elif EvalObject.last_success_rate >= self.upper_threshold and self.curr_stage_idx<self.stage_idx_range[1]:
                self.curr_stage_idx += 1 
                for i, pub in enumerate(self._publishers_next):
                    pub.publish(self._trigger)
                    time.sleep(1)
                    if i == 0:
                        self.log_curr_stage(EvalObject.logger)

    def log_curr_stage(self, logger):
        print(f"Current Stage is : {self.curr_stage_idx}")
        logger.record("stage_idx", self.curr_stage_idx)


class StopTrainingOnRewardThreshold(BaseCallback):
    """
    Stop the training once a threshold in episodic reward
    has been reached (i.e. when the model is good enough).

    It must be used with the ``EvalCallback``.

    :param reward_threshold:  Minimum expected reward per episode
        to stop training.
    :param verbose:
    """

    def __init__(self, reward_threshold: float, verbose: int = 0):
        super(StopTrainingOnRewardThreshold, self).__init__(verbose=verbose)
        self.reward_threshold = reward_threshold

    def _on_step(self) -> bool:
        assert self.parent is not None, (
            "``StopTrainingOnMinimumReward`` callback must be used "
            "with an ``EvalCallback``"
        )
        # Convert np.bool_ to bool, otherwise callback() is False won't work

        continue_training = not bool(
            self.parent.best_mean_reward >= self.reward_threshold
            and rospy.get_param("/last_stage_reached", True)
        )

        if self.verbose > 0 and not continue_training:
            print(
                f"Stopping training because the mean reward {self.parent.best_mean_reward:.2f} "
                f" is above the threshold {self.reward_threshold}"
            )
        return continue_training
