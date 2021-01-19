from stable_baselines3.common.callbacks import BaseCallback
from task_generator.task_generator.tasks import StagedRandomTask

class InitiateNewTrainStage(BaseCallback):
    """
    Introduces new training stage when threshhold reached.
    It must be used with "EvalCallback".

    :param TaskManager (obj, StagedRandomTask): holds task specific methods and stages
    :param TheshholdType (str): checks threshhold for either percentage of successful episodes (succ_per) or mean reward (rew)
    :param StartAt (int): stage to start training with
    :param rew_threshold (int): mean reward threshold to trigger new stage
    :param succ_per_threshold (float): threshold percentage of succesful episodes to trigger new stage
    :param task_mode (str): training task mode, if not 'staged' callback won't be called
    :param verbose:
    """
    def __init__(self, TaskManager: StagedRandomTask, TreshholdType: str, rew_threshold: float = 10, succ_per_threshold: float = 0.8, task_mode: str = "staged", verbose = 0):
        super(InitiateNewTrainStage, self).__init__(verbose = verbose)
        self.task_manager = TaskManager
        self.threshhold_type = TreshholdType
        self.rew_threshold = rew_threshold
        self.succ_per_threshold = succ_per_threshold
        self.verbose = verbose
        self.activated = bool(task_mode == "staged")

    def _on_step(self) -> bool:
        assert self.parent is not None, "'InitiateNewTrainStage' callback must be used " "with an 'EvalCallback'"
        
        if self.activated:
            if self.parent.n_eval_episodes < 10:
                raise Warning("Only %d evaluation episodes considered for threshold monitoring" % self.parent.n_eval_episodes)

            #TODO: access eval episodes to calculate percentage (consider editing EvalCallBack of stablebaselines3)
            if (self.threshhold_type == "rew" and self.parent.best_mean_reward > self.rew_threshold) or (self.threshhold_type == "succ_per" and 99 > self.succ_per_threshold):
                self.task_manager.next_stage()
                self.parent.best_mean_reward = 0

        return True