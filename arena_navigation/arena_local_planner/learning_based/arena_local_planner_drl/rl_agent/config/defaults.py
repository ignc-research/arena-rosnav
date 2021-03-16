from numpy.lib.function_base import _i0_2
from .config import CfgNode as CN
import os
import rospkg

arena_local_planner_drl_root= rospkg.RosPack().get_path('arena_local_planner_drl')


_C = CN()

_C.OUTPUT_DIR_ROOT = os.path.join(arena_local_planner_drl_root,'ouput')





#Robot's setting
_C.ROBOT = CN()
# setting's file for flatland
_C.ROBOT.FLATLAND_DEF =  os.path.join(
                rospkg.RosPack().get_path('simulator_setup'),
                'robot', 'myrobot.model.yaml')
# here defines robot's actions
_C.ROBOT.ACTIONS_DEF = os.path.join(arena_local_planner_drl_root,'configs','robot_actions.yaml')
_C.ROBOT.IS_ACTION_DISCRETE = True

_C.TASK = CN()
_C.TASK.NAME = 'RandomTask'
_C.TASK.CURR_STAGE = 1

_C.ENV = CN()
_C.ENV.NAME='FlatlandEnv'
# in case robot gets stuck and can get out
# currently they are handled in env class
_C.ENV.MAX_STEPS_PER_EPISODE = 525


# rl MODELrithm's parameters which will be passed the curresponding MODELrithm's constructor
# 1. a list of possible parameters _C.MODELn be found here
#   https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html?highlight=ppo#stable_baselines3.ppo.PPO
# 2. a good blog show how to set the parameters properly
#   https://zhuanlan.zhihu.com/p/99901400
#
_C.MODEL = CN()
_C.MODEL.NAME= 'PPO'
_C.MODEL.LEARNING_RATE= 0.0003
_C.MODEL.BATCH_SIZE = 64
# stablebaseline requires that
_C.MODEL.N_STEPS = 1000//_C.MODEL.BATCH_SIZE*_C.MODEL.BATCH_SIZE
_C.MODEL.N_EPOCHS = 3
_C.MODEL.GAMMA = 0.99
_C.MODEL.GAE_LAMBDA = 0.95
_C.MODEL.CLIP_RANGE = 0.2
_C.MODEL.MAX_GRAD_NORM =  0.5
_C.MODEL.ENT_COEF = 0.005
_C.MODEL.VF_COEF = 0.2


_C.REWARD = CN()       
_C.REWARD.RULE_NAME = "rule_01" 
# if none it will be set automatically 
_C.REWARD.SAFE_DIST = None
_C.REWARD.GOAL_RADIUS = 0.25


_C.INPUT = CN()
_C.INPUT.NORM = True
_C.INPUT.NORM_SAVE_FILENAME="norm_env.nenv"

_C.NET_ARCH = CN()
_C.NET_ARCH.VF = [64,64]
_C.NET_ARCH.PI = [64,64]
_C.NET_ARCH.FEATURE_EXTRACTOR = CN()
_C.NET_ARCH.FEATURE_EXTRACTOR.NAME = 'MLP_ARENA2D'
_C.NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM = 128
_C.TRAINING = CN()
_C.TRAINING.N_TIMESTEPS = 4e6


# parameters meaning can be found here
# https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html?highlight=EvalCallback#stable_baselines3.common.callbacks.EvalCallback

_C.EVAL = CN()
_C.EVAL.N_EVAL_EPISODES = 40
_C.EVAL.EVAL_FREQ = 25000

# if None, disable the callback
_C.EVAL.STOP_TRAINING_ON_REWARD= 14








