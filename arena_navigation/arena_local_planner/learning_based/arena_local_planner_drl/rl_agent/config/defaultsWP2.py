from .config import CfgNode as CN
import os
import rospkg

arena_local_planner_drl_root = rospkg.RosPack().get_path("arena_local_planner_drl")


_C = CN()

_C.OUTPUT_DIR_ROOT = os.path.join(arena_local_planner_drl_root, "output")
# Robot's setting
_C.ROBOT = CN()
# setting's file for flatland
_C.ROBOT.FLATLAND_DEF = os.path.join(
    rospkg.RosPack().get_path("simulator_setup"), "robot", "myrobot.model.yaml"
)
# here defines robot's actions
_C.ROBOT.ACTIONS_DEF = os.path.join(
    arena_local_planner_drl_root, "configs", "robot_actions.yaml"
)
_C.ROBOT.IS_ACTION_DISCRETE = True

_C.TASK = CN()
# _C.TASK.NAME = 'RandomTask'
_C.TASK.NAME = "StagedRandomTask"


_C.ENV = CN()
_C.ENV.NAME = "WPEnvMapFrame2"
# _C.ENV.NAME='WPEnv'
# in case robot gets stuck and can get out
# currently they are handled in env class


# rl MODELrithm's parameters which will be passed the curresponding MODELrithm's constructor
# 1. a list of possible parameters _C.MODELn be found here
#   https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html?highlight=ppo#stable_baselines3.ppo.PPO
# 2. a good blog show how to set the parameters properly
#   https://zhuanlan.zhihu.com/p/99901400
#


_C.REWARD = CN()

# _C.REWARD.ROBOT_OBSTACLE_MIN_DIST = 0.1
_C.REWARD.RULE_NAME = "RewardCalculatorWP2_RULE00"
_C.REWARD.REWARD_ON_COLLISION = -10
_C.REWARD.REWARD_ON_GOAL_REACHED = 3
_C.REWARD.REWARD_ON_TIME_COST = 0.01
_C.REWARD.REWARD_ON_TIMEOUT = -0.3
_C.REWARD.REWARD_ON_PROGRESS_BASE = 0.01
_C.REWARD.NO_PROGRESS_REWARD = True
_C.REWARD.NO_TIMEOUT_REWARD = False


_C.WAYPOINT_GENERATOR = CN()

_C.WAYPOINT_GENERATOR.ACTIONS_DEF = os.path.join(
    arena_local_planner_drl_root, "configs", "waypoint_generator_actions2.yaml"
)
# rlca can only take 0.5, remember robot's radius is 0.3
_C.WAYPOINT_GENERATOR.ROBOT_WAYPOINT_MIN_DIST = 0.5
# make sure this value bigger than the linear value defined in the configs/waypoint_generator_actions*.yaml
_C.WAYPOINT_GENERATOR.GOAL_RADIUS = 0.75
_C.WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE = False
# unit m/s should be set with the consideration of the lookahead distance defined in the launch file and robot's veloctiy
# e.g. lookahead_distance = 2m robot_velocity = 0.3m/s   average_reach_time = 2/0.3 == 6.6 s
_C.WAYPOINT_GENERATOR.STEP_TIMEOUT = 25


_C.INPUT = CN()
# where to normalize the inpute or not
_C.INPUT.NORM = True
_C.INPUT.NORM_SAVE_FILENAME = "norm_env.nenv"

_C.NET_ARCH = CN()
_C.NET_ARCH.VF = [32,32]
_C.NET_ARCH.PI = [32,32]
_C.NET_ARCH.FEATURE_EXTRACTOR = CN()
# _C.NET_ARCH.FEATURE_EXTRACTOR.NAME = "CNN_Laser_MLP_Goal_Dyn_Obs"
# _C.NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM = 32 + 32 + 6
#DEBUG
# _C.NET_ARCH.FEATURE_EXTRACTOR.NAME = "Pure_Dyn_Obs"
# _C.NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM = 32 + 6
_C.NET_ARCH.FEATURE_EXTRACTOR.NAME = "Pure_Static_Obs"
_C.NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM = 32 + 6


_C.TRAINING = CN()
_C.TRAINING.N_TIMESTEPS = int(4e6)
_C.TRAINING.ROBOT_START_POS_GOAL_POS_MIN_DIST = 4
# MAX_STEPS_PER_EPISODE*lookahead_dist == traj dist
_C.TRAINING.MAX_STEPS_PER_EPISODE = 300

# parameters meaning can be found here
# https://stable-baselines3.readthedocs.io/en/master/guide/callbacks.html?highlight=EvalCallback#stable_baselines3.common.callbacks.EvalCallback
_C.EVAL = CN()
# DEBUG
_C.EVAL.N_EVAL_EPISODES = 30
_C.EVAL.EVAL_FREQ = int(4e4)
# if None, disable the callback
_C.EVAL.STOP_TRAINING_ON_REWARD = None
_C.EVAL.STOP_TRAINING_ON_REWARD_THRESHOLD = 15
# only be active when the task is StagedRandomTask
_C.EVAL.CURRICULUM = CN()
# "rew" means "mean reward", "succ" means "success rate of episode "
_C.EVAL.CURRICULUM.THRESHOLD_RANGE = [0.7, 0.85]
_C.EVAL.CURRICULUM.STAGE_STATIC_OBSTACLE = [0, 0, 0]
_C.EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE = [0, 3, 6]
# _C.EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE = [16, 3, 6]
_C.EVAL.CURRICULUM.INIT_STAGE_IDX = 0


_C.DEPLOY = CN()
_C.DEPLOY.SCENERIOS_JSON_PATH = "/home/joe/ssd/projects/arena-rosnav-ws/src/arena-rosnav/simulator_setup/scenarios/eval/obstacle_map1_obs20.json"


_C.MODEL = CN()
_C.MODEL.NAME = "PPO"
_C.MODEL.LEARNING_RATE = 0.0003
# mini batch size
_C.MODEL.BATCH_SIZE = 16
# stablebaseline requires that
_C.MODEL.N_STEPS = 4 ** 10 // _C.MODEL.BATCH_SIZE * _C.MODEL.BATCH_SIZE
_C.MODEL.N_EPOCHS = 7
_C.MODEL.GAMMA = 0.9
_C.MODEL.GAE_LAMBDA = 0.95
_C.MODEL.CLIP_RANGE = 0.22
# this string will be treated as a callable object. we use this to schedule the the clip range
# _C.MODEL.CLIP_RANGE = f"res = 1-int(n_step/{_C.TRAINING.N_TIMESTEPS}*10)/10"
_C.MODEL.MAX_GRAD_NORM = 0.5
_C.MODEL.ENT_COEF = 0.01
_C.MODEL.VF_COEF = 0.5

