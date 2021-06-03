import os
import sys
import rospy
import rospkg
import json
import numpy as np
import time
import warnings

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv, VecNormalize
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy

from task_generator.task_generator.tasks import StopReset
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.envs.flatland_gym_env import FlatlandEnv
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.argsparser import parse_run_agent_args
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.train_agent_utils import *

### HYPERPARAMETERS ###
AGENTS = ['AGENT_1_2021_04_02__22_03', 'AGENT_2_2021_03_30__23_10', 'AGENT_3_2021_04_01__08_06', 'AGENT_4_2021_04_02__01_07', 
    'AGENT_5_2021_03_31__18_52', 'AGENT_6_2021_04_04__02_12', 'AGENT_7_2021_04_06__07_00', 'AGENT_8_2021_04_05__22_11', 'AGENT_9_2021_04_04__11_09', 
    'AGENT_10_2021_04_03__16_57', 'AGENT_11_2021_04_14__20_13', 'AGENT_12_2021_04_05__12_08', 'AGENT_13_2021_04_04__18_04', 'AGENT_14_2021_04_07__01_17', 
    'AGENT_15_2021_04_08__01_27', 'AGENT_16_2021_04_09__22_24', 'AGENT_17_2021_04_10__15_36', 'AGENT_18_2021_04_11__13_54', 'AGENT_19_2021_04_12__13_17']
max_steps_per_episode = np.inf
eval_episodes = 1000

def get_paths(args: dict, AGENT: str):
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    PATHS={
        'model': os.path.join(dir, 'agents', AGENT),
        'vecnorm': os.path.join(dir, 'agents', AGENT, 'vec_normalize.pkl'),
        'robot_setting' : os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml'),
        'robot_as' : os.path.join(dir, 'configs', 'default_settings.yaml'),
        'scenario' : os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'scenarios', args.scenario+'.json'),
        'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
        'log': os.path.join(dir, 'evaluation_logs', AGENT)
    }
    if args.log:
        if not os.path.exists(PATHS['log']):
            os.makedirs(PATHS['log'])
    return PATHS

def make_env(with_ns: bool,
             PATHS: dict, 
             PARAMS: dict, 
             log: bool=False):
    """
    Utility function for multiprocessed env
    
    :param params: (dict) hyperparameters of agent to be trained
    :param PATHS: (dict) script relevant paths
    :param log: (bool) to differentiate between train and eval env
    :return: (Callable)
    """
    def _init():
        ns = f"eval_sim" if with_ns else ""

        env = FlatlandEnv(
            ns, PARAMS['reward_fnc'], PARAMS['discrete_action_space'], 
            goal_radius=0.05, max_steps_per_episode=max_steps_per_episode, train_mode=False, task_mode='scenario', PATHS=PATHS, curr_stage=4,
            extended_eval=True)
        if log:
            # eval env
            env = Monitor(
                env, PATHS['log'], False, 
                info_keywords=("collisions", "distance_travelled", "time_safe_dist", "time", "done_reason", "is_success"))
        return env
    return _init

if __name__ == "__main__":
    args, _ = parse_run_agent_args()

    if args.load:
        AGENTS = []
        AGENTS.append(args.load)

    assert len(AGENTS) > 0, "No agent name was given for evaluation"

    ros_params = rospy.get_param_names()
    ns_for_nodes = False if '/single_env' in ros_params else True

    start = time.time()
    while len(AGENTS) != 0:
        AGENT = AGENTS.pop(0)
        print(f"START RUNNING AGENT:    {AGENT}")
        PATHS = get_paths(args, AGENT)

        assert os.path.isfile(
            os.path.join(PATHS['model'], "best_model.zip")), "No model file found in %s" % PATHS['model']
        assert os.path.isfile(
            PATHS['scenario']), "No scenario file named %s" % PATHS['scenario']

        PARAMS = load_hyperparameters_json(PATHS)
        print_hyperparameters(PARAMS)

        env = DummyVecEnv([make_env(ns_for_nodes, PATHS, PARAMS, args.log)])
        if PARAMS['normalize']:
            if not os.path.isfile(PATHS['vecnorm']):
                # without it agent performance will be strongly altered
                warnings.warn(
                    f"Couldn't find VecNormalize pickle for {PATHS['model'].split('/')[-1]}, going to skip this model")
                continue
            
            env = VecNormalize.load(PATHS['vecnorm'], env)

        # load agent
        agent = PPO.load(os.path.join(PATHS['model'], "best_model.zip"), env)

        try:
            evaluate_policy(
                model=agent,
                env=env,
                n_eval_episodes=eval_episodes,
                deterministic=True
            )
        except StopReset:
            pass

    time = round(time.time()-start)
    print(f"Time passed:    {time}s")
    print("EVALUATION DONE!")
    sys.exit()

    # env.reset()
    # first_obs = True

    # # iterate through each scenario max_repeat times
    # while True:
    #     if first_obs:
    #         # send action 'stand still' in order to get first obs
    #         if params['discrete_action_space']:
    #             obs, rewards, dones, info = env.step([6])
    #         else:
    #             obs, rewards, dones, info = env.step([[0.0, 0.0]])
    #         first_obs = False
    #         cum_reward = 0.0

    #     # timer = time.time()
    #     action, _ = agent.predict(obs, deterministic=True)
    #     # print(f"Action predict time: {(time.time()-timer)*2.5} (sim time)")

    #     # clip action
    #     if not params['discrete_action_space']:
    #         action = np.maximum(
    #             np.minimum(agent.action_space.high, action), agent.action_space.low)
        
    #     # apply action
    #     obs, rewards, done, info = env.step(action)

    #     cum_reward += rewards
        
    #     if done:
    #         if args.verbose == '1':
    #             if info[0]['done_reason'] == 0:
    #                 done_reason = "exceeded max steps"
    #             elif info[0]['done_reason'] == 1:
    #                 done_reason = "collision"
    #             else:
    #                 done_reason = "goal reached"
                
    #             print("Episode finished with reward of %f (finish reason: %s)"% (cum_reward, done_reason))
    #         env.reset()
    #         first_obs = True

    #     time.sleep(0.001)
    #     if rospy.is_shutdown():
    #         print('shutdown')
    #         break
