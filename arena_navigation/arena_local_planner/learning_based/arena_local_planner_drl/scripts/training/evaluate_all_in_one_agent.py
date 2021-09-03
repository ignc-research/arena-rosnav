import json
import os
import random
import time

import numpy as np
import rospkg
import torch
from rl_agent.envs.all_in_one_flatland_gym_env import AllInOneEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from tools.all_in_one_utils import Evaluator
from tools.train_agent_utils import check_hyperparam_format, print_hyperparameters

base_Agent = 'indoor_teb_drl4_rule06_policy1'
primitive_agents = ['rlca_only', 'teb_only', 'drl_only', 'mpc_only', 'global_path_following_only',
                    'teb_large_min_dist_only']
simple_all_in_one_switches = ['simple_all_in_one', 'random']

AGENTS = ['simple_all_in_one', 'teb_only', 'drl_only']
eval_episodes = 100
seed = random.randint(1, 1000)
map_config = "indoor.json"


def get_paths(AGENT: str, primitive_agent=False, is_random_agent=False):
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    if primitive_agent:
        paths = {
            'all_in_one_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'agent_parameters',
                                                  AGENT + '.json'),
            'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml'),
            'robot_as': os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
            'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
            'drl_agents': os.path.join(dir, 'agents'),
            'hyperparams': os.path.join(dir, 'configs', 'hyperparameters', 'all_in_one_default.json'),
            'map_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters', map_config)
        }
    else:
        paths = {
            'hyperparams': os.path.join(dir, 'agents', AGENT, 'hyperparameters.json'),
            'model': os.path.join(dir, 'agents', AGENT),
            'all_in_one_parameters': os.path.join(dir, 'agents', AGENT, 'all_in_one_parameters.json'),
            'vecnorm': os.path.join(dir, 'agents', AGENT, 'vec_normalize.pkl'),
            'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml'),
            'robot_as': os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
            'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
            'drl_agents': os.path.join(dir, 'agents'),
            'map_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters', map_config)
        }
    if is_random_agent:
        AGENT = "random"
    eval_numb = 0
    paths['log'] = os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'evaluation_logs',
                                AGENT + "_" + str(eval_numb))
    while os.path.exists(paths['log']):
        eval_numb += 1
        paths['log'] = os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'evaluation_logs',
                                    AGENT + "_" + str(eval_numb))
    os.makedirs(paths['log'])

    return paths


def make_env(paths: dict,
             params: dict):
    """
    Utility function for multiprocessed env

    :param params: (dict) hyperparameters of agent to be trained
    :param paths: (dict) script relevant paths
    :param log: (bool) to differentiate between train and eval env
    :return: (Callable)
    """

    def _init():
        return AllInOneEnv("eval_sim", paths['robot_setting'], paths['robot_as'], params['reward_fnc'],
                           goal_radius=params['goal_radius'], debug=True,
                           paths=paths, train_mode=False, evaluation=True,
                           max_steps_per_episode=params['eval_max_steps_per_episode'], seed=seed,
                           extended_eval=True, evaluation_episodes=eval_episodes)

    return _init


def random_agent(numb_models: int):
    def _get_random_action(_):
        return random.randint(0, numb_models - 1)

    return _get_random_action


def random_action_probs(numb_models: int):
    def _get_random_action_probs(_, __):
        return numb_models * np.log([1. / numb_models])

    return _get_random_action_probs


def simple_all_in_one(obs):
    _RS = 4  # robot state size
    _L = 360  # laser scan size
    switch_distance = 1.1
    laser_scan_dynamic = obs[_L:-_RS]
    min_dist = np.min(laser_scan_dynamic)
    if min_dist <= switch_distance:
        return 0
    else:
        return 1


def simple_all_in_one_action_probs(obs, _):
    _RS = 4  # robot state size
    _L = 360  # laser scan size
    switch_distance = 1.1
    laser_scan_dynamic = torch.unsqueeze(obs[:, _L:-_RS], 1).cpu().numpy()
    if np.min(laser_scan_dynamic) <= switch_distance:
        return [np.log(1), np.log(0)]
    else:
        return [np.log(0), np.log(1)]


def load_hyperparameters_json(PATHS):
    doc_location = os.path.join(PATHS.get('hyperparams'))
    if os.path.isfile(doc_location):
        with open(doc_location, "r") as file:
            hyperparams = json.load(file)
        check_hyperparam_format(loaded_hyperparams=hyperparams, PATHS=PATHS)
        return hyperparams
    else:
        raise FileNotFoundError("Found no file %s" % (os.path.join(PATHS.get('hyperparams'))))


if __name__ == "__main__":

    start = time.time()
    while len(AGENTS) != 0:
        AGENT = AGENTS.pop(0)
        print(f"START RUNNING AGENT:    {AGENT}")

        if AGENT == "random":
            paths = get_paths(base_Agent, is_random_agent=True)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params)])
            policy = random_agent(env.env_method("get_number_models")[0])
            action_probs = random_action_probs(env.env_method("get_number_models")[0])
            env = VecNormalize(env, norm_obs=False, norm_reward=False)
        elif AGENT in primitive_agents:
            paths = get_paths(AGENT, primitive_agent=True)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params)])
            env = VecNormalize(env, norm_obs=False, norm_reward=False)


            def policy(_):
                return 0


            def action_probs(_, __):
                return [np.log(1.0)]

        elif AGENT == "simple_all_in_one":
            paths = get_paths(AGENT, primitive_agent=True)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params)])
            env = VecNormalize(env, norm_obs=False, norm_reward=False)
            policy = simple_all_in_one
            action_probs = simple_all_in_one_action_probs

        else:
            paths = get_paths(AGENT)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params)])
            assert os.path.isfile(
                os.path.join(paths['model'], "best_model.zip")), "No model file found in %s" % paths['model']
            # load vec norm
            env = VecNormalize.load(paths['vecnorm'], env)
            # load agent
            agent = PPO.load(os.path.join(paths['model'], "best_model.zip"), env)


            def policy(x):
                return agent.predict(x, deterministic=True)[0]


            def action_probs(x, y):
                return agent.policy.evaluate_actions(x, y)[1]

        evaluator = Evaluator()
        evaluator.evaluate_policy_manually(policy, action_probs, env, eval_episodes, paths['log'], params['gamma'],
                                           paths['all_in_one_parameters'])

        env.close()
        print("Evaluation of agent " + AGENT + " completed!")

    time = round(time.time() - start)
    print(f"Time passed:    {time}s")
    print("EVALUATION DONE!")
