import csv
import json
import os
import random

import numpy as np
import rospkg
import torch
import yaml
from rl_agent.envs.all_in_one_flatland_gym_env import AllInOneEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from tools.all_in_one_utils import Evaluator
from tools.train_agent_utils import check_hyperparam_format, print_hyperparameters

import rospy

base_Agent1 = 'mixed_teb_drl4_rule06_policy2'  # 0.87
base_Agent2 = 'mixed_teb_drl4_rule07_policy3'
base_Agent3 = 'teb_drl4_rule07_policy2'  # 0.86
base_Agent4 = "2xteb_drl4_rule07_policy2"  # 88
base_agent_13 = "teb_drl4_rule07_nn13_16+d_mixed_5M_2"  # 0.93

base_Agent11 = "teb_drl4_rule06_nn7_fx3_10M"  # 86
base_Agent12 = "tebx2_drl4_rule06_nn7_fx3_10obst_20M"  # 81
base_agent_14 = "teb_drl4_rule07_nn21_fx3_mixed_5M_2"  # 0.84
base_agent_15 = "teb_drl4_rule06_nn22_fx3_mixed_5M"  # 0.88

primitive_agents = ['teb_only',  # 0.76
                    'drl_only',  # 0.82
                    'drl03_only', 'rlca_only', 'mpc_only', 'dwa_only',
                    'teb_large_min_dist_only', 'teb_dyn_obst_only', 'arena_ros_only', 'eband_only']
primitive_agent_prefix = 'single_planners' + rospy.get_param('robot_model')

simple_all_in_one_switches = ['simple_all_in_one', 'random']

AGENTS_gv = ['simple_all_in_one', 'drl_only', 'teb_only']
eval_episodes_gv = 50
#  seed = random.randint(1, 1000)
seed_gv = 21
map_config_gv = "indoor_obs10.json"

evaluation_name_gv = "jackal"

max_episode_iterations_gv = 350


def get_paths(AGENT: str, map_config: str, evaluation_name: str, primitive_agent=False, is_random_agent=False):
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    robot_model = rospy.get_param('robot_model')
    if primitive_agent:
        paths = {
            'all_in_one_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'agent_parameters',
                                                  'single_planners', robot_model, AGENT + '.json'),
            'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', robot_model +
                                          '.model.yaml'),
            'robot_as': os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
            'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
            'drl_agents': os.path.join(dir, 'agents', 'rosnav-agents'),
            'hyperparams': os.path.join(dir, 'configs', 'hyperparameters', robot_model + '_default.json'),
            'map_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters', map_config)
        }
    else:
        paths = {
            'hyperparams': os.path.join(dir, 'agents', 'aio-agents', AGENT, 'hyperparameters.json'),
            'model': os.path.join(dir, 'agents', 'aio-agents', AGENT),
            'all_in_one_parameters': os.path.join(dir, 'agents', 'aio-agents', AGENT, 'all_in_one_parameters.json'),
            'vecnorm': os.path.join(dir, 'agents', 'aio-agents', AGENT, 'vec_normalize.pkl'),
            'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', robot_model +
                                          '.model.yaml'),
            'robot_as': os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
            'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
            'drl_agents': os.path.join(dir, 'agents', 'rosnav-agents'),
            'map_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters', map_config)
        }
    if is_random_agent:
        AGENT = "random"
    eval_numb = 0
    paths['log'] = os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'evaluation_logs',
                                evaluation_name,
                                AGENT + "_" + str(eval_numb))
    while os.path.exists(paths['log']):
        eval_numb += 1
        paths['log'] = os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'evaluation_logs',
                                    evaluation_name, AGENT + "_" + str(eval_numb))
    os.makedirs(paths['log'])

    return paths


def make_env(paths: dict,
             params: dict,
             seed: int,
             eval_episodes: int):
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
                           max_steps_per_episode=max_episode_iterations_gv, seed=seed,
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
    _L = get_laser_scan_site()  # laser scan size
    switch_distance = 1.6
    laser_scan_dynamic = obs[_L:2 * _L]
    min_dist = np.min(laser_scan_dynamic)

    if min_dist <= switch_distance:
        return 0
    else:
        return 1


def get_laser_scan_site():
    robot_model = rospy.get_param("robot_model")

    ROBOT_SETTING_PATH = rospkg.RosPack().get_path("simulator_setup")
    yaml_ROBOT_SETTING_PATH = os.path.join(
        ROBOT_SETTING_PATH, "robot", f"{robot_model}.model.yaml"
    )
    with open(yaml_ROBOT_SETTING_PATH, "r") as fd:
        robot_data = yaml.safe_load(fd)
        for plugin in robot_data["plugins"]:
            if plugin["type"] == "Laser":
                laser_angle_min = plugin["angle"]["min"]
                laser_angle_max = plugin["angle"]["max"]
                laser_angle_increment = plugin["angle"]["increment"]
                _L = int(
                    round(
                        (laser_angle_max - laser_angle_min) / laser_angle_increment
                    )
                )  # num of laser beams
                break
    return _L


def simple_all_in_one_action_probs(obs, _):
    _RS = 4  # robot state size
    _L = get_laser_scan_site()  # laser scan size
    switch_distance = 1.6
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


def evaluate_agents(AGENTS: [str], eval_episodes: int, seed: int, map_config: str, evaluation_name: str):
    summary_all: [[str]] = []
    for AGENT in AGENTS:
        print(f"START RUNNING AGENT:    {AGENT}")

        if AGENT == "random":
            paths = get_paths('base_Agent', map_config, evaluation_name, is_random_agent=True)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params, seed, eval_episodes)])
            policy = random_agent(env.env_method("get_number_models")[0])
            action_probs = random_action_probs(env.env_method("get_number_models")[0])
            env = VecNormalize(env, norm_obs=False, norm_reward=False)
        elif AGENT in primitive_agents:
            paths = get_paths(AGENT, map_config, evaluation_name, primitive_agent=True)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params, seed, eval_episodes)])
            env = VecNormalize(env, norm_obs=False, norm_reward=False)

            def policy(_):
                return 0

            def action_probs(_, __):
                return [np.log(1.0)]

        elif AGENT == "simple_all_in_one":
            paths = get_paths(AGENT, map_config, evaluation_name, primitive_agent=True)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params, seed, eval_episodes)])
            env = VecNormalize(env, norm_obs=False, norm_reward=False)
            policy = simple_all_in_one
            action_probs = simple_all_in_one_action_probs

        else:
            paths = get_paths(AGENT, map_config, evaluation_name)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params, seed, eval_episodes)])
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
        summary = evaluator.evaluate_policy_manually(policy, action_probs, env, eval_episodes, paths['log'],
                                                     params['gamma'],
                                                     paths['all_in_one_parameters'])
        summary_all.append(summary)
        env.close()
        print("Evaluation of agent " + AGENT + " completed!")

    # save summary of all runs
    log_dir = os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'evaluation_logs',
                           evaluation_name)
    with open(log_dir + '/evaluation_summary.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(
            ["Planner", "Mean success rate", "Mean time outs", "Mean collisions", "Mean time",
             "Mean distance travelled", "Mean reward",
             "Mean computation time per second simulation time",
             "Mean computation per local planner iteration", "Mean model distribution",
             "Mean model distribution close obstacle distance", "Mean model distribution medium obstacle distance",
             "Mean model distribution large obstacle distance", "Mean policy switching prob",
             "Mean policy switching prob close obstacle distance",
             "Mean policy switching prob medium obstacle distance",
             "Mean policy switching prob large obstacle distance", ])
        for i, row in enumerate(summary_all):
            writer.writerow([AGENTS[i]] + row)

    print("EVALUATION DONE!")


if __name__ == "__main__":
    evaluate_agents(AGENTS=AGENTS_gv, eval_episodes=eval_episodes_gv, seed=seed_gv, map_config=map_config_gv,
                    evaluation_name=evaluation_name_gv)
