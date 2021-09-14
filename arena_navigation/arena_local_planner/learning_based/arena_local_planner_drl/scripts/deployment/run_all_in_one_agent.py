import os
import sys
import time
import warnings

import numpy as np
import rospkg
import rospy
from rl_agent.envs.all_in_one_flatland_gym_env import AllInOneEnv
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from task_generator.tasks import StopReset

from scripts.training.evaluate_all_in_one_agent import load_hyperparameters_json, simple_all_in_one, \
    simple_all_in_one_action_probs
from tools.all_in_one_utils import Evaluator
from tools.argsparser import parse_run_agent_args
from tools.train_agent_utils import print_hyperparameters

PRIMITIVE_AGENTS = ['simple_all_in_one', 'random', 'drl_only', 'teb_only', 'rlca_only', 'mpc_only']
AVAILABLE_AGENTS = ['mixed_teb_drl4_rule06_policy2', 'simple_all_in_one']
AGENTS = ['mixed_teb_drl4_rule06_policy2']
max_steps_per_episode = np.inf
eval_episodes = 1000


def get_paths(AGENT: str, args, primitive_agent=False, is_random_agent=False):
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    if primitive_agent:
        paths = {
            'scenario': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'scenarios',
                                     args.scenario + '.json'),
            'all_in_one_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'agent_parameters',
                                                  AGENT + '.json'),
            'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml'),
            'robot_as': os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
            'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
            'drl_agents': os.path.join(dir, 'agents'),
            'hyperparams': os.path.join(dir, 'configs', 'hyperparameters', 'all_in_one_default.json'),
        }
    else:
        paths = {
            'scenario': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'scenarios',
                                     args.scenario + '.json'),
            'hyperparams': os.path.join(dir, 'agents', AGENT, 'hyperparameters.json'),
            'model': os.path.join(dir, 'agents', AGENT),
            'all_in_one_parameters': os.path.join(dir, 'agents', AGENT, 'all_in_one_parameters.json'),
            'vecnorm': os.path.join(dir, 'agents', AGENT, 'vec_normalize.pkl'),
            'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml'),
            'robot_as': os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
            'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
            'drl_agents': os.path.join(dir, 'agents'),
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


def make_env(with_ns: bool,
             PATHS: dict,
             PARAMS: dict,
             log: bool = False):
    """
    Utility function for multiprocessed env

    :param params: (dict) hyperparameters of agent to be trained
    :param PATHS: (dict) script relevant paths
    :param log: (bool) to differentiate between train and eval env
    :return: (Callable)
    """

    def _init():
        ns = f"eval_sim" if with_ns else ""

        env = AllInOneEnv(
            ns, PATHS['robot_setting'], PATHS['robot_as'], PARAMS['reward_fnc'],
            goal_radius=0.05, max_steps_per_episode=max_steps_per_episode, train_mode=False,
            paths=PATHS, extended_eval=True, run_scenario=True)
        if log:
            # eval env
            env = Monitor(
                env, PATHS['log'], False,
                info_keywords=(
                    "collisions", "distance_travelled", "time_safe_dist", "time", "done_reason", "is_success"))
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
        if AGENT in PRIMITIVE_AGENTS:
            if AGENT == "simple_all_in_one":
                paths = get_paths(AGENT, args, primitive_agent=True)
                params = load_hyperparameters_json(paths)
                print_hyperparameters(params)
                env = DummyVecEnv([make_env(ns_for_nodes, paths, params, args.log)])
                env = VecNormalize(env, norm_obs=False, norm_reward=False)
                policy = simple_all_in_one
                action_probs = simple_all_in_one_action_probs
            else:
                paths = get_paths(AGENT, args, primitive_agent=True)
                params = load_hyperparameters_json(paths)
                print_hyperparameters(params)
                env = DummyVecEnv([make_env(ns_for_nodes, paths, params, args.log)])
                env = VecNormalize(env, norm_obs=False, norm_reward=False)


                def policy(_):
                    return 0


                def action_probs(_, __):
                    return [np.log(1.0)]

            evaluator = Evaluator()
            evaluator.evaluate_policy_manually(policy, action_probs, env, eval_episodes, paths['log'], params['gamma'],
                                               paths['all_in_one_parameters'], log_statistics=False)
        else:
            PATHS = get_paths(AGENT, args, False, False)

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

        env.close()

    time = round(time.time() - start)
    print(f"Time passed:    {time}s")
    print("EVALUATION DONE!")
    sys.exit()
