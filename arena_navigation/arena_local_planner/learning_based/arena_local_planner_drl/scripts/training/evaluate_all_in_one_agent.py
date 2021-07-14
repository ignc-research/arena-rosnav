import json
import os
import random
import time

import rospkg
from rl_agent.envs.all_in_one_flatland_gym_env import AllInOneEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from tools.all_in_one_utils import evaluate_policy_manually
from tools.train_agent_utils import check_hyperparam_format, print_hyperparameters

base_Agent = 'all_in_one_agents_teb_rlca_rule03_policy13'
AGENTS = [base_Agent, "random", "rlca_only", "teb_only", "drl_only"]
eval_episodes = 40
seed = random.randint(1,1000)

def get_paths(AGENT: str, primitive_agent=False, is_random_agent=False):
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    if primitive_agent:
        paths = {
            'all_in_one_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', AGENT + '.json'),
            'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml'),
            'robot_as': os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
            'curriculum': os.path.join(dir, 'configs', 'training_curriculum_map1small.yaml'),
            'drl_agents': os.path.join(dir, 'agents'),
            'hyperparams': os.path.join(dir, 'configs', 'hyperparameters', 'all_in_one_default.json')
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
            'drl_agents': os.path.join(dir, 'agents')
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
            env = VecNormalize(env)
        elif AGENT in ['rlca_only', 'teb_only', 'drl_only']:
            paths = get_paths(AGENT, primitive_agent=True)
            params = load_hyperparameters_json(paths)
            print_hyperparameters(params)
            env = DummyVecEnv([make_env(paths, params)])

            def policy(_):
                return 0
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

        evaluate_policy_manually(policy, env, eval_episodes, paths['log'], params['gamma'],
                                 paths['all_in_one_parameters'])

        env.close()
        print("Evaluation of agent " + AGENT + " completed!")

    time = round(time.time() - start)
    print(f"Time passed:    {time}s")
    print("EVALUATION DONE!")
