import os
import copy
import argparse
import time
from argparse import ArgumentParser
from typing import List

from numpy.lib.utils import info
from rl_agent.envs.wp_env_map_frame2 import WPEnvMapFrame2
import rospy
import numpy as np
import gym
from tqdm import tqdm

from rl_agent.config import get_cfg, CfgNode
from rl_agent.envs import build_env, build_env_wrapper  # SubprocVecEnv
from rl_agent.model import build_model
from rl_agent.utils.callbacks import TrainStageCallbackWP, StopTrainingOnRewardThreshold
from rl_agent.utils.debug import timeit
from rl_agent.envs import ENV_REGISTRY
from rl_agent.utils.reward import REWARD_REGISTRY
from stable_baselines3.common.vec_env import vec_normalize

from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy

from task_generator import build_task_wrapper
from task_generator.build import build_task
from task_generator.tasks import StopReset
from gym.utils import colorize


def get_default_arg_parser():
    parser = ArgumentParser()
    return parser


def setup_config(args, num_dynamic_obstacles=0):
    # get the global default config and merge it with the optional config file and arguments
    cfg = get_cfg(is_cfg_for_waypoint=True)
    # don't wannt waste too much time on config setting, otherwise need to setup new config to
    # use random task
    cfg.EVAL.CURRICULUM.INIT_STAGE_IDX = 0
    cfg.EVAL.CURRICULUM.STAGE_STATIC_OBSTACLE = (0,)
    cfg.EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE = (num_dynamic_obstacles,)
    cfg.freeze()
    return cfg


def get_namespaces():
    """get a list  of namespaces whose length is the number of environments
    Returns:
        namespaces_list(list): a list of namespaces
    """
    # identical with the one in launch file
    ns_prefix = "sim"
    num_envs = rospy.get_param("num_envs")
    assert (
        num_envs > 1
    ), "Make sure there are more that 2 simulation environments available since one of them will be used for evalutation"

    print(
        colorize(
            f"Found {num_envs} ENVS, all of them will be used for  pre-training!",
            "green",
        )
    )

    ns_list = [ns_prefix + "_" + str(i + 1) for i in range(num_envs)]
    return ns_list


def make_envs(cfg, namespaces):
    ns = ""
    train_mode = False
    task_wraps = [build_task_wrapper(cfg, ns) for ns in namespaces]
    train_env_wraps = [
        build_env_wrapper(cfg, task, ns, True, False, is_pretrain_mode_on=True)
        for task, ns in zip(task_wraps, namespaces)
    ]
    train_envs = SubprocVecEnv(train_env_wraps, start_method="fork")
    if cfg.INPUT.NORM:
        train_envs = VecNormalize(
            train_envs, training=True, norm_obs=True, norm_reward=False, clip_reward=15
        )
    return train_envs


def collect_expert_data(venv: VecNormalize, num_steps, num_skip_first_n_steps=10,folder_name = 'expert_data_map1_00'):
    # VecNormalize use moving_average to normalize the observation,
    # At the beginning it haven't been stable yet, those data will be discarded
    num_steps = int(num_steps)
    print("=" * 10 + f"Start to collect {num_steps} steps' data" + "=" * 10)
    curr_dirpath = os.path.dirname(__file__)
    pretraining_data_dirpath = os.path.join(curr_dirpath, "pretraining_data",folder_name)
    os.makedirs(pretraining_data_dirpath, exist_ok=True)
    print(f"expert data will be saved to:\n\t{pretraining_data_dirpath}")
    assert isinstance(venv.action_space, gym.spaces.Box)
    num_envs = venv.num_envs
    num_steps = int(num_steps / num_envs) * num_envs
    num_skip_first_n_steps = int(num_skip_first_n_steps / num_envs) * num_envs
    num_steps_to_save = num_steps - num_skip_first_n_steps
    assert num_steps_to_save > 0

    try:
        expert_observations_norm = np.empty(
            (num_steps_to_save,) + venv.observation_space.shape
        )
        expert_observations = np.empty(
            (num_steps_to_save,) + venv.observation_space.shape
        )

        expert_actions = np.empty((num_steps_to_save,) + venv.action_space.shape)
        obs_norm = venv.reset()
        obs = venv.get_original_obs()
        curr_total_steps = 0

        with tqdm(total=num_steps) as pbar:
            while curr_total_steps <= num_steps:
                # in pretrain mode the action doesn't matter,
                # it will listen to subgoal, move the robot to the subgoal, run the simulator for a short step
                # to get the new observation
                old_obs_norm = obs_norm
                old_obs = obs
                obs_norm, rews, dones, infos = venv.step([None] * num_envs)
                
                curr_total_steps += num_envs
                pbar.update(num_envs)
                if curr_total_steps < num_skip_first_n_steps:
                    continue
                curr_expert_actions = [
                    infos[i]["expert_action"] for i in range(venv.num_envs)
                ]
                expert_observations_norm[
                    curr_total_steps
                    - num_skip_first_n_steps : curr_total_steps
                    - num_skip_first_n_steps
                    + num_envs
                ] = old_obs_norm

                expert_observations[
                    curr_total_steps
                    - num_skip_first_n_steps : curr_total_steps
                    - num_skip_first_n_steps
                    + num_envs
                ] = old_obs
                
                expert_actions[
                    curr_total_steps
                    - num_skip_first_n_steps : curr_total_steps
                    - num_skip_first_n_steps
                    + num_envs
                ] = curr_expert_actions
    except Exception as e:
        pass

    # finally:
    #     print("saving all collected data, it may take a while...")
    #     venv.save(os.path.join(pretraining_data_dirpath,'venv'))
    #     np.savez_compressed(
    #         os.path.join(pretraining_data_dirpath, 'expert_data'),
    #         expert_actions=expert_actions,
    #         expert_observations=expert_observations,
    #         expert_observations_norm = expert_observations_norm 
    #     )
    #     print(f"Expert data have been saved to the path {pretraining_data_dirpath}")

def main():
    print("DEPRECATED")
    num_steps = 1e6
    parser = get_default_arg_parser()
    args = parser.parse_args()
    cfg = setup_config(args, 0)
    ns = get_namespaces()
    env = make_envs(cfg,ns)
    collect_expert_data(env,num_steps)


if __name__ == "__main__":
    # This version is too slow!!!!!!
    main()

# HOW TO USE
# 1. collecting export data
# 1.1
# 	* run 'roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode=true' to disable local planner and use plan manager to sample and get subgoal based on the global path generated by the global planner. The lookahead_distance param in the plan magager will automatically set properly.

