import os
import copy
import argparse
import time
from argparse import ArgumentParser
from typing import List

from numpy.lib.utils import info
from torch._C import device
import torch
from rl_agent.envs.wp_env_map_frame2 import WPEnvMapFrame2
import rospy
import numpy as np
import gym
from tqdm import tqdm
import pickle
from typing import Optional

from rl_agent.config import get_cfg, CfgNode
from rl_agent.envs import build_env, build_env_wrapper  # SubprocVecEnv
from rl_agent.model import build_model
from rl_agent.utils.callbacks import TrainStageCallbackWP, StopTrainingOnRewardThreshold
from rl_agent.utils.debug import timeit
from rl_agent.envs import ENV_REGISTRY
from rl_agent.utils.reward import REWARD_REGISTRY
from stable_baselines3 import ppo
from stable_baselines3.common.vec_env import vec_normalize
from stable_baselines3.ppo import PPO

from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.evaluation import evaluate_policy

from torch.utils.data.dataset import Dataset
import torch as th
import torch.nn as nn
import torch.optim as optim
from torch.optim.lr_scheduler import StepLR

import torch as th
from task_generator import build_task_wrapper
from task_generator.build import build_task
from task_generator.tasks import StopReset
from gym.utils import colorize


def get_default_arg_parser():
    parser = ArgumentParser()
    parser.add_argument(
        "--phase1", "-p1", help="phase 1 is to collect expert data", action="store_true"
    )
    parser.add_argument(
        "--phase2",
        "-p2",
        help="phase 2 is to use collected data to initialize the network",
        action="store_true",
    )
    parser.add_argument(
        "--expert_data_np_name",
        '-e',
        default="expert_data_map1_00.npz"
    )
    parser.add_argument(
        "--epochs",
        default=50,
        type=int,
    )
    parser.add_argument(
        "--name_prefix",
        help = 'The name prefix to to the pretrained model and environment',
        default="",
    )
    parser.add_argument(
        "opt",
        help="Overwrite the config loaded from the defaults and config file by providing a list of key and value pairs,\
            In deployment model This feature is disabled.",
        default=None,
        nargs=argparse.REMAINDER,
    )
    return parser


def setup_config(args, num_dynamic_obstacles=0):
    # get the global default config and merge it with the optional config file and arguments
    cfg = get_cfg(is_cfg_for_waypoint=True)
    # don't wannt waste too much time on config setting, otherwise need to setup new config to
    # use random task
    cfg.merge_from_list(args.opt)
    cfg.EVAL.CURRICULUM.INIT_STAGE_IDX = 0
    cfg.EVAL.CURRICULUM.STAGE_STATIC_OBSTACLE = (0,)
    cfg.EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE = (num_dynamic_obstacles,)
    cfg.freeze()
    return cfg


def make_env(cfg):
    ns = ""
    train_mode = False
    task = build_task(cfg, ns)
    # set pretrain mode
    env = build_env(cfg, task, ns, train_mode, False, is_pretrain_mode_on=True)
    return env


def collect_expert_data(
    env: WPEnvMapFrame2, num_steps, pretraining_data_dirpath, expert_data_np_name
):
    num_steps = int(num_steps)
    print("=" * 10 + f"Start to collect {num_steps} steps' data" + "=" * 10)
    curr_dirpath = os.path.abspath(os.path.dirname(__file__))
    pretraining_data_dirpath = os.path.join(curr_dirpath, "pretraining_data")
    os.makedirs(pretraining_data_dirpath, exist_ok=True)
    print(f"expert data will be saved to:\n\t{pretraining_data_dirpath}")
    # assert isinstance(env.action_space, gym.spaces.Box)
    try:
        expert_observations = np.empty((num_steps,) + env.observation_space.shape)
        expert_actions = np.empty((num_steps,) + env.action_space.shape)
        obs = env.reset()
        for i in tqdm(range(num_steps)):
            # in pretrain mode the action doesn't matter,
            # it will listen to subgoal, move the robot to the subgoal, run the simulator for a short step
            # to get the new observation
            obs, reward, done, info = env.step(None)
            action = info["expert_action"]
            expert_observations[i] = obs
            expert_actions[i] = action
            if done:
                obs = env.reset()
    finally:
        np.savez_compressed(
            os.path.join(pretraining_data_dirpath, expert_data_np_name),
            expert_actions=expert_actions,
            expert_observations=expert_observations,
        )
        print(f"expert data have been saved to:\n\t{pretraining_data_dirpath}")


def pretraining_network(
    cfg,
    args,
    model: PPO,
    env,
    pretraining_data_dirpath,
    expert_data_np_name,
    batch_size=64,
    epochs=100,
    num_observation=None,
    scheduler_gamma=0.7,
    learning_rate=1e-2,
    log_interval=1000,
    use_cuda=True,
    seed=1,
    save_on_every_num_epoch=10,
    name_prefix = ''
):
    class ExpertDataSet(Dataset):
        def __init__(self, expert_observations, expert_actions, num_observation=None):
            if num_observation is not None:
                num_observation = min(num_observation, len(expert_observations))
            else:
                num_observation = len(expert_observations)
            self.observations = expert_observations[:num_observation, :]
            self.actions = expert_actions[:num_observation, :]

            # filter the data 
            # idx1 = expert_observations[:, 360] > 0.5
            # idx2 = np.abs((expert_actions**2).sum(axis=1)**0.5-2)<1
            # a = np.arctan2(expert_observations[:,361],expert_observations[:,362])
            # b = np.arctan2(expert_actions[:,0],expert_actions[:,1])
            # c = np.stack((np.abs(a-b),2*np.pi-np.abs(a-b)),axis=1)
            # d = np.min(c,axis=1)/np.pi*180
            # idx3 = d<11

            # idx = np.logical_and(idx1, idx2, idx3)
            # observations = expert_observations[idx]
            # actions = expert_actions[idx]
            
            # thetas = np.arctan2(observations[:,361],observations[:,362])/np.pi*180
            # idx1 = np.logical_and(thetas>-50,thetas<50)
            # idx2 = np.logical_and(thetas>-180,thetas<-150)
            # idx3 = np.logical_and(thetas>-130,thetas<180)            
            # idx = np.logical_or(idx1,idx2,idx3)
            # observations_augumented = np.vstack([observations,observations[idx]])
            # actions_augumented = np.vstack([actions,actions[idx]])

            # self.observations = observations_augumented
            # self.actions = actions_augumented


            # theta = np.arctan2(self.observations[:,361],self.observations[:,362])
            # rho  = 2
            # fake_actions = np.array([rho*np.cos(theta),rho*np.sin(theta)]).T

            # self.actions = expert_actions[:num_observation, :]
            # self.actions = self.actions[idx]
            # self.actions = fake_actions
            self.norm_rms = False

        def register_env_norm(self, venv_norm: VecNormalize):
            self.norm_rms = True
            self.venv_norm = venv_norm

        def __getitem__(self, index):
            if self.norm_rms:
                return (
                    self.venv_norm.normalize_obs(self.observations[index]),
                    self.actions[index],
                )
            else:
                return (self.observations[index], self.actions[index])

        def __len__(self):
            return len(self.observations)

    def train(policy, device, train_loader, optimizer, criterion, log_interval, epoch):
        # scripts adapted from
        # https://colab.research.google.com/github/Stable-Baselines-Team/rl-colab-notebooks/blob/sb3/pretraining.ipynb
        policy.train()
        for batch_idx, (data, target) in enumerate(train_loader):
            data, target = data.to(device), target.to(device)
            optimizer.zero_grad()
            action, _, _ = policy(data)
            action_prediction = action.double()

            loss = criterion(action_prediction, target)
            loss.backward()
            optimizer.step()
            if batch_idx % log_interval == 0:
                print(
                    "Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}".format(
                        epoch,
                        batch_idx * len(data),
                        len(train_loader.dataset),
                        100.0 * batch_idx / len(train_loader),
                        loss.item(),
                    )
                )

    def save_norm_env_policy(epoch_idx, policy, vec_norm,name_prefix=''):
        if len(name_prefix):
            name_prefix += '_'

        pretrain_policy_filepath = os.path.join(
            pretraining_data_dirpath, f"{name_prefix}pretrain_policy_{epoch_idx}.pkl"
        )
        pretrain_vecnorm_obs_rms_filepath = os.path.join(
            pretraining_data_dirpath, f"{name_prefix}pretrain_vecnormvecnorm_obs_rms_{epoch_idx}.pkl"
        )
        with open(pretrain_policy_filepath, "wb") as f:
            pickle.dump(policy, f)
            print(f"Successfully saved the model to {pretrain_policy_filepath}")
        if cfg.INPUT.NORM:
            with open(pretrain_vecnorm_obs_rms_filepath, "wb") as f:
                pickle.dump(vec_norm.obs_rms, f)
                print(f"Successfully saved rms to {pretrain_vecnorm_obs_rms_filepath}")

    # load data
    filepath = os.path.join(pretraining_data_dirpath, expert_data_np_name)

    print(f"Loading numpy array from {filepath}")
    expert_data = np.load(filepath)
    expert_observation, expert_actions = (
        expert_data["expert_observations"],
        expert_data["expert_actions"],
    )
    print(f"loaded expert data successfully!")
    expert_dataset = ExpertDataSet(expert_observation, expert_actions)

    th.manual_seed(seed)
    if cfg.INPUT.NORM:
        vec_norm = VecNormalize(
            DummyVecEnv([lambda: env]), True, norm_obs=True, norm_reward=False
        )
        # initialize obs with moving average
        num_observation_warm = int(1e3)
        random_ints = np.random.choice(
            len(expert_observation), size=num_observation_warm
        )
        expert_observation_vec_warm = expert_observation[random_ints, :]
        vec_norm.obs_rms.update(expert_observation_vec_warm)
        print("Successfully initialize the normlized env")
    else:
        vec_norm = None
    if vec_norm is not None:
        expert_dataset.register_env_norm(vec_norm)
    train_loader = th.utils.data.DataLoader(
        dataset=expert_dataset, batch_size=batch_size, shuffle=True
    )

    use_cuda = use_cuda and th.cuda.is_available()
    device = th.device("cuda" if use_cuda else "cpu")
    policy = model.policy.to(device)
    # Define an Optimizer and a learning rate schedule.
    optimizer = optim.Adadelta(policy.parameters(), lr=learning_rate)
    scheduler = StepLR(optimizer, step_size=1, gamma=scheduler_gamma)

    # Now we are finally ready to train the policy model.
    for epoch in range(1, epochs + 1):
        train(
            policy,
            device,
            train_loader,
            optimizer,
            criterion=nn.MSELoss(),
            log_interval=log_interval,
            epoch=epoch,
        )
        if epoch % save_on_every_num_epoch == 0:
            save_norm_env_policy(epoch_idx=epoch, policy=policy, vec_norm=vec_norm,name_prefix=name_prefix)
        scheduler.step()


def main():    
    parser = get_default_arg_parser()
    args = parser.parse_args()
    print(args)
    NUM_STEPS = 1e5
    EXPERT_DATA_NP_NAME = args.expert_data_np_name
    # EXPERT_DATA_NP_NAME = "expert_data_emptymap_00.npz"
    curr_dirpath = os.path.abspath(os.path.dirname(__file__))
    pretraining_data_dirpath = os.path.join(curr_dirpath, "pretraining_data")

    os.makedirs(pretraining_data_dirpath, exist_ok=True)

    cfg = setup_config(args, 0)
    env = make_env(cfg)
    if args.phase1:
        collect_expert_data(
            env, NUM_STEPS, pretraining_data_dirpath, EXPERT_DATA_NP_NAME
        )
    if args.phase2:
        model = build_model(cfg, env)
        EPOCHS = args.epochs
        pretraining_network(
            cfg,
            args,
            model,
            env,
            pretraining_data_dirpath,
            EXPERT_DATA_NP_NAME,
            epochs=EPOCHS,
            save_on_every_num_epoch=10,
            name_prefix=args.name_prefix,
        )


if __name__ == "__main__":
    main()

# muti env is slow(reason: sync ), therefore we still use single env
# HOW TO USE
# 1. collecting export data
# 1.1
# 	* run 'roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode=true' to disable local planner and use plan manager to sample and get subgoal based on the global path generated by the global planner. The lookahead_distance param in the plan magager will automatically set properly.

