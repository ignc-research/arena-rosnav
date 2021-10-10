import os
import copy
import argparse
import time
from argparse import ArgumentParser
from typing import List, Union
import sys
from numpy.lib.utils import info
from torch._C import device
import torch
from rl_agent.envs.wp_env_map_frame2 import WPEnvMapFrame2
from rl_agent.envs.wp_env_map_frame3 import WPEnvMapFrame3
from rl_agent.utils.debug import DummyFile, nostdout
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
        "--expert_data_np_name", "-e",
        type=str
    )
    parser.add_argument(
        "--epochs", default=500, type=int,
    )
    parser.add_argument(
        "--name_prefix",
        help="The name prefix to to the pretrained model and environment",
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
    print(
        f"setup_config:Forced to set STAGE_STATIC_OBSTACLE 0 and STAGE_DYNAMIC_OBSTACLE {num_dynamic_obstacles} "
    )
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
    env: Union[WPEnvMapFrame2, WPEnvMapFrame3],
    num_steps,
    pretraining_data_dirpath,
    expert_data_np_name,
):
    num_steps = int(num_steps)
    print("=" * 10 + f"Start to collect {num_steps} steps' data" + "=" * 10)
    curr_dirpath = os.path.abspath(os.path.dirname(__file__))
    print(
        f"expert data will be saved to:\n\t{os.path.join(pretraining_data_dirpath,expert_data_np_name)}"
    )
    # assert isinstance(env.action_space, gym.spaces.Box)

    try:
        expert_observations = np.empty((num_steps,) + env.observation_space.shape)
        expert_actions_raw_xy = np.empty((num_steps,) + (2,))
        expert_action_indices = np.empty((num_steps,))
        expert_actions = np.empty((num_steps,) + env.action_space.shape)
        obs = env.reset()
        for i in tqdm(range(num_steps), file=sys.stdout):
            # in pretrain mode the action doesn't matter,
            # it will listen to subgoal, move the robot to the subgoal, run the simulator for a short step
            # to get the new observation]
            # in case it is really a bad pose, and the expert also have no idea to put the goal
            # then you can put the goal on a close obstale to make it fail on purpose.
            with nostdout():
                try_times = 3
                while try_times > 0:
                    try_times -= 1
                    obs, reward, done, info = env.step(None)
                    raw_expert_action_x, raw_expert_action_y, action_idx = info[
                        "expert_action"
                    ]
                    is_success = info["is_success"]
                    if not done or done and is_success:
                        expert_observations[i] = obs
                        expert_actions_raw_xy[i] = [
                            raw_expert_action_x,
                            raw_expert_action_y,
                        ]
                        expert_action_indices[i] = action_idx
                        break
                    # collide or timeout
                    print(
                        colorize(
                            "\tReach waypoint failed, this waypoint ignored", "magenta"
                        )
                    )
                    if try_times == 0:
                        obs = env.reset()
                        print(
                            colorize(
                                "Continuously failure detected, env reset", "yellow"
                            )
                        )
                        try_times = 2
                if done and is_success:
                    obs = env.reset()

    finally:
        np.savez_compressed(
            os.path.join(pretraining_data_dirpath, expert_data_np_name),
            expert_observations=expert_observations[:i],
            expert_actions_raw_xy=expert_actions_raw_xy[:i],
            expert_action_indices=expert_action_indices[:i],
        )
        print(
            f"expert data have been saved to:\n\t{os.path.join(pretraining_data_dirpath, expert_data_np_name)}"
        )


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
    learning_rate=1e-3,
    log_interval=1000,
    use_cuda=True,
    seed=1,
    save_on_every_num_epoch=50,
    name_prefix="",
):

    """
    behavioural clone
    """

    class ExpertDataSet(Dataset):
        def __init__(self, expert_observations, expert_actions, num_observation=None):
            if num_observation is not None:
                num_observation = min(num_observation, len(expert_observations))
            else:
                num_observation = len(expert_observations)
            self.observations = expert_observations[:num_observation, :]
            self.actions = expert_actions[:num_observation]

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

    def train(
        policy,
        device,
        train_loader,
        optimizer,
        criterion,
        log_interval,
        epoch,
        tensorboard_log: str = "",
        ent_weight: float = 0,
        l2_weight: float = 0.0,
    ):
        from torch.distributions import Categorical
        # scripts adapted from
        # https://colab.research.google.com/github/Stable-Baselines-Team/rl-colab-notebooks/blob/sb3/pretraining.ipynb
        policy.train()
        running_loss = .0
        for batch_idx, (obs, acts) in enumerate(train_loader):

            obs, acts = obs.to(device).detach(), acts.to(device).detach()
            _, log_prob, entropy = policy.evaluate_actions(obs, acts)
            prob_true_act = th.exp(log_prob).mean()
            log_prob = log_prob.mean()
            entropy = entropy.mean()
            # TODO_D VAE skip
            l2_norms = [th.sum(th.square(w)) for w in policy.parameters()]
            l2_norm = sum(l2_norms) / 2  # divide by 2 to cancel with gradient of square

            ent_loss = -ent_weight * entropy
            neglogp = -log_prob
            l2_loss = l2_weight * l2_norm
            # loss = neglogp + ent_loss + l2_loss
            loss = neglogp

            stats_dict = dict(
                neglogp=neglogp.item(),
                loss=loss.item(),
                entropy=entropy.item(),
                ent_loss=ent_loss.item(),
                prob_true_act=prob_true_act.item(),
                l2_norm=l2_norm.item(),
                l2_loss=l2_loss.item(),
            )
            # TODO_D tensorboard log

            optimizer.zero_grad()
            loss.backward(retain_graph=False)
            optimizer.step()
            if batch_idx % log_interval == 0:
                print(
                    "Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}".format(
                        epoch,
                        batch_idx * len(obs),
                        len(train_loader.dataset),
                        100.0 * batch_idx / len(train_loader),
                        loss.item(),
                    )
                )

    def save_norm_env_policy(epoch_idx, policy, vec_norm, name_prefix=""):
        if len(name_prefix):
            name_prefix += "_"

        pretrain_policy_filepath = os.path.join(
            pretraining_data_dirpath, f"{name_prefix}pretrain_policy_{epoch_idx}.pkl"
        )
        pretrain_vecnorm_obs_rms_filepath = os.path.join(
            pretraining_data_dirpath,
            f"{name_prefix}pretrain_vecnorm_obs_rms_{epoch_idx}.pkl",
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
    # TODO continious action need convertion if the collected data are generated with an env having discrete action space
    expert_observation, expert_actions = (
        expert_data["expert_observations"],
        expert_data["expert_action_indices"],
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
    # optimizer = optim.Adadelta(policy.parameters(), lr=learning_rate)
    optimizer = optim.SGD(policy.parameters(), lr=learning_rate, momentum=0.9)


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
            save_norm_env_policy(
                epoch_idx=epoch,
                policy=policy,
                vec_norm=vec_norm,
                name_prefix=name_prefix,
            )


def main():
    parser = get_default_arg_parser()
    args = parser.parse_args()
    print(args)
    NUM_STEPS = int(1e4)
    # EXPERT_DATA_NP_NAME = "expert_data_emptymap_00.npz"
    # overwrite dynamic obstacle here
    cfg = setup_config(args, 20)
    env = make_env(cfg)
    curr_dirpath = os.path.abspath(os.path.dirname(__file__))
    env_class_name = env.__class__.__name__
    pretraining_data_dirpath = os.path.join(
        curr_dirpath, "pretraining_data", env_class_name
    )
    os.makedirs(pretraining_data_dirpath, exist_ok=True)
    # you can change it
    if not args.expert_data_np_name:
        expert_data_np_name = env_class_name + ".npz"
    else:
        expert_data_np_name = args.expert_data_np_name 
    if args.phase1:
        collect_expert_data(
            env, NUM_STEPS, pretraining_data_dirpath, expert_data_np_name
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
            expert_data_np_name,
            epochs=EPOCHS,
            save_on_every_num_epoch=20,
            name_prefix=args.name_prefix,
        )


if __name__ == "__main__":
    main()

# muti env is slow(reason: sync ), therefore we still use single env
# HOW TO USE
# 1. collecting export data
# 1.1
# 	* run 'roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode=true' to disable local planner and use plan manager to sample and get subgoal based on the global path generated by the global planner. The lookahead_distance param in the plan magager will automatically set properly.

