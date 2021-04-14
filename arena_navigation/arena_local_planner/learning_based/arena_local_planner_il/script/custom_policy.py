import os
from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import gym
import rospkg
import torch as th
import yaml
import numpy as np

from torch import nn
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

_RS = 6  # robot state size

ROBOT_SETTING_PATH = rospkg.RosPack().get_path('simulator_setup')
yaml_ROBOT_SETTING_PATH = os.path.join(ROBOT_SETTING_PATH, 'robot', 'myrobot.model.yaml')

with open(yaml_ROBOT_SETTING_PATH, 'r') as fd:
    robot_data = yaml.safe_load(fd)
    for plugin in robot_data['plugins']:
        if plugin['type'] == 'Laser':
            laser_angle_min = plugin['angle']['min']
            laser_angle_max = plugin['angle']['max']
            laser_angle_increment = plugin['angle']['increment']
            _L = int(round((laser_angle_max - laser_angle_min) / laser_angle_increment) + 1)  # num of laser beams
            break


class MLP_ARENA2D(nn.Module):
    """
    Custom Multilayer Perceptron for policy and value function.
    Architecture was taken as reference from: https://github.com/ignc-research/arena2D/tree/master/arena2d-agents.

    :param feature_dim: dimension of the features extracted with the features_extractor (e.g. features from a CNN)
    :param last_layer_dim_pi: (int) number of units for the last layer of the policy network
    :param last_layer_dim_vf: (int) number of units for the last layer of the value network
    """

    def __init__(
            self,
            feature_dim: int,
            last_layer_dim_pi: int = 32,
            last_layer_dim_vf: int = 32,
    ):
        super(MLP_ARENA2D, self).__init__()

        # Save output dimensions, used to create the distributions
        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf

        # Body network
        self.body_net = nn.Sequential(
            nn.Linear(_L+_RS, 64),
            nn.ReLU(),
            nn.Linear(64, feature_dim),
            nn.ReLU()
        )

        # Policy network
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim, last_layer_dim_pi),
            nn.ReLU()
        )

        # Value network
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim, last_layer_dim_vf),
            nn.ReLU()
        )

    def forward(self, features: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        """
        :return: (th.Tensor, th.Tensor) latent_policy, latent_value of the specified network.
            If all layers are shared, then ``latent_policy == latent_value``
        """
        body_x = self.body_net(features)
        return self.policy_net(body_x), self.value_net(body_x)


class MLP_ARENA2D_POLICY(ActorCriticPolicy):
    """
    Policy using the custom Multilayer Perceptron.
    """

    def __init__(
            self,
            observation_space: gym.spaces.Space,
            action_space: gym.spaces.Space,
            lr_schedule: Callable[[float], float],
            batch_size,
            net_arch: Optional[List[Union[int, Dict[str, List[int]]]]] = None,
            activation_fn: Type[nn.Module] = nn.ReLU,
            *args,
            **kwargs,
    ):
        super(MLP_ARENA2D_POLICY, self).__init__(
            observation_space,
            action_space,
            lr_schedule,
            net_arch,
            activation_fn,
            *args,
            **kwargs,
        )
        # Enable orthogonal initialization
        self.ortho_init = True
        self.batch_size = batch_size

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = MLP_ARENA2D(64)

    def forward(self, obs, deterministic=False):
        '''
        rewrite forward function to get supervised learning usable gradient graph
        '''
        latent_pi, latent_vf, latent_sde = self._get_latent(obs)
        values = self.value_net(latent_vf)
        distribution = self._get_action_dist_from_latent(latent_pi, latent_sde=latent_sde)
        actions = distribution.get_actions(deterministic=deterministic)
        
        actions_list = np.array([np.array([i for i in range(distribution.action_dim)]) for n in range(self.batch_size)])
        actions_list = th.from_numpy(actions_list.T).to(self.device)
        log_prob = distribution.log_prob(actions)
        log_prob_list = distribution.log_prob(actions_list)
        log_prob_list = th.transpose(log_prob_list, 0, 1)
        self.log_prob_list = log_prob_list
        return actions, values, log_prob

class MLP_ARENA2D_CONTINUOUS_POLICY(ActorCriticPolicy):
    """
    Policy using the custom Multilayer Perceptron.
    """

    def __init__(
            self,
            observation_space: gym.spaces.Space,
            action_space: gym.spaces.Space,
            lr_schedule: Callable[[float], float],
            batch_size,
            net_arch: Optional[List[Union[int, Dict[str, List[int]]]]] = None,
            activation_fn: Type[nn.Module] = nn.ReLU,
            *args,
            **kwargs,
    ):
        super(MLP_ARENA2D_CONTINUOUS_POLICY, self).__init__(
            observation_space,
            action_space,
            lr_schedule,
            net_arch,
            activation_fn,
            *args,
            **kwargs,
        )
        # Enable orthogonal initialization
        self.ortho_init = True
        self.batch_size = batch_size

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = MLP_ARENA2D(64)

