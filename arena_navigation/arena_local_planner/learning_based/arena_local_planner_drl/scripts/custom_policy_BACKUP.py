import os
from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import gym
import rospkg
import torch as th
import yaml

from torch import nn
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

_RS = 2  # robot state size

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

    def _build_mlp_extractor(self) -> None:
        self.mlp_extractor = MLP_ARENA2D(64)

class AGENT_1(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network to serve as feature extractor ahead of the policy and value network.
    Architecture was taken as reference from: https://arxiv.org/abs/1808.03841

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 128):
        super(AGENT_1, self).__init__(observation_space, features_dim+_RS)

        self.cnn = nn.Sequential(
            nn.Conv1d(1, 32, 5, 2),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            # tensor_forward = th.as_tensor(observation_space.sample()[None]).float()
            tensor_forward = th.randn(1, 1, _L)
            n_flatten = self.cnn(tensor_forward).shape[1]

        self.fc_1 = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )


    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor),
            extracted features by the network
        """
        laser_scan = th.unsqueeze(observations[:, :-_RS], 1)
        robot_state = observations[:, -_RS:]

        extracted_features = self.fc_1(self.cnn(laser_scan))
        features = th.cat((extracted_features, robot_state), 1)

        return features


"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_drl_local_planner: (dict)
"""
policy_kwargs_agent_1 = dict(features_extractor_class=AGENT_1,
                             features_extractor_kwargs=dict(features_dim=256),
                             net_arch=[dict(vf=[64], pi=[64])], 
                             activation_fn=th.nn.ReLU)

class AGENT_2(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network to serve as feature extractor ahead of the policy and value network.
    Architecture was taken as reference from: https://arxiv.org/abs/1808.03841
    (DRL_LOCAL_PLANNER)

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 128):
        super(AGENT_2, self).__init__(observation_space, features_dim+_RS)

        self.cnn = nn.Sequential(
            nn.Conv1d(1, 32, 5, 2),
            nn.ReLU(),
            nn.Conv1d(32, 32, 3, 2),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            # tensor_forward = th.as_tensor(observation_space.sample()[None]).float()
            tensor_forward = th.randn(1, 1, _L)
            n_flatten = self.cnn(tensor_forward).shape[1]

        self.fc_1 = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )

    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor),
            extracted features by the network
        """
        laser_scan = th.unsqueeze(observations[:, :-_RS], 1)
        robot_state = observations[:, -_RS:]

        extracted_features = self.fc_1(self.cnn(laser_scan))
        features = th.cat((extracted_features, robot_state), 1)

        return features

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_drl_local_planner: (dict)
"""
policy_kwargs_agent_2 = dict(features_extractor_class=AGENT_2,
                             features_extractor_kwargs=dict(features_dim=256),
                             net_arch=[dict(vf=[128], pi=[128])], 
                             activation_fn=th.nn.ReLU)

class AGENT_3(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network to serve as feature extractor ahead of the policy and value network.

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 128):
        super(AGENT_3, self).__init__(observation_space, features_dim+_RS)

        self.cnn = nn.Sequential(
            nn.Conv1d(1, 32, 5, 2),
            nn.ReLU(),
            nn.Conv1d(32, 32, 3, 2),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            # tensor_forward = th.as_tensor(observation_space.sample()[None]).float()
            tensor_forward = th.randn(1, 1, _L)
            n_flatten = self.cnn(tensor_forward).shape[1]

        self.fc_1 = nn.Sequential(
            nn.Linear(n_flatten, 256),
            nn.ReLU(),
        )

        self.fc_2 = nn.Sequential(
            nn.Linear(256, features_dim),
            nn.ReLU()
        )

    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor),
            extracted features by the network
        """
        laser_scan = th.unsqueeze(observations[:, :-_RS], 1)
        robot_state = observations[:, -_RS:]

        extracted_features = self.fc_2(self.fc_1(self.cnn(laser_scan)))
        features = th.cat((extracted_features, robot_state), 1)

        # return self.fc_2(features)
        return features


"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_drl_local_planner: (dict)
"""
policy_kwargs_agent_3 = dict(features_extractor_class=AGENT_3,
                             features_extractor_kwargs=dict(features_dim=(128)),
                             net_arch=[dict(vf=[64, 64], pi=[64, 64])], 
                             activation_fn=th.nn.ReLU)

class AGENT_4(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network (Nature CNN) to serve as feature extractor ahead of the policy and value head.
    Architecture was taken as reference from: https://github.com/ethz-asl/navrep
    (CNN_NAVREP)

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 32):
        super(AGENT_4, self).__init__(observation_space, features_dim + _RS)

        self.cnn = nn.Sequential(
            nn.Conv1d(1, 32, 8, 4),
            nn.ReLU(),
            nn.Conv1d(32, 64, 9, 4),
            nn.ReLU(),
            nn.Conv1d(64, 128, 6, 4),
            nn.ReLU(),
            nn.Conv1d(128, 256, 4, 4),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            tensor_forward = th.randn(1, 1, _L)
            n_flatten = self.cnn(tensor_forward).shape[1]

        self.fc = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )

    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor) features,
            extracted features by the network
        """

        laser_scan = th.unsqueeze(observations[:, :-_RS], 1)
        robot_state = observations[:, -_RS:]

        extracted_features = self.fc(self.cnn(laser_scan))
        features = th.cat((extracted_features, robot_state), 1)

        return features


"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_4 = dict(features_extractor_class=AGENT_4,
                             features_extractor_kwargs=dict(features_dim=32),
                             net_arch=[dict(vf=[64, 64], pi=[64, 64])], 
                             activation_fn=th.nn.ReLU)


"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_5 = dict(net_arch=[128, dict(pi=[64], vf=[64])], 
                            activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_6 = dict(net_arch=[128, 64, dict(pi=[64, 64], vf=[64, 64])], 
                             activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_7 = dict(net_arch=[128, 64, 64, dict(pi=[64, 64], vf=[64, 64])], 
                             activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_8 = dict(net_arch=[64, 64, 64, dict(pi=[64, 64], vf=[64, 64])], 
                             activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_drl_local_planner: (dict)
"""
policy_kwargs_agent_9 = dict(features_extractor_class=AGENT_1,
                             features_extractor_kwargs=dict(features_dim=256),
                             net_arch=[dict(vf=[64, 64], pi=[64, 64])], 
                             activation_fn=th.nn.ReLU)

class AGENT_10(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network (Nature CNN) to serve as feature extractor ahead of the policy and value head.

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 32):
        super(AGENT_10, self).__init__(observation_space, features_dim + _RS)

        self.cnn = nn.Sequential(
            nn.Conv1d(1, 32, 8, 4),
            nn.ReLU(),
            nn.Conv1d(32, 64, 4, 2),
            nn.ReLU(),
            nn.Conv1d(64, 64, 3, 1),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Compute shape by doing one forward pass
        with th.no_grad():
            tensor_forward = th.randn(1, 1, _L)
            n_flatten = self.cnn(tensor_forward).shape[1]

        self.fc = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU(),
        )

    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor) features,
            extracted features by the network
        """

        laser_scan = th.unsqueeze(observations[:, :-_RS], 1)
        robot_state = observations[:, -_RS:]

        extracted_features = self.fc(self.cnn(laser_scan))
        features = th.cat((extracted_features, robot_state), 1)

        return features

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_10 = dict(features_extractor_class=AGENT_10,
                              features_extractor_kwargs=dict(features_dim=512),
                              net_arch=[dict(vf=[128], pi=[128])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_11 = dict(features_extractor_class=AGENT_10,
                              features_extractor_kwargs=dict(features_dim=512),
                              net_arch=[dict(vf=[64, 64], pi=[64, 64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_12 = dict(features_extractor_class=AGENT_10,
                              features_extractor_kwargs=dict(features_dim=64),
                              net_arch=[dict(vf=[64, 64], pi=[64, 64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_13 = dict(net_arch=[64, 64, dict(pi=[64, 32], vf=[64, 32])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_14 = dict(net_arch=[64, 64, dict(pi=[64], vf=[64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_15 = dict(net_arch=[64, 64, 64, 64, dict(pi=[64, 64], vf=[64, 64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_16 = dict(net_arch=[64, 64, 64, 64, dict(pi=[64, 64, 64], vf=[64, 64, 64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_17 = dict(features_extractor_class=AGENT_10,
                              features_extractor_kwargs=dict(features_dim=64),
                              net_arch=[dict(vf=[64, 64, 64], pi=[64, 64, 64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_18 = dict(features_extractor_class=AGENT_4,
                              features_extractor_kwargs=dict(features_dim=64),
                              net_arch=[dict(vf=[64, 64, 64], pi=[64, 64, 64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_19 = dict(net_arch=[128, 128, 128, dict(pi=[64, 64], vf=[64, 64])], 
                              activation_fn=th.nn.ReLU)

"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_kwargs_navrep: (dict)
"""
policy_kwargs_agent_20 = dict(net_arch=[128, 128, 128, 128, dict(pi=[64, 64, 64], vf=[64, 64, 64])], 
                              activation_fn=th.nn.ReLU)