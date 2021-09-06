import os

import gym
import rospkg
import torch as th
import yaml
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from torch import nn

_RS = 4  # robot state size

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


class AGENT_1(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network (Nature CNN) to serve as feature extractor ahead of the policy and value head.

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 32):
        super(AGENT_1, self).__init__(observation_space, features_dim + _RS)

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

class AGENT_2(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network (Nature CNN) to serve as feature extractor ahead of the policy and value head.

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 32):
        super(AGENT_2, self).__init__(observation_space, 2 * features_dim + _RS)

        self.cnn1 = nn.Sequential(
            nn.Conv1d(1, 32, 8, 4),
            nn.ReLU(),
            nn.Conv1d(32, 64, 4, 2),
            nn.ReLU(),
            nn.Conv1d(64, 64, 3, 1),
            nn.ReLU(),
            nn.Flatten(),
        )

        self.cnn2 = nn.Sequential(
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
            n_flatten1 = self.cnn1(tensor_forward).shape[1]
            n_flatten2 = self.cnn2(tensor_forward).shape[1]

        self.fc1 = nn.Sequential(
            nn.Linear(n_flatten1, features_dim),
            nn.ReLU(),
        )

        self.fc2 = nn.Sequential(
            nn.Linear(n_flatten2, features_dim),
            nn.ReLU(),
        )

    def forward(self, observations: th.Tensor) -> th.Tensor:

        """
        :return: (th.Tensor) features,
            extracted features by the network
        """

        laser_scan_dynamic = th.unsqueeze(observations[:, _L:-_RS], 1)
        laser_scan_static = th.unsqueeze(observations[:, :_L], 1)
        robot_state = observations[:, -_RS:]

        extracted_features_1 = self.fc1(self.cnn1(laser_scan_dynamic))
        extracted_features_2 = self.fc2(self.cnn2(laser_scan_static))
        features = th.cat((extracted_features_1, extracted_features_2, robot_state), 1)

        return features


policy_kwargs_agent_1 = dict(features_extractor_class=AGENT_1,
                             features_extractor_kwargs=dict(features_dim=64),
                             net_arch=[dict(vf=[64, 64, 64], pi=[64, 64, 64])],
                             activation_fn=th.nn.ReLU)

policy_kwargs_agent_2 = dict(features_extractor_class=AGENT_2,
                             features_extractor_kwargs=dict(features_dim=64),
                             net_arch=[dict(vf=[128, 128], pi=[128, 128])],
                             activation_fn=th.nn.ReLU)

policy_kwargs_agent_11 = dict(net_arch=[dict(pi=[128, 64, 64, 32], vf=[64, 64, 64, 32])],
                              activation_fn=th.nn.ReLU)
