import os
from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import gym
import rospkg
import torch as th
import yaml

from torch import nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticCnnPolicy
from .build import FEATURE_REGISTRY
_RS = 2  # robot state size
_RSWP = 4 # relative subgoal, relative global goal

ROBOT_SETTING_PATH = rospkg.RosPack().get_path('simulator_setup')
yaml_ROBOT_SETTING_PATH = os.path.join(
    ROBOT_SETTING_PATH, 'robot', 'myrobot.model.yaml')

with open(yaml_ROBOT_SETTING_PATH, 'r') as fd:
    robot_data = yaml.safe_load(fd)
    for plugin in robot_data['plugins']:
        if plugin['type'] == 'Laser':
            laser_angle_min = plugin['angle']['min']
            laser_angle_max = plugin['angle']['max']
            laser_angle_increment = plugin['angle']['increment']
            _L = int(round((laser_angle_max - laser_angle_min) /
                     laser_angle_increment) + 1)  # num of laser beams
            break







@FEATURE_REGISTRY.register()
class CNN_Laser_MLP_Goal_Dyn_Obs(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network (Nature CNN) to serve as feature extractor ahead of the policy and value head.
    Architecture was taken as reference from: https://github.com/ethz-asl/navrep

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """
    # dont have so much time to use config to set this, in many places those stuffs have been hardcoded.sorry for that
    # check robot_feature in obstervation_collector
    ROBOT_GOAL_FEATURE_INDICES = [_L+1,_L+ 6]
    ROBOT_FEATURE_SIZE = 6
    # maximum 10 dynamical objects's state will be collected
    # 1.less? append 'blank object'
    # 2. more? choose 10 most close ones
    NUM_DYN_OBS = 10
    DYN_OBS_FEATURE_SIZE:int = 6

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 32+32+6):
        super().__init__(observation_space, features_dim)

        self.laser_cnn = nn.Sequential(
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
            n_flatten = self.laser_cnn(tensor_forward).shape[1]

        self.laser_fc = nn.Sequential(
            nn.Linear(n_flatten, 32),
	    nn.ReLU()
        )

        self.dynamic_obstacle_fc = nn.Sequential(
            nn.Linear(self.DYN_OBS_FEATURE_SIZE*self.NUM_DYN_OBS,32),
            nn.ReLU(),
            nn.Linear(32,32),
            nn.ReLU()
	)


    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor) features,
            extracted features by the network
        """

        scan_raw_feature = th.unsqueeze(observations[:, :_L], 1)
        robot_raw_feature = observations[:, _L:_L+6]
        dynamic_obs_raw_feature = observations[:,_L+6:]
        
        scan_feature = self.laser_fc(self.laser_cnn(scan_raw_feature))
        dynamic_obs_feature = self.dynamic_obstacle_fc(dynamic_obs_raw_feature)
        features = th.cat((scan_feature,robot_raw_feature ,dynamic_obs_feature),1)

        return features



