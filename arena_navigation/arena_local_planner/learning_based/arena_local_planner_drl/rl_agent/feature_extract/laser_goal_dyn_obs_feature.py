import os
from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import gym
import rospkg
import torch as th
import yaml
import torch
from torch import nn
import numpy as np
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticCnnPolicy
from .build import FEATURE_REGISTRY
from .ring import lidar_to_rings
from .vae import VAE


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
class CNN_Laser_Obstalcle_GlobalPlan(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network (Nature CNN) to serve as feature extractor ahead of the policy and value head.
    Architecture was taken as reference from: https://arxiv.org/pdf/1905.05279.pdf

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
    NUM_POSES_GLOBAL_PLAN= 10

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 64+64+64):
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
            nn.Linear(n_flatten, 64),
	        nn.ReLU()
        )
        self.global_path_fc = nn.Sequential(
            nn.Linear(CNN_Laser_Obstalcle_GlobalPlan.NUM_POSES_GLOBAL_PLAN*2, 32),
            nn.ReLU(),
            nn.Linear(32,64),
        )
        self.attention_coff_fc = nn.Sequential(
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64,4),
        )

        self.dynamic_obstacle_fc = nn.Sequential(
            nn.Linear(self.DYN_OBS_FEATURE_SIZE*self.NUM_DYN_OBS,32),
            nn.ReLU(),
            nn.Linear(32,64),
            nn.ReLU()
	)   
        self.features_fc = nn.Sequential(
            nn.Linear(features_dim,features_dim),
            nn.ReLU(),
        )

    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor) features,
            extracted features by the network
        """

        scan_raw_feature = th.unsqueeze(observations[:, :_L], 1)
        dynamic_obs_raw_feature = observations[:,_L:-CNN_Laser_Obstalcle_GlobalPlan.NUM_POSES_GLOBAL_PLAN*2]
        global_path_raw_feature = observations[:,-CNN_Laser_Obstalcle_GlobalPlan.NUM_DYN_OBS*2:]



        scan_feature = self.laser_fc(self.laser_cnn(scan_raw_feature))
        dynamic_obs_feature = self.dynamic_obstacle_fc(dynamic_obs_raw_feature)
        global_path_feature = self.global_path_fc(global_path_raw_feature)
        all_features = th.cat((scan_feature, dynamic_obs_feature,global_path_feature),1)
        attention_coff = th.softmax(self.attention_coff_fc(all_features),dim=1)
        all_features_with_attention = th.cat((scan_feature*attention_coff[:,0][:,None],dynamic_obs_feature*attention_coff[:,1][:,None],global_path_feature*attention_coff[:,2][:,None]),axis=1)

        return all_features_with_attention


@FEATURE_REGISTRY.register()
class CNN_LaserVAE_Obstalcle_GlobalPlan(BaseFeaturesExtractor):
    """
    use pretrained encoder to to encode laser 

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
    # num_poses_global_path in wp_env_map_frame3.py
    NUM_DYN_OBS = 10
    DYN_OBS_FEATURE_SIZE:int = 6
    NUM_POSES_GLOBAL_PLAN= 10

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 64+64+64):
        print("Make sure disable input normalization")
        super().__init__(observation_space, features_dim)

        self.laser_fc = nn.Sequential(
            nn.Linear(32, 64),
	        nn.ReLU()
        )
        self.global_path_fc = nn.Sequential(
            nn.Linear(CNN_Laser_Obstalcle_GlobalPlan.NUM_POSES_GLOBAL_PLAN*2, 32),
            nn.ReLU(),
            nn.Linear(32,64),
            
        )
        self.attention_coff_fc = nn.Sequential(
            nn.Linear(64*3, 128),
            nn.ReLU(),
            nn.Linear(128,3),
        )

        self.dynamic_obstacle_fc = nn.Sequential(
            nn.Linear(self.DYN_OBS_FEATURE_SIZE*self.NUM_DYN_OBS,32),
            nn.ReLU(),
            nn.Linear(32,64),
            nn.ReLU()
	)   
        self.features_fc = nn.Sequential(
            nn.Linear(features_dim,features_dim),
            nn.ReLU(),
        )
        # copied from ring.py.bak, kind of tricky but if we don't do it, we can not pickel the model.
        x = np.linspace(0, 1, 64 - 2)
        laser_range_max = 3.6
        expansion_term=1
        min_resolution=0.01
        min_dist=0.1
        expansion_curve = np.power(x, expansion_term)  # a.k.a range_level_depths
        renormalisation_factor = np.sum(expansion_curve) / (
            laser_range_max - (min_resolution * (64 - 2)) - min_dist
        )
        range_level_depths = expansion_curve / renormalisation_factor + min_resolution
        range_level_maxs = np.cumsum(range_level_depths) + min_dist
        range_level_maxs = np.concatenate([[min_dist], range_level_maxs, [np.inf]]).astype(np.float32)
        self.ring_range_level_mins = np.concatenate([[0.0], range_level_maxs[:-1]]).astype(np.float32)
        self.ring_range_level_maxs = range_level_maxs

        self.load_vae()

    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor) features,
            extracted features by the network
        """

        scan_raw_feature = observations[:, :_L]
        scan_np = scan_raw_feature.cpu().numpy()
        scan_np = np.ascontiguousarray(scan_np,dtype=np.float32)
        # copied from ring.py.bak, kind of tricky but if we don't do it, we can not pickel the model.
        scan  = lidar_to_rings(scan_np,64,64,self.ring_range_level_mins,self.ring_range_level_maxs)/2.0
        dynamic_obs_raw_feature = observations[:,_L:-CNN_Laser_Obstalcle_GlobalPlan.NUM_DYN_OBS*2]
        global_path_raw_feature = observations[:,-CNN_Laser_Obstalcle_GlobalPlan.NUM_DYN_OBS*2:]

        with torch.no_grad():
            vae_input =th.tensor(np.transpose(scan,[0,3,1,2]),dtype=th.float32).detach()
            if observations.is_cuda:
                vae_input = vae_input.to(device='cuda')
            scan_encoded,*_= self.vae.encode(vae_input)
        scan_encoded = scan_encoded.detach()

        scan_feature = self.laser_fc(scan_encoded)
        dynamic_obs_feature = self.dynamic_obstacle_fc(dynamic_obs_raw_feature)
        global_path_feature = self.global_path_fc(global_path_raw_feature)

        all_features = th.cat((scan_feature, dynamic_obs_feature,global_path_feature),1)
        attention_coff = th.softmax(self.attention_coff_fc(all_features),dim=1)
        
        # attention_coff = th.softmax(self.attention_coff_fc(all_features),dim=1).unsqueeze(1)
        # all_features_att_applied = th.bmm(attention_coff,all_features.view(-1,3,64))

        all_features_with_attention = th.cat((scan_feature*attention_coff[:,0][:,None],dynamic_obs_feature*attention_coff[:,1][:,None],global_path_feature*attention_coff[:,2][:,None]),axis=1)

        return all_features_with_attention

    def load_vae(self):
        self.vae = VAE()
        self.vae.load_from_file()








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
        self.features_fc = nn.Sequential(
            nn.Linear(features_dim,features_dim),
            nn.ReLU(),
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
        features1 = th.cat((scan_feature,robot_raw_feature ,dynamic_obs_feature),1)
        features2 = self.features_fc(features1)
        return features2




@FEATURE_REGISTRY.register()
class Pure_Dyn_Obs(BaseFeaturesExtractor):
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

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 32+6):
        super().__init__(observation_space, features_dim)

    #     self.dynamic_obstacle_fc = nn.Sequential(
    #         nn.Linear(self.DYN_OBS_FEATURE_SIZE*self.NUM_DYN_OBS,32),
    #         nn.ReLU(),
    #         nn.Linear(32,32),
    #         nn.ReLU()
	# )
        self.dynamic_obstacle_fc = nn.Sequential(
            nn.Linear(self.DYN_OBS_FEATURE_SIZE*self.NUM_DYN_OBS,128),
            nn.ReLU(),
            nn.Linear(128,128),
            nn.ReLU()
	)

        self.dynamic_obstacle_fc2 = nn.Sequential(
            nn.Linear(features_dim, features_dim),
            nn.ReLU(),
            nn.Linear(features_dim, features_dim),
            nn.ReLU()
        )


    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor) features,
            extracted features by the network
        """
        robot_raw_feature = observations[:, _L:_L+6]
        dynamic_obs_raw_feature = observations[:,_L+6:]
        
        dynamic_obs_feature = self.dynamic_obstacle_fc(dynamic_obs_raw_feature)
        features = th.cat((robot_raw_feature ,dynamic_obs_feature),1)
        features = self.dynamic_obstacle_fc2(features)

        return features

@FEATURE_REGISTRY.register()
class Pure_Static_Obs(BaseFeaturesExtractor):
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

        self.fc = nn.Sequential(
            nn.Linear(32+3,features_dim),
            nn.ReLU(),
	)


    def forward(self, observations: th.Tensor) -> th.Tensor:
        """
        :return: (th.Tensor) features,
            extracted features by the network
        """

        scan_raw_feature = th.unsqueeze(observations[:, :_L], 1)
        robot_raw_feature = observations[:, _L:_L+3]
        
        scan_feature = self.laser_fc(self.laser_cnn(scan_raw_feature))
        features = th.cat((scan_feature,robot_raw_feature),1)
        features2 = self.fc(features)
        return features2