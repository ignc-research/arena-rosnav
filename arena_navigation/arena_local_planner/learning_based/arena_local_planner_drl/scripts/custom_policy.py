import os
from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import gym
import rospkg
import torch as th
import yaml

from torch import nn
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.policy_sarl_utils import SARL

# _RS = 2+2*6  # robot state size+ human state size
_RS = 9  # robot state size 3
self_state_dim = 9
num_humans =  5  #
num_robo_obstacles =  2
max_num_humans= 21
human_state_size= 19 #size of human info 19
robo_obstacle_state_size= 19 #size of human info 19
_HS= 19*21  # human state size
HIDDEN_SHAPE_LSTM=32 # hidden size of lstm
HIDDEN_SHAPE_GRU=32   # hidden size of gru

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
            nn.Linear(_L, 128),
            nn.ReLU(),
            nn.Linear(128, feature_dim),
            nn.ReLU()
        )

        # Policy network
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim+1+_RS, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_pi),
            nn.ReLU()
        )

        # Value network
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim+1+_RS, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_pi),
            nn.ReLU()
        )

    def forward(self, features: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        """
        :return: (th.Tensor, th.Tensor) latent_policy, latent_value of the specified network.
            If all layers are shared, then ``latent_policy == latent_value``
        """
        size=features.shape
        time=features[:, 0].reshape(size[0], -1)
        body_x = self.body_net(features[:, 1:_L+1])
        robot_state=features[:, _L+1:_L+1+_RS]
        # humans_state=features[:, _L+1:_L+1+num_humans*human_state_size]
        # human_hidden=self.body_net_human(humans_state)
        features_1 = th.cat((time, body_x), 1)
        # features_2=th.cat((features_1, human_hidden), 1)
        features=th.cat((features_1, robot_state), 1)
        return self.policy_net(features), self.value_net(features)


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
        self.mlp_extractor = MLP_ARENA2D(128)

class MLP_HUMAN(nn.Module):
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
        super(MLP_HUMAN, self).__init__()
        # Save output dimensions, used to create the distributions
        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf

        # Body networks
        self.body_net_laser = nn.Sequential(
            nn.Linear(_L, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, feature_dim),
            nn.ReLU()
        ).to('cuda')
        self.body_net_human = nn.Sequential(
            nn.Linear(human_state_size*num_humans+num_robo_obstacles*robo_obstacle_state_size, 128),
            nn.ReLU(),
            nn.Linear(128, 96),
            nn.ReLU(),
            nn.Linear(96, 32),
            nn.ReLU()
        ).to('cuda')

        # Policy network
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim+_RS+33, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_pi),
            nn.ReLU(),
        ).to('cuda')

        # Value network
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim+_RS+33, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, last_layer_dim_vf),
            nn.ReLU()
        ).to('cuda')

    def forward(self, features: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        """
        :return: (th.Tensor, th.Tensor) latent_policy, latent_value of the specified network.
            If all layers are shared, then ``latent_policy == latent_value``
        """
        size=features.shape
        time=features[:, 0].reshape(size[0], -1)
        body_x = self.body_net_laser(features[:, 1:_L+1])
        robot_state=features[:, _L+1:_L+1+_RS]
        humans_state=features[:, _L+1:_L+1+num_humans*human_state_size] 
        robo_obstacles_state=features[:, _L+1+max_num_humans*human_state_size :_L+1+max_num_humans*human_state_size+num_robo_obstacles*robo_obstacle_state_size]   
        human_robo_hidden=self.body_net_human(th.cat((humans_state, robo_obstacles_state), 1))
        features_1 = th.cat((time, body_x), 1)
        features_2=th.cat((features_1, human_robo_hidden), 1)
        features=th.cat((features_2, robot_state), 1)
        return self.policy_net(features), self.value_net(features)

class MLP_HUMAN_POLICY(ActorCriticPolicy):
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
        super(MLP_HUMAN_POLICY, self).__init__(
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
        self.features_dim=64
        self.mlp_extractor = MLP_HUMAN(self.features_dim)

class DRL_LOCAL_PLANNER(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network to serve as feature extractor ahead of the policy and value network.
    Architecture was taken as reference from: https://arxiv.org/abs/1808.03841

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 128):
        super(DRL_LOCAL_PLANNER, self).__init__(observation_space, features_dim)

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

        extracted_features = self.fc_1(self.cnn(laser_scan))
        features = th.cat((extracted_features, robot_state), 1)

        # return self.fc_2(features)
        return features


"""
Global constant to be passed as an argument to the PPO of Stable-Baselines3 in order to build both the policy
and value network.

:constant policy_drl_local_planner: (dict)
"""
policy_kwargs_drl_local_planner = dict(features_extractor_class=DRL_LOCAL_PLANNER,
                                       features_extractor_kwargs=dict(features_dim=(256+_RS)),
                                       net_arch=[dict(vf=[128], pi=[128])], 
                                       activation_fn=th.nn.ReLU)


class CNN_NAVREP(BaseFeaturesExtractor):
    """
    Custom Convolutional Neural Network (Nature CNN) to serve as feature extractor ahead of the policy and value head.
    Architecture was taken as reference from: https://github.com/ethz-asl/navrep

    :param observation_space: (gym.Space)
    :param features_dim: (int) Number of features extracted.
        This corresponds to the number of unit for the last layer.
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 32):
        super(CNN_NAVREP, self).__init__(observation_space, features_dim)

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
            nn.Linear(n_flatten, features_dim - _RS),
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
policy_kwargs_navrep = dict(features_extractor_class=CNN_NAVREP,
                            features_extractor_kwargs=dict(features_dim=32),
                            net_arch=[dict(vf=[64, 64], pi=[64, 64])], 
                            activation_fn=th.nn.ReLU)


class MLP_LSTM(nn.Module):
    """
    Net structure of CARDRL, using LSTM for dynamic infos, 

    :param feature_dim: dimension of the features extracted with the features_extractor (e.g. features from a CNN)
    :param last_layer_dim_pi: (int) number of units for the last layer of the policy network
    :param last_layer_dim_vf: (int) number of units for the last layer of the value network
    """

    def __init__(
            self,
            feature_dim: int,
            last_layer_dim_pi: int = 64,
            last_layer_dim_vf: int = 64,
    ):
        super(MLP_LSTM, self).__init__()

        # Save output dimensions, used to create the distributions
        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf

        # Body networks
        self.body_net_fc = nn.Sequential(
            nn.Linear(_L, 256),
            nn.ReLU(),
            nn.Linear(256, feature_dim),
            nn.ReLU()
        ).to('cuda')

        self.body_net_lstm = nn.LSTM(input_size=human_state_size, hidden_size=HIDDEN_SHAPE_LSTM).to('cuda')

        # Policy network
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim +1+ _RS+ HIDDEN_SHAPE_LSTM , 96), 
            nn.ReLU(),
            nn.Linear(96, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_pi),
            nn.ReLU()
        ).to('cuda')

        # Value network
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim + 1+_RS+ HIDDEN_SHAPE_LSTM , 96), 
            nn.ReLU(),
            nn.Linear(96, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_vf),
            nn.ReLU()
        ).to('cuda')

    def forward(self, features: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        """
        :return: (th.Tensor, th.Tensor) latent_policy, latent_value of the specified network.
            If all layers are shared, then ``latent_policy == latent_value``
        """
        size=features.shape
        time=features[:, 0].reshape(size[0], -1)
        body_x = self.body_net_fc(features[:, 1:_L+1])
        robot_state=features[:, _L+1:_L+1+_RS]
        humans_state=features[:, _L+1:_L+1+num_humans*human_state_size]
        humans_state=humans_state.reshape((humans_state.shape[0],-1,human_state_size)).flip([0,1]).permute(1,0,2)
        _, (h_n, _) = self.body_net_lstm(humans_state) # feed through lstm with initial hidden state = zeros
        human_features = h_n.view(h_n.shape[1],-1)
        features_1 = th.cat((body_x, robot_state), 1)
        features_2 = th.cat((time, features_1), 1)
        features=th.cat((features_2, human_features), 1)
        return self.policy_net(features), self.value_net(features)


class MLP_LSTM_POLICY(ActorCriticPolicy):
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
        super(MLP_LSTM_POLICY, self).__init__(
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
        self.features_dim=64
        self.mlp_extractor = MLP_LSTM(self.features_dim)

class MLP_SARL(nn.Module):
    """
    Policy SARL : the idea described in paper https://arxiv.org/abs/1809.08835
    Crowd-robot interaction: Crowd-aware robot navigation with attention-based deep reinforcement learning
    :param feature_dim: dimension of the features extracted with the features_extractor (e.g. features from a CNN)
    :param last_layer_dim_pi: (int) number of units for the last layer of the policy network
    :param last_layer_dim_vf: (int) number of units for the last layer of the value network
    """

    def __init__(
            self,
            feature_dim: int,
            last_layer_dim_pi: int = 64,
            last_layer_dim_vf: int = 64,
    ):
        super(MLP_SARL, self).__init__()

        # Save output dimensions, used to create the distributions
        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf

        # Body networks
        self.body_net_fc = nn.Sequential(
            nn.Linear(_L, 256),
            nn.ReLU(),
            nn.Linear(256, feature_dim),
            nn.ReLU()
        ).to('cuda')

        sarl=SARL()
        sarl.build_net()
        sarl.set_device('cuda')
        self.body_net_human=sarl.model
        # # Policy network
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim+sarl.model.mlp3_input_dim+1,128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_pi),
            nn.ReLU()
        ).to('cuda')

        # #Value network
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim+sarl.model.mlp3_input_dim+1,128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_pi),
            nn.ReLU()
        ).to('cuda')

    def forward(self, features: th.Tensor)-> Tuple[th.Tensor, th.Tensor]:
        """
        :return: (th.Tensor, th.Tensor) latent_policy, latent_value of the specified network.
            If all layers are shared, then ``latent_policy == latent_value``
        """
        size=features.shape
        # print('feature size', size)
        time=features[:, 0].reshape(size[0], -1)
        body_x = self.body_net_fc(features[:, 1:_L+1])
        robot_state=features[:, _L+1:_L+1+_RS]
        humans_state=features[:, _L+1:_L+1+num_humans*human_state_size]
        humans_state=humans_state.reshape((humans_state.shape[0],-1,human_state_size)).flip([0,1])
        # joint state includes robot state
        joint_state=self.body_net_human(humans_state) # feed through SARL
        features_1 = th.cat((time, body_x), 1)
        features_value=th.cat((features_1,joint_state), 1)
        # print('value feature size', features_value.shape)
        # humans_state = humans_state.permute(1, 0, 2)
        # _, h_n = self.body_net_gru(humans_state) # feed through gru with initial hidden state = zeros
        # human_features = h_n.view(h_n.shape[1], -1)
        # features_2 = th.cat((features_1, robot_state), 1)
        # features_policy = th.cat((features_2, human_features), 1)
        return self.policy_net(features_value), self.value_net(features_value)



class MLP_SARL_POLICY(ActorCriticPolicy):
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
        super(MLP_SARL_POLICY, self).__init__(
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
        self.features_dim=64
        self.mlp_extractor = MLP_SARL(self.features_dim)

class MLP_GRU(nn.Module):
    """
    Structure idea of CARDRL, using GRU for dynamic infos, 

    :param feature_dim: dimension of the features extracted with the features_extractor (e.g. features from a CNN)
    :param last_layer_dim_pi: (int) number of units for the last layer of the policy network
    :param last_layer_dim_vf: (int) number of units for the last layer of the value network
    """

    def __init__(
            self,
            feature_dim: int,
            last_layer_dim_pi: int = 64,
            last_layer_dim_vf: int = 64,
    ):
        super(MLP_GRU, self).__init__()

        # Save output dimensions, used to create the distributions
        self.latent_dim_pi = last_layer_dim_pi
        self.latent_dim_vf = last_layer_dim_vf

        # Body networks
        self.body_net_fc = nn.Sequential(
            nn.Linear(_L, 256),
            nn.ReLU(),
            nn.Linear(256, feature_dim),
            nn.ReLU()
        ).to('cuda')

        self.body_net_gru = nn.GRU(input_size=human_state_size, hidden_size=HIDDEN_SHAPE_GRU).to('cuda')

        # Policy network
        self.policy_net = nn.Sequential(
            nn.Linear(feature_dim +1+ _RS + HIDDEN_SHAPE_GRU, 96),
            nn.ReLU(),
            nn.Linear(96, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_pi),
            nn.ReLU()
        ).to('cuda')

        # Value network
        self.value_net = nn.Sequential(
            nn.Linear(feature_dim + 1+_RS + HIDDEN_SHAPE_GRU, 96),
            nn.ReLU(),
            nn.Linear(96, 64),
            nn.ReLU(),
            nn.Linear(64, last_layer_dim_vf),
            nn.ReLU()
        ).to('cuda')

    def forward(self, features: th.Tensor) -> Tuple[th.Tensor, th.Tensor]:
        """
        :return: (th.Tensor, th.Tensor) latent_policy, latent_value of the specified network.
            If all layers are shared, then ``latent_policy == latent_value``
        """
        size=features.shape
        time=features[:, 0].reshape(size[0], -1)
        body_x = self.body_net_fc(features[:, 1:_L+1])
        robot_state = features[:, _L+1:_L+1+_RS]
        humans_state = features[:, _L+1:_L+1+num_humans*human_state_size]
        humans_state = humans_state.reshape((humans_state.shape[0],-1,human_state_size)).flip([0,1]).permute(1,0,2)
        _, h_n = self.body_net_gru(humans_state) # feed through gru with initial hidden state = zeros
        human_features = h_n.view(h_n.shape[1],-1)
        features_1 = th.cat((body_x, robot_state), 1)
        features_2 = th.cat((time, features_1), 1)
        features = th.cat((features_2, human_features), 1)
        return self.policy_net(features), self.value_net(features)


class MLP_GRU_POLICY(ActorCriticPolicy):
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
        super(MLP_GRU_POLICY, self).__init__(
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
        self.features_dim=64
        self.mlp_extractor = MLP_GRU(self.features_dim)