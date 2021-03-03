# inheritance needed moduls
from stable_baselines3.common.policies import ActorCriticPolicy
import torch.nn as nn
import gym
from typing import Callable, Dict, List, Optional, Tuple, Type, Union
from stable_baselines3.common.type_aliases import Schedule

import numpy as np
import torch as th

# discrete policy
class ILPolicy(ActorCriticPolicy): 
    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        lr_schedule: Schedule,
        net_arch: Optional[List[Union[int, Dict[str, List[int]]]]] = None,
        activation_fn: Type[nn.Module] = nn.Tanh,
        batch_size=64,
        *args,
        **kwargs,
    ):
        super(ILPolicy, self).__init__(observation_space, action_space, lr_schedule, net_arch, activation_fn, *args,
            **kwargs,)
        self.batch_size = batch_size

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


class ILPolicyContinuous(ActorCriticPolicy):
    '''
    TODO need a countinous policy to tackle planning algo. collected dataset.
    '''
    def __init__(
        self,
        observation_space: gym.spaces.Space,
        action_space: gym.spaces.Space,
        lr_schedule: Schedule,
        net_arch: Optional[List[Union[int, Dict[str, List[int]]]]] = None,
        activation_fn: Type[nn.Module] = nn.Tanh,
        batch_size=64,
        *args,
        **kwargs,
    ):
        super(ILPolicyContinuous, self).__init__(observation_space, action_space, lr_schedule, net_arch, activation_fn, *args,
            **kwargs,)
        self.batch_size = batch_size

    def forward(self, obs, deterministic=False):
        '''
        rewrite forward function to get supervised learning usable gradient graph
        '''
        pass