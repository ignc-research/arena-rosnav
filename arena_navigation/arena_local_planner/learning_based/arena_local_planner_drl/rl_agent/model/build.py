from typing import List,Type,Tuple
from torch import nn
import numpy as np
from stable_baselines3.common.policies import(
    register_policy,
    ActorCriticPolicy,
)
from stable_baselines3.common.distributions import (
    BernoulliDistribution,
    CategoricalDistribution,
    DiagGaussianDistribution,
    Distribution,
    MultiCategoricalDistribution,
    StateDependentNoiseDistribution,
    make_proba_distribution,
)
from functools import partial
from stable_baselines3.common.policies import ActorCriticPolicy as ActorCriticPolicy_
from stable_baselines3.common.preprocessing import get_action_dim, maybe_transpose, preprocess_obs
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor, FlattenExtractor, MlpExtractor, NatureCNN, create_mlp
from stable_baselines3.common.type_aliases import Schedule
from stable_baselines3.common.utils import get_device, is_vectorized_observation
from stable_baselines3.common.vec_env.obs_dict_wrapper import ObsDictWrapper
from ..utils import Registry
from ..config import CfgNode
from ..feature_extract import get_policy_kwargs

def create_sde_features_extractor(
    features_dim: int, sde_net_arch: List[int], activation_fn: Type[nn.Module]
) -> Tuple[nn.Sequential, int]:
    """
    Create the neural network that will be used to extract features
    for the gSDE exploration function.

    :param features_dim:
    :param sde_net_arch:
    :param activation_fn:
    :return:
    """
    # Special case: when using states as features (i.e. sde_net_arch is an empty list)
    # don't use any activation function
    sde_activation = activation_fn if len(sde_net_arch) > 0 else None
    latent_sde_net = create_mlp(features_dim, -1, sde_net_arch, activation_fn=sde_activation, squash_output=False)
    latent_sde_dim = sde_net_arch[-1] if len(sde_net_arch) > 0 else features_dim
    sde_features_extractor = nn.Sequential(*latent_sde_net)
    return sde_features_extractor, latent_sde_dim
class ActorCriticPolicyVAE(ActorCriticPolicy):
    """a customized policy which won't initialize vae weight and bias 

    Args:
        ActorCriticPolicy ([type]): [description]
    """
    def __init__(self,*args, **kwargs):
        super().__init__(*args, **kwargs)

    def _build(self, lr_schedule) -> None:
        """
        Create the networks and the optimizer.

        :param lr_schedule: Learning rate schedule
            lr_schedule(1) is the initial learning rate
        """
        self._build_mlp_extractor()

        latent_dim_pi = self.mlp_extractor.latent_dim_pi

        # Separate features extractor for gSDE
        if self.sde_net_arch is not None:
            self.sde_features_extractor, latent_sde_dim = create_sde_features_extractor(
                self.features_dim, self.sde_net_arch, self.activation_fn
            )

        if isinstance(self.action_dist, DiagGaussianDistribution):
            self.action_net, self.log_std = self.action_dist.proba_distribution_net(
                latent_dim=latent_dim_pi, log_std_init=self.log_std_init
            )
        elif isinstance(self.action_dist, StateDependentNoiseDistribution):
            latent_sde_dim = latent_dim_pi if self.sde_net_arch is None else latent_sde_dim
            self.action_net, self.log_std = self.action_dist.proba_distribution_net(
                latent_dim=latent_dim_pi, latent_sde_dim=latent_sde_dim, log_std_init=self.log_std_init
            )
        elif isinstance(self.action_dist, CategoricalDistribution):
            self.action_net = self.action_dist.proba_distribution_net(latent_dim=latent_dim_pi)
        elif isinstance(self.action_dist, MultiCategoricalDistribution):
            self.action_net = self.action_dist.proba_distribution_net(latent_dim=latent_dim_pi)
        elif isinstance(self.action_dist, BernoulliDistribution):
            self.action_net = self.action_dist.proba_distribution_net(latent_dim=latent_dim_pi)
        else:
            raise NotImplementedError(f"Unsupported distribution '{self.action_dist}'.")

        self.value_net = nn.Linear(self.mlp_extractor.latent_dim_vf, 1)
        # Init weights: use orthogonal initialization
        # with small initial weight for the output
        if self.ortho_init:
            # TODO: check for features_extractor
            # Values from stable-baselines.
            # features_extractor/mlp values are
            # originally from openai/baselines (default gains/init_scales).
            module_gains = {
                self.features_extractor: np.sqrt(2),
                self.mlp_extractor: np.sqrt(2),
                self.action_net: 0.01,
                self.value_net: 1,
            }
            for module, gain in module_gains.items():
                for m in module.children():
                    # frezzing vae parameter
                    if m.__class__.__name__ =='VAE':
                        print("Skip randomly initializing VAE Module")
                        continue
                    m.apply(partial(self.init_weights, gain=gain))
register_policy("ActorCriticPolicyVAE",ActorCriticPolicyVAE)

MODEL_REGISTRY = Registry('reinforcement learning model registry')




def build_model(cfg:CfgNode,env,tensorboard_log:str = '',debug:bool = False):
    # policy name is insignificant, according to the implementation of
    # MLpPolicy and CnnPolicy, the only difference between them is 
    # the default feature extractor, since we use our customized one
    # it doesn't matter.
    if "VAE" in cfg.NET_ARCH.FEATURE_EXTRACTOR.NAME:
        print(f"FOUND 'VAE' in feature_extractor's name {cfg.NET_ARCH.FEATURE_EXTRACTOR.NAME}")
        policy_name = "ActorCriticPolicyVAE"
        print(f"Loading policy {policy_name} and vae params will not be randomly initialized")
    else:   
        policy_name = 'MlpPolicy'
    
    policy_kwargs = get_policy_kwargs(cfg)
    model_name = cfg.MODEL.NAME
    # the keys not passed to the constractor
    skipped_keys = ['NAME']
    model_kwargs = {k.lower():v for k,v in cfg.MODEL.items() if k not in skipped_keys }
    model_kwargs['policy'] = policy_name
    model_kwargs['verbose'] = 1 if not debug else 2
    model_kwargs['env'] = env
    model_kwargs['tensorboard_log'] = tensorboard_log
    model_kwargs['policy_kwargs'] = policy_kwargs
    return MODEL_REGISTRY.get(model_name)(**model_kwargs)
    


    

    

    
