from typing import Type, List

from abc import ABC, abstractmethod
from enum import Enum
from torch.nn.modules.module import Module
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class PolicyType(Enum):
    CNN = "CnnPolicy"
    MLP = "MlpPolicy"


class BaseAgent(ABC):
    """Base class for models loaded on runtime from
    the Stable-Baselines3 policy registry during PPO instantiation.
    The architecture of the eventual policy is determined by the
    'policy_kwargs' of the SB3 RL algorithm.
    """

    def __init__(self):
        pass

    @property
    @abstractmethod
    def type(self) -> PolicyType:
        pass

    @property
    @abstractmethod
    def features_extractor_class(self) -> Type[BaseFeaturesExtractor]:
        pass

    @property
    @abstractmethod
    def features_extractor_kwargs(self) -> dict:
        pass

    @property
    @abstractmethod
    def net_arch(self) -> List[dict]:
        pass

    @property
    @abstractmethod
    def activation_fn(self) -> Type[Module]:
        pass

    def get_kwargs(self):
        kwargs = {
            "features_extractor_class": self.features_extractor_class,
            "features_extractor_kwargs": self.features_extractor_kwargs,
            "net_arch": self.net_arch,
            "activation_fn": self.activation_fn,
        }
        if not kwargs['features_extractor_class']:
            del kwargs['features_extractor_class']
        if not kwargs['features_extractor_kwargs']:
            del kwargs['features_extractor_kwargs']
        return kwargs
