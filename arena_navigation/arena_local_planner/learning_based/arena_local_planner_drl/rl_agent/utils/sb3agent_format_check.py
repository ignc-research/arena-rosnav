from typing import Type

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from torch.nn.modules.module import Module

from rl_agent.model.base_agent import BaseAgent, PolicyType


def check_format(cls: Type[BaseAgent]):
    assert isinstance(cls.type, PolicyType), "Type has to be of type 'PolicyType'!"

    if cls.features_extractor_class:
        assert issubclass(
            cls.features_extractor_class, BaseFeaturesExtractor
        ), "Feature extractors have to derive from 'BaseFeaturesExtractor'!"

    if cls.features_extractor_kwargs:
        assert (
            type(cls.features_extractor_kwargs) is dict
        ), "Features extractor kwargs have to be of type 'dict'!"

    if cls.net_arch:
        assert (
            type(cls.net_arch) is list
        ), "Network architecture kwargs have to be of type 'list'!"
        for entry in cls.net_arch:
            assert (
                type(entry) is dict or type(entry) is int
            ), "Network architecture entries have to be of either type 'list' or 'dict'!"
            if type(entry) is dict:
                assert "pi" in entry or "vf" in entry, (
                    "net_arch dictionaries have to contain either 'pi' or 'vf'"
                    "for the respective network head!"
                )

    if cls.activation_fn:
        assert issubclass(
            cls.activation_fn, Module
        ), "Activation functions have to be taken from torch!"
