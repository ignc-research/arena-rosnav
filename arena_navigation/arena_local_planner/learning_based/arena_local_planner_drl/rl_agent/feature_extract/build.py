from rl_agent.utils import Registry
import torch as th

FEATURE_REGISTRY = Registry('feature extractor registry')


def get_policy_kwargs(cfg):
    feature_extractor_name = cfg.NET_ARCH.FEATURE_EXTRACTOR.NAME

    features_extractor_class = FEATURE_REGISTRY.get(feature_extractor_name)
    # convert other key values except for the NAME
    # In config we always set the key with uppercase,so we need to convert here
    features_extractor_kwargs = {
        k.lower(): v for k, v in cfg.NET_ARCH.FEATURE_EXTRACTOR.items() if k != 'NAME'}
    # it can also be set in config,if its necessary
    vf = cfg.NET_ARCH.VF
    pi = cfg.NET_ARCH.PI
    net_arch = [dict(vf=[64, 64], pi=[64, 64])]
    activation_fn = th.nn.ReLU

    return dict(features_extractor_class=features_extractor_class,
                features_extractor_kwargs=features_extractor_kwargs,
                net_arch=net_arch,
                activation_fn=activation_fn)

