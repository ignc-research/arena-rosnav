from ..utils import Registry
from ..config import CfgNode
from ..feature_extract import get_policy_kwargs

MODEL_REGISTRY = Registry('reinforcement learning model registry')




def build_model(cfg:CfgNode,env,tensorboard_log:str = '',debug:bool = False):
    # policy name is insignificant, according to the implementation of
    # MLpPolicy and CnnPolicy, the only difference between them is 
    # the default feature extractor, since we use our customized one
    # it doesn't matter.
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
    


    

    

    
