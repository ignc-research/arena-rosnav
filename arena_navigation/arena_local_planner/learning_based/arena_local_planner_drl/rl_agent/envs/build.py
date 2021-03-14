from ..utils import Registry
from ..config import CfgNode

ENV_REGISTRY = Registry('environment registry')


def build_env(cfg:CfgNode,task,ns,train_mode,debug):
    """build a env 

    Args:
        cfg (CfgNode): [description]
        task (ABSTask): Specific Task
        ns ([type]): namespace 
    """
    env_name  = cfg.ENV.NAME
    return ENV_REGISTRY.get(env_name)(cfg,task,ns,train_mode,debug)
    