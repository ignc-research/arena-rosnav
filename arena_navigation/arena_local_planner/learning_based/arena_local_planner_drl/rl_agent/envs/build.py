from typing import Any, Callable
from ..utils import Registry
from ..config import CfgNode
from typing import Any,Callable,List,Union
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

# task can be callable object, use string here to avoid circle import
def build_env_wrapper(cfg:CfgNode,task:Union[Callable,"task_generator.AbsTask"],ns:str,train_mode:bool,debug:bool):
    def wrap():
        env = build_env(cfg,task,ns,train_mode=train_mode,debug=debug)
        return env
    return wrap
    