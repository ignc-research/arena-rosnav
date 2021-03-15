from typing import List
from rl_agent.config import CfgNode
from rl_agent.utils import Registry 

TASK_REGISTRY = Registry("task registry")


def build_task(cfg:CfgNode,ns):
    task_name = cfg.TASK.NAME
    return TASK_REGISTRY.get(task_name)(cfg,ns)

def build_tasks(cfg,namespaces:List[str]):
    tasks = [build_task(cfg,ns) for ns in namespaces]
    return tasks


def build_task_wrappers(cfg,namespaces:List[str]):
    def task_wrapper(cfg,ns):
        def task_wrappee():
            return build_task(cfg,ns)
        return task_wrappee
    tasks = [task_wrapper(cfg,ns) for ns in namespaces]
    return tasks


def build_task_wrapper(cfg:CfgNode,namespace:str):
    # def wrappe():
    #     return build_task(cfg,namespace)
    # return wrappe
    return lambda : build_task(cfg,namespace)