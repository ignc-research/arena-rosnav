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

