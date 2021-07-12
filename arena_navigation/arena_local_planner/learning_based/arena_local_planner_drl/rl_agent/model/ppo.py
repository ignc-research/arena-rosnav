from stable_baselines3 import PPO as PPO_
from .build import MODEL_REGISTRY


@MODEL_REGISTRY.register()
def PPO(*args, **kwargs):
    need_convert_keys = ['clip_range', 'clip_range_vf']
    for k, v in kwargs.items():
        if k in need_convert_keys and isinstance(v, str):
            if 'n_step' not in v or 'res':
                raise ValueError(
                    f"string {v} are assigned in {k}, it should be function with the parameter 'n_step' ")
            def f(n_step):
                local = {'n_step':n_step}
                exec(v,{},local)
                return local['res']
            v = f
    return PPO_(*args, **kwargs)
