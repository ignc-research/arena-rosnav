from stable_baselines3 import PPO as PPO_
from .build import MODEL_REGISTRY



@MODEL_REGISTRY.register()
def PPO(*args,**kwargs):
    return PPO_(*args,**kwargs)