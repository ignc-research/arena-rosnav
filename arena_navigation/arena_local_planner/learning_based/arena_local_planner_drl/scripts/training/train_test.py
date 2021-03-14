import gym

from stable_baselines3 import A2C
from stable_baselines3.common.vec_env.subproc_vec_env import SubprocVecEnv

envs = []
for _ in range(5):
    env = lambda: gym.make('CartPole-v1')
    envs.append(env)

vec_env = SubprocVecEnv(envs,start_method='fork')



model = A2C('MlpPolicy',vec_env, verbose=1)
model.learn(total_timesteps=10000)

obs = env.reset()
for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
    # env.render()
    if done:
      obs = env.reset()