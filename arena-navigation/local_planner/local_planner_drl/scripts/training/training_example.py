import os
from stable_baselines3 import A2C
from rl_agent.envs.flatland_gym_env import FlatlandEnv
from task_generator.tasks import get_predefined_task
import rospy
import rospkg

rospy.init_node("test")
task = get_predefined_task()
models_folder_path = rospkg.RosPack().get_path('simulator_setup')
flatland_local_planner_drl_folder_path = rospkg.RosPack().get_path('flatland_local_planner_drl')


env = FlatlandEnv(task,os.path.join(models_folder_path,'robot','myrobot.model.yaml'),
                    os.path.join(flatland_local_planner_drl_folder_path,'configs','default_settings.yaml'),True,
                  )
model = A2C('MlpPolicy', env, verbose=1)
import time

s = time.time()
model.learn(total_timesteps=3000)
print("steps per second: {}".format(1000/(time.time()-s)))
# obs = env.reset()
# for i in range(1000):
#     action, _state = model.predict(obs, deterministic=True)
#     obs, reward, done, info = env.step(action)
#     env.render()
#     if done:
#       obs = env.reset()

