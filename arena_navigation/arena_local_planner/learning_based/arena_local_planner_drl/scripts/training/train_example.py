import time
import os
import sys
from stable_baselines3 import A2C
from rl_agent.envs.flatland_gym_env import FlatlandEnv
from task_generator.tasks import get_predefined_task
import rospy
import rospkg



rospy.init_node("test", disable_signals=True)
# task = get_predefined_task()
task = get_predefined_task(ns="sim_2", mode="random", PATHS={
                           "scenerios_json_path": "/home/joe/ssd/projects/arena-rosnav-ws/src/arena-rosnav/simulator_setup/scenerios/example_scenerio.json"})
            
models_folder_path = rospkg.RosPack().get_path('simulator_setup')
arena_local_planner_drl_folder_path = rospkg.RosPack().get_path(
    'arena_local_planner_drl')


env = FlatlandEnv("sim_2", task, os.path.join(models_folder_path, 'robot', 'myrobot.model.yaml'),
                  os.path.join(arena_local_planner_drl_folder_path,
                               'configs', 'default_settings.yaml'), "rule_00", True,
                  )
model = A2C('MlpPolicy', env, verbose=1)

s = time.time()
try:
    model.learn(total_timesteps=1000)
except KeyboardInterrupt:
    try:
        sys.exit(0)
    except SystemExit:
        os._exit(0)

print("steps per second: {}".format(1000/(time.time()-s)))
# obs = env.reset()
# for i in range(1000):
#     action, _state = model.predict(obs, deterministic=True)
#     obs, reward, done, info = env.step(action)
#     env.render()
#     if done:
#       obs = env.reset()
