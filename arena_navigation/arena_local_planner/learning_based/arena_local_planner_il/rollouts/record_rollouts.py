import argparse
import time
from datetime import datetime
import os
import sys
from stable_baselines3 import A2C
from rl_agent.envs.flatland_gym_env import FlatlandEnv
from task_generator.tasks import get_predefined_task
import rospy
import rospkg
import numpy as np
from collections import OrderedDict
import h5py

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-o', '--outputformat', type=str, help='choose output format: "h5" or "npz"', default='h5')
parser.add_argument('-s', '--scenario', type=str, metavar="[scenario name]", default='/home/michael/catkin_ws/src/arena-rosnav/simulator_setup/scenerios/obstacle_map1_obs20.json', help='path of scenario json file for deployment')
args = parser.parse_args()

rospy.init_node("record_rollouts")

task = get_predefined_task(mode="ScenerioTask", PATHS={"scenerios_json_path": args.scenario})
models_folder_path = rospkg.RosPack().get_path('simulator_setup')
arena_local_planner_drl_folder_path = rospkg.RosPack().get_path(
    'arena_local_planner_drl')


env = FlatlandEnv(task, os.path.join(models_folder_path, 'robot', 'myrobot.model.yaml'),
                  os.path.join(arena_local_planner_drl_folder_path,
                               'configs', 'default_settings.yaml'), "rule_00", False,
                  )

obs = env.reset()
#TODO end recording loop with keypress or by ending when the scenario has been repeated the correct number of times
observations = []
actions = []
for i in range(20000):
    print(i)
    merged_obs, obs_dict, action = env.observation_collector.get_observations_and_action()
    observations.append(merged_obs)
    actions.append(action)
    if env.check_if_done(merged_obs, obs_dict):
        env.reset()

# save rollouts
if args.outputformat == 'h5':
    date_str = datetime.now().strftime('%Y%m%d_%H-%M')
    actions_dict, states_dict = OrderedDict(), OrderedDict()

    for i in range(len(actions)):
        actions_dict[str(i)] = actions[i]
        states_dict[str(i)] = observations[i]
    
    file_action = h5py.File(f'./output/{date_str}_action.hdf5', "w")
    file_state = h5py.File(f'./output/{date_str}_state.hdf5', "w")

    file_action.create_dataset(str(actions_dict[0]), data=np.array(actions_dict))  #?
    file_state.create_dataset(str(states_dict[0]), data=np.array(states_dict[1]))  #?

    file_state.close()
    file_action.close()
    rospy.loginfo("h5py writer have been shut down.")

if args.outputformat == 'npz':
    # save observations and actions in an npz file:
    date_str = datetime.now().strftime('%Y%m%d_%H-%M')
    path = str(f'./output/rollout_{date_str}')
    np.savez_compressed(
        path,
        observations = np.array(observations),
        actions = np.array(actions)
    )

# How should the obstacles appear?
#TODO need to reset task (either directly through task.reset() or indirectly through Flatlandenv.reset() to return robot and obstacles
# to starting position)
#TODO need to add check that goal has been reached e.g. by overriding flatlandenv's step function or (simpler) by 
# adding a function here that extracts goal in robot frame from obs and checks that it is within a tolerance range
#TODO could also use get_reward to check for collisions or cases where the robot gets stuck

#use env.reset()
#and 
# calculate reward
#reward, reward_info = self.reward_calculator.get_reward(
#    obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'])
#done = reward_info['is_done']
#
#print("reward:  {}".format(reward))
#
## info
#info = {}
#if done:
#   info['done_reason'] = reward_info['done_reason']
#else:
#if self._steps_curr_episode == self._max_steps_per_episode:
#   done = True
#   info['done_reason'] = 0

#obs = env.reset()
