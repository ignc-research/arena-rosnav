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
import subprocess


rospy.init_node("record_rollouts", disable_signals=True)

parser = argparse.ArgumentParser(description='.')
parser.add_argument('-o', '--outputformat', type=str, help='choose output format: "h5" or "npz"', default='npz')
parser.add_argument('-s', '--scenario', type=str, metavar="[scenario name]", default='/home/michael/catkin_ws/src/arena-rosnav/simulator_setup/scenerios/obstacle_map1_obs20.json', help='path of scenario json file for deployment')
args = parser.parse_args()

task = get_predefined_task(mode="ScenerioTask", PATHS={"scenerios_json_path": args.scenario})  # json scenario
#task = get_predefined_task(mode="random", PATHS={"scenerios_json_path": args.scenario})  # random scenario
#task = get_predefined_task(mode="staged", start_stage = 1, PATHS={"scenerios_json_path": args.scenario, "curriculum": "/home/michael/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/configs/training_curriculum.yaml"})  # staged scenarios

models_folder_path = rospkg.RosPack().get_path('simulator_setup')
arena_local_planner_drl_folder_path = rospkg.RosPack().get_path(
    'arena_local_planner_drl')

env = FlatlandEnv(task, os.path.join(models_folder_path, 'robot', 'myrobot.model.yaml'),
                  os.path.join(arena_local_planner_drl_folder_path,
                               'configs', 'default_settings.yaml'), "rule_00", False,
                  )

def clear_costmaps():
        bashCommand = "rosservice call /move_base/clear_costmaps"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        #self._service_clear_client()
        return output, error

obs = env.reset()
observations = []
actions = []
while(True):
    merged_obs, obs_dict, action = env.observation_collector.get_observations_and_action()

    reward, reward_info = env.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'])
    
    done, info = env.check_if_done(reward_info)
    if done:
        if info['done_reason'] == 1:
            # if the episode is done because the robot collided with an obstacle, ignore this episode
            # reduce repeat count by 1 and start again
            print('collision')
            task._num_repeats_curr_scene -= 1
            #time.sleep(1.0)
            #task.next_stage()
            clear_costmaps()
            env.reset()
            clear_costmaps()
            #time.sleep(2.0)
        else:
            observations.append(merged_obs)
            actions.append(action)
            # try resetting the environment: this will either reset the obstacles and robot and start another episode for recording
            # or it will end the recording because all scenarios have been run their maximum number of times
            try:
                #task.next_stage()
                env.reset()
            except Exception as e:
                print(e)
                print('All scenarios have been evaluated!')
                break
    else:
        # if the episode is not done, save this timesteps's observations and actions to the arrays and continue the episode
        observations.append(merged_obs)
        actions.append(action)


# save rollouts
if args.outputformat == 'h5':
    date_str = datetime.now().strftime('%Y%m%d_%H-%M')
        
    file_action = h5py.File(f'./output/{date_str}_action.hdf5', "w")
    file_state = h5py.File(f'./output/{date_str}_state.hdf5', "w")
    
    file_action.create_dataset("actions", data=np.array(actions))
    file_state.create_dataset("states", data=np.array(observations))

    file_state.close()
    file_action.close()
    rospy.loginfo("h5py writer have been shut down.")

if args.outputformat == 'npz':
    # save observations and actions in an npz file:
    date_str = datetime.now().strftime('%Y%m%d_%H-%M')
    path = str(f'./output/rollout_{date_str}')
    print(f'Saving rollouts to {path}')
    np.savez_compressed(
        path,
        observations = np.array(observations),
        actions = np.array(actions)
    )
    print('Done saving rollouts')
