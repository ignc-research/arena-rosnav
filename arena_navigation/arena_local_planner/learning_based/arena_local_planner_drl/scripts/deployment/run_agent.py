import os
import rospy
import rospkg
import json
import numpy as np
import time

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv

from task_generator.task_generator.tasks import get_predefined_task
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.envs.flatland_gym_env import FlatlandEnv
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.argsparser import parse_run_agent_args
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.tools.train_agent_utils import *


if __name__ == "__main__":
    args, _ = parse_run_agent_args()

    rospy.init_node("run_node")

    # get paths
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    PATHS={
        'model': os.path.join(dir, 'agents', args.load),
        'robot_setting' : os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml'),
        'robot_as' : os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs', 'default_settings.yaml'),
        'scenerios_json_path' : os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'scenerios', 'scenario1.json')
    }

    assert os.path.isfile(
        os.path.join(PATHS['model'], "best_model.zip")), "No model file found in %s" % PATHS['model']

    # initialize hyperparams
    params = load_hyperparameters_json(agent_hyperparams, PATHS)

    print("START RUNNING AGENT:    %s" % params['agent_name'])
    print_hyperparameters(params)

    # initialize task manager
    task_manager = get_predefined_task(mode='ScenerioTask', PATHS=PATHS)
    # initialize gym env
    env = DummyVecEnv([lambda: FlatlandEnv(task_manager, PATHS.get('robot_setting'), PATHS.get('robot_as'), params['reward_fnc'], params['discrete_action_space'], goal_radius=1.00, max_steps_per_episode=350)])
    # load agent
    agent = PPO.load(os.path.join(PATHS['model'], "best_model.zip"), env)

    env.reset()
    
    # send action 'stand still' in order to get first obs
    if params['discrete_action_space']:
        obs, rewards, dones, info = env.step(len(env.action_space)-1)
    else:
        obs, rewards, dones, info = env.step([0, 0])

    # iterate through each scenario max_repeat times
    while True:
        action, _ = agent.predict(obs)

        # clip action
        if not params['discrete_action_space']:
            action = np.maximum(np.minimum(agent.action_space.high, action), agent.action_space.low)
        
        # apply action
        obs, rewards, done, info = env.step(action)

        cum_reward = 0.0
        cum_reward += rewards
        
        if args.verbose == '1':
            if done:
                if info['done_reason'] == 0:
                    done_reason = "exceeded max steps"
                elif info['done_reason'] == 1:
                    done_reason = "collision"
                else:
                    done_reason = "goal reached"
                
                print("Episode finished with reward of %f (finish reason: %s)"% (cum_reward, done_reason))
                cum_reward = 0

        time.sleep(0.0001)
        if rospy.is_shutdown():
            print('shutdown')
            break
