from il_state_collector import ILStateCollectorEnv
from task_generator.tasks import get_predefined_task
from pre_training_policies import pretrainPPO
from geometry_msgs.msg import Twist
import rospkg
import rospy
import os
import torch as th
import numpy as np

def action_mapping(action:int):
    '''
    used to map action number to Twist info
    '''
    action_dict = {0:(0.8,0.),1:(-0.1,0.),\
                   2:(0.15,0.75),3:(0.15,-0.75),4:(0.,1.5), \
                   5:(0.,-1.5),6:(0.,0.)}
    return action_dict[action]

def pretraining_train(dataset_length:int, pre_episodes:int, learn_step:int, policy_storage_addr:str, pretraining:bool):
    '''
    run pretraining and save model as file in given address
    pretraining: for True pretraining will run first.
    '''
    task = get_predefined_task()
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')
    arena_local_planner_drl_folder_path = rospkg.RosPack().get_path('arena_local_planner_drl')
    rospy.init_node('il_pretraining')

    env = ILStateCollectorEnv(task, os.path.join(models_folder_path,'robot','myrobot.model.yaml'),
                    os.path.join(arena_local_planner_drl_folder_path,'configs','default_settings.yaml'),"rule_00", True, False,
                    max_steps_per_episode=400
                    )
    # learning rate larger than 1e-4 will mess up prertained weight
    model = pretrainPPO('MlpPolicy', env, learning_rate=5e-5, verbose=1, pretraining=pretraining, dataset_length=dataset_length)
    if pretraining:
        #run supervised learning
        model.pretrain(5e-4, 64, policy_storage_addr, epi= pre_episodes) #first two varables:learning rate, batch_size
    else:
        # load the pretrained model weight
        model.policy = th.load(policy_storage_addr + '/20.pth')
    #run drl
    model.learn(total_timesteps=learn_step)

def run_pretrained_model(model_path):
    '''
    run pretrained model and return the reward regarding drl reward functions
    for discrete env
    '''
    task = get_predefined_task()
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')
    arena_local_planner_drl_folder_path = rospkg.RosPack().get_path('arena_local_planner_drl')
    rospy.init_node('il_pretraining')
    env = ILStateCollectorEnv(task,os.path.join(models_folder_path,'robot','myrobot.model.yaml'),
                    os.path.join(arena_local_planner_drl_folder_path,'configs','default_settings.yaml'),"rule_00", False, False,
                    max_steps_per_episode=400
                    )
    model = pretrainPPO('MlpPolicy', env, verbose=1)
    # load pretrained weights
    model.policy = th.load(model_path)
    model.policy.eval()
    # cmd_vel publisher
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    while not rospy.is_shutdown():
        obs, obs_dict = env.observation_collector.get_observations()
        # calculate reward
        reward, reward_info = env.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'])
        done = reward_info['is_done']
        if done:
            env.reset()

        obs = np.stack([obs for _ in range(64)], axis=0)
        obs = th.from_numpy(obs).float().to(model.device)
        action, _, _ = model.policy(obs)
        action = action.detach().cpu().numpy()
        if env._is_action_space_discrete:
            # decide action by Majority voting
            values, counts = np.unique(action, return_counts=True)
            ind = np.argmax(counts)
            action = values[ind]
            linear,angular = action_mapping(action)
        else:
            linear, angular = action[1,:]
            print(action[1,:])
        action = Twist()
        action.linear.x = linear
        action.angular.z = angular
        cmd_pub.publish(action)

def evaluate_policies():
    pass

if __name__ == '__main__':
    addr = '/home/jiayun/git_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_il/data/pretrained_policy'
    ##if using pretraining 30000 pairs need 5 sec to match data##
    pretraining_train(30000, 100000, 10000000, addr, pretraining=True)
    #run_pretrained_model(addr+'/con.pth')
    
    # TODO find a way to evaluate policy by using drl reward functions.

