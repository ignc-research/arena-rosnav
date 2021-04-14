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

def pretraining(dataset_length:int, episodes:int, policy_storage_addr:str):
    '''
    run pretraining and save model as file in given address
    '''
    task = get_predefined_task()
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')
    arena_local_planner_drl_folder_path = rospkg.RosPack().get_path('arena_local_planner_drl')
    rospy.init_node('il_pretraining')
    env = ILStateCollectorEnv(task,os.path.join(models_folder_path,'robot','myrobot.model.yaml'),
                    os.path.join(arena_local_planner_drl_folder_path,'configs','default_settings.yaml'),True,
                    )
    model = pretrainPPO('MlpPolicy', env, verbose=1, dataset_length=dataset_length)
    #run supervised learning
    model.pretrain(1e-5, model.batch_size, policy_storage_addr, cuda = 1, epi= episodes)

def run_pretrained_model(model_path):
    '''
    run pretrained model and return the reward regarding drl reward functions
    '''
    task = get_predefined_task()
    models_folder_path = rospkg.RosPack().get_path('simulator_setup')
    arena_local_planner_drl_folder_path = rospkg.RosPack().get_path('arena_local_planner_drl')
    rospy.init_node('il_pretraining')
    env = ILStateCollectorEnv(task,os.path.join(models_folder_path,'robot','myrobot.model.yaml'),
                    os.path.join(arena_local_planner_drl_folder_path,'configs','default_settings.yaml'),True,
                    )
    model = pretrainPPO('MlpPolicy', env, verbose=1)
    # load pretrained weights
    model.policy = th.load(model_path)
    model.policy.eval()
    # cmd_vel publisher
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    dead_loop = 0
    while not rospy.is_shutdown():
        obs, _ = env.observation_collector.get_observations()
        dead_loop += 1
        if np.linalg.norm(obs[-2:]-obs[-5:-3]) < 1 or dead_loop > 500:
            env.reset()
            dead_loop = 0

        obs = np.stack([obs for _ in range(64)], axis=0)
        obs = th.from_numpy(obs).float().to('cuda')
        action, _, _, _ = model.policy(obs)
        action = action.cpu().numpy()
        # decide action by Majority voting
        values, counts = np.unique(action, return_counts=True)
        ind = np.argmax(counts)
        action = values[ind]
        linear,angular = action_mapping(action)
        action = Twist()
        action.linear.x = linear
        action.angular.z = angular
        cmd_pub.publish(action)

def evaluate_policies():
    pass

if __name__ == '__main__':
    addr = '/home/jiayun//catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_il/data/pretrained_policy'
    #pretraining(20000, 3000000, addr)
<<<<<<< HEAD
    run_pretrained_model(addr+'/60.pth')
=======
    run_pretrained_model(addr+'/80.pth')
>>>>>>> 547891d8aa7853d32753ad63a631ac8a8c64436c


    # TODO find a way to evaluate policy by using drl reward functions.
    # TODO could switch to DRL
    #model.learn(total_timesteps=100000)
    #print("steps per second: {}".format(1000/(time.time()-s)))
