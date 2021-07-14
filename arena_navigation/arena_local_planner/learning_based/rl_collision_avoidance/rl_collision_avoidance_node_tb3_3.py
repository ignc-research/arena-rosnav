#!/usr/bin/env python

from operator import sub
from torch._C import LockingLogger
import rospy
from rospy.names import reload_mappings
from std_msgs.msg import Float32, ColorRGBA, Int32, String
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point
from ford_msgs.msg import Clusters
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from functools import partial
from multiprocessing import Process, Manager
import copy


# algorithm slef dependencies
import os
import numpy as np
import rospy
import torch
import torch.nn as nn

from torch.optim import Adam
from collections import deque
import time
import sys



curr_dir_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(curr_dir_path)





from model.net import MLPPolicy, CNNPolicy
from circle_world import StageWorld
from model.ppo import calculate_action_batch, transform_buffer

# for stage world
from sensor_msgs.msg import LaserScan


class NN_TB3():
    def __init__(self,env_idx,sub_goal_list,pose_list,velocity_list,scan_list) -> None:
        rospy.init_node(f'rl_collision_avoidance_tb3_parent_process_{env_idx}', anonymous=True)
        self.env_idx = env_idx
        self.sub_goal_list = sub_goal_list
        self.pose_list = pose_list
        self.velocity_list = velocity_list
        self.scan_list = scan_list

        ns_prefix = "sim_"

        sub_pose = rospy.Subscriber(f'{ns_prefix}{env_idx}/odom', Odometry, self.cbPose)
        sub_subgoal = rospy.Subscriber(f'{ns_prefix}˚{env_idx}/waypoint', PoseStamped, self.cbSubGoal)
        sub_laser = rospy.Subscriber(f'{ns_prefix}{env_idx}/scan', LaserScan, self.cbLaserscan)
        
    def cbSubGoal(self,msg):
        self.sub_goal_list[self.env_idx] = [
            msg.pose.position.x, msg.pose.position.y]

    def cbPose(self,msg):

        self.velocity_list[self.env_idx] = [
            msg.twist.twist.linear.x, msg.twist.twist.angular.z]

        q = msg.pose.pose.orientation
        psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y),
                         1-2*(q.y*q.y+q.z*q.z))  # bounded by [-pi, pi]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose_list[self.env_idx] = [x, y, psi]

    def cbLaserscan(self,scan):
        self.scan_list[self.env_idx] = scan.ranges

    def run(self):
        rospy.spin()






class NN_tb3_training():
    def __init__(self,num_envs,model,action_bound,sub_goal_list,pose_list,velocity_list,scan_list):

        self.model = model
        self.action_bound = action_bound

        ns_prefix = "sim_" 
        self.num_envs = num_envs

        # subgoals
        self.sub_goal_list = sub_goal_list

        # pose, x,y,psi
        self.pose_list = pose_list
        # velocity linear angular
        self.velocity_list = velocity_list
        # hard coded
        self.scan_list = scan_list

        # publishers
        self.pub_twist_list = []
        
        for i in range(self.num_envs):
            self.pub_twist_list.append(rospy.Publisher(f'{ns_prefix}{i+1}/cmd_vel', Twist, queue_size=1))

    def process_observation(self):
        scans = np.array(copy.deepcopy(self.scan_list))
        # adapt scan info when min and max angel equal to [-1.57,4.69] (rlca is [-3.14,3.14])
        scans = np.roll(scans, 90)
        scans = np.nan_to_num(scans, nan=3.5, posinf=3.5, neginf=3.5)

        x = np.linspace(0, 1, 512)
        xp = np.linspace(0, 1, 360)

        scan_sparse = np.array([np.interp(x, xp, scans[i])
                                for i in range(scans.shape[0])])

        return scan_sparse / 6.0 - 0.5

    def control_vel(self,env_idx, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.  # it's not necessary
        move_cmd.linear.z = 0.
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        move_cmd.angular.z = action[1]
        self.pub_twist_list[env_idx].publish(move_cmd)

    def get_local_goal(self):

        local_goal = []
        for i in range(len(self.pose_list)):
            goal_x, goal_y = self.sub_goal_list[i]
            x, y, theta = self.pose_list[i]
            local_x = (goal_x - x) * np.cos(theta) + \
                (goal_y - y) * np.sin(theta)
            local_y = -(goal_x - x) * np.sin(theta) + \
                (goal_y - y) * np.cos(theta)
            local_goal.append([local_x, local_y])
        # return subgoal position based on robot
        return np.array(local_goal, dtype='float64')

    def cal_action(self):
        # ************************************ Inpsut ************************************
        obs = self.process_observation()[:, None, :]
        obs = np.repeat(obs, 3, axis=1)
        goals_in_robots_frame = self.get_local_goal()
        velocities = np.array(self.velocity_list, dtype='float64')

        # self.control_pose(state)

        # ************************************ Output ************************************
        actions = calculate_action_batch(
            obs, goals_in_robots_frame, velocities, self.model, self.action_bound)
        actions[:, 0] = 0.3*actions[:, 0]   # the maximum speed of cmd_vel 0.3
        for i in range(self.num_envs):
            self.control_vel(i, actions[i])




def parent_process_run(num_envs,sub_goal_list,pos_list,velocity_list,scan_list):
    rate = 100

    # Set parameters of env
    LASER_HIST = 3
    NUM_ENV = 1    # the number of agents in the environment
    OBS_SIZE = 512  # number of leaserbeam
    action_bound = [[0, -1], [1, 1]]    # the limitation of velocity

    trained_model_file = os.path.dirname(os.path.abspath(__file__)) + '/policy/stage2.pth'
    policy = CNNPolicy(frames=LASER_HIST, action_space=2)
    policy.cpu()    # policy.cuda() for gpu
    # torch.load(trained_model_file) for gpu
    state_dict = torch.load(
        trained_model_file, map_location=torch.device('cpu'))
    policy.load_state_dict(state_dict)

    print('==================================\nrl_collision_avoidance node started')

    nn_tb3 = NN_tb3_training(num_envs=num_envs,model=policy,action_bound=action_bound,sub_goal_list=sub_goal_list,pose_list=pose_list,velocity_list=velocity_list,scan_list=scan_list)

    while not rospy.is_shutdown():
        start_time = time.time()
        nn_tb3.cal_action()
        print(f"Inference time: {time.time()-start_time}")
        end_time = time.time()
        remaining_sleep_time = 1/rate-(end_time-start_time)
        if remaining_sleep_time > 0:
            # print(f"sleep time{remaining_sleep_time}")
            time.sleep(1/rate-(end_time-start_time))
        else:
            rospy.logwarn(
                "rate is set to high! use \"rostopic hz /sim_1/move_base/cmd_vel /clock:=/sim_1/clock\" to check the publishing rate in simulator \"")

def start_child_process(env_idx,sub_goals,pose_list,velocity_list,scan_list):
    print(f"CHILD_{env_idx}")
    nn_tb3_single_node = NN_TB3(env_idx=env_idx,sub_goal_list=sub_goals,pose_list=pose_list,velocity_list=velocity_list,scan_list=scan_list)
    nn_tb3_single_node.run()


if __name__ == '__main__':

    import argparse 
    import sys
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs",type=int)
    args = parser.parse_args([sys.argv[1]]) 
    num_envs = args.num_envs
    
    print(num_envs)

    with Manager() as manager:
        sub_goals = manager.list([[0, 0] for _ in range(num_envs)])
        pose_list = manager.list([[0, 0, 0] for _ in range(num_envs)])

        # velocity linear angular
        velocity_list = manager.list([[0, 0] for _ in range(num_envs)])
        # hard coded
        scan_list = manager.list([np.zeros(360).tolist()]*num_envs)
        processes = [Process(target=start_child_process,args=(env_idx,sub_goals,pose_list,velocity_list,scan_list)) for env_idx in range(1,num_envs+1)]

        for p in processes:
            p.start()
            # Set parameters of env
        rospy.init_node('rl_collision_avoidance_tb3_parent_process', anonymous=False)
        num_envs = rospy.get_param("/num_envs")
        parent_process_run(num_envs,sub_goals,pose_list,velocity_list,scan_list)



        
