#!/usr/bin/env python

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


# algorithm slef dependencies
import os
import numpy as np
import rospy
import torch
import torch.nn as nn
from mpi4py import MPI

from torch.optim import Adam
from collections import deque
import time

from model.net import MLPPolicy, CNNPolicy
from circle_world import StageWorld
from model.ppo import calculate_action_batch, transform_buffer

# for stage world
from sensor_msgs.msg import LaserScan


class NN_tb3_training():
    def __init__(self, env, policy, action_bound, OBS_SIZE, index, num_env):
        self.beam_mum = OBS_SIZE
        self.laser_cb_num = 0
        self.scan = None
        self.env = env
        self.env.index = 0

        self.policy = policy
        self.action_bound = action_bound

        ns_prefix = "sim_"
        self.num_envs = rospy.get_param("/num_envs")

        # subgoals
        self.sub_goal_list = [[0, 0] for _ in range(self.num_envs)]

        # pose, x,y,psi
        self.pose_list = [[0, 0, 0] for _ in range(self.num_envs)]
        # velocity linear angular
        self.velocity_list = [[0, 0] for _ in range(self.num_envs)]
        # hard coded
        self.scan_list = [np.zeros(360).tolist()]*self.num_envs

        # publishers
        self.pub_twist_list = []
        self.sub_pose_list = []
        self.sub_subgoal_list = []
        self.laser_sub_list = []
        for i in range(self.num_envs):
            self.pub_twist_list.append(rospy.Publisher(
                f'{ns_prefix}{i+1}/cmd_vel', Twist, queue_size=1))

            self.sub_pose_list.append(rospy.Subscriber(
                f'{ns_prefix}{i+1}/odom', Odometry, partial(self.cbPose, i)))
            self.sub_subgoal_list.append(rospy.Subscriber(
                f'{ns_prefix}{i+1}/waypoint', PoseStamped, partial(self.cbSubGoal, i)))
            self.laser_sub_list = rospy.Subscriber(
                f'{ns_prefix}{i+1}/scan', LaserScan, partial(self.cbLaserscan, i))

    def cbSubGoal(self, env_index, msg):
        self.sub_goal_list[env_index] = [
            msg.pose.position.x, msg.pose.position.y]

    def cbPose(self, env_index, msg):

        self.velocity_list[env_index] = [
            msg.twist.twist.linear.x, msg.twist.twist.angular.z]

        q = msg.pose.pose.orientation
        psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y),
                         1-2*(q.y*q.y+q.z*q.z))  # bounded by [-pi, pi]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose_list[env_index] = [x, y, psi]

    def cbLaserscan(self, env_index, scan):
        self.scan_list[env_index] = scan.ranges

    def process_observation(self):
        scans = np.array(self.scan_list)
        # adapt scan info when min and max angel equal to [-1.57,4.69] (rlca is [-3.14,3.14])
        scans = np.roll(scans, 90)
        scans = np.nan_to_num(scans, nan=3.5, posinf=3.5, neginf=3.5)
        sparse_beam_num = self.beam_mum

        x = np.linspace(0, 1, sparse_beam_num)
        xp = np.linspace(0, 1, 360)

        scan_sparse = np.array([np.interp(x, xp, scans[i])
                                for i in range(scans.shape[0])])

        return scan_sparse / 6.0 - 0.5

    def control_vel(self, env_idx, action):
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
            obs, goals_in_robots_frame, velocities, self.policy, self.action_bound)
        actions[:, 0] = 0.3*actions[:, 0]   # the maximum speed of cmd_vel 0.3
        for i in range(self.num_envs):
            self.control_vel(i, actions[i])


def run():

    # Set parameters of env
    LASER_HIST = 3
    NUM_ENV = 1    # the number of agents in the environment
    OBS_SIZE = 512  # number of leaserbeam
    action_bound = [[0, -1], [1, 1]]    # the limitation of velocity

    # Set env and agent policy
    # index is useful for parallel programming, 0 is for the first agent
    env = StageWorld(OBS_SIZE, index=0, num_env=NUM_ENV)
    trained_model_file = os.path.dirname(__file__) + '/policy/stage2.pth'
    policy = CNNPolicy(frames=LASER_HIST, action_space=2)
    policy.cpu()    # policy.cuda() for gpu
    # torch.load(trained_model_file) for gpu
    state_dict = torch.load(
        trained_model_file, map_location=torch.device('cpu'))
    policy.load_state_dict(state_dict)

    rospy.init_node('rl_collision_avoidance_tb3', anonymous=False)
    print('==================================\nrl_collision_avoidance node started')

    nn_tb3 = NN_tb3_training(env, policy, action_bound,
                             OBS_SIZE, index=0, num_env=NUM_ENV)






    rate = 100
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

