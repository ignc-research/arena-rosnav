#!/usr/bin/env python
from ros_nav import RosNav
import rospy
import copy
# crowdnav
import numpy as np
from geometry_msgs.msg import Twist
import math
import configparser
import torch
import gym
# 
from crowd_nav.policy.cadrl import CADRL
from crowd_nav.policy.lstm_rl import LstmRL
from crowd_nav.policy.sarl import SARL
from crowd_sim.envs.utils.robot import Robot

class TestNode():
    def __init__(self, env, env_config, policy):
        self.tb3 = RosNav('/goal','/plan_manager/subgoal')
        self.desired_speed = 0.3
        self.angle2Action = 0.0

        # NN
        self.env = env
        self.env_config = env_config
        # configure robot
        self.robot = Robot(env_config, 'robot')
        self.robot.set_policy(policy)

        self.env.set_robot(self.robot)    #pass robot parameters into env
        self.ob = env.reset('test',1)     #intial some parameters from .config file such as time_step,success_reward for other instances
        self.policy = policy
        self.policy.set_env(env) 

 
        # control loop
        rospy.Timer(rospy.Duration(0.2),self.cbControl)
        rospy.Timer(rospy.Duration(0.01),self.cbComputeActionCrowdNav)
    
    def cbControl(self,event):
        twist = Twist()
        if not self.tb3.goalReached():
            if abs(self.angle2Action) > 0.1 and self.angle2Action > 0:
                twist.angular.z = -0.3
                print("spinning in place +")
            elif abs(self.angle2Action) > 0.1 and self.angle2Action < 0:
                twist.angular.z = 0.3
                print("spinning in place -")
            # else:
            vel = np.array([self.tb3.raw_action[0],self.tb3.raw_action[1]])
            twist.linear.x = 0.1*np.linalg.norm(vel)
            print(twist.linear.x)
        self.tb3.pub_twist.publish(twist)

    def update_angle2Action(self):
        # action vector
        v_a = np.array([self.tb3.raw_action[0], self.tb3.raw_action[1]])
        # pose direction
        phi = self.tb3.angle_pose
        e_dir = np.array([math.cos(phi), math.sin(phi)])
        # angle: <v_a, e_dir>
        self.angle2Action = np.math.atan2(np.linalg.det([v_a,e_dir]),np.dot(v_a,e_dir))

    def cbComputeActionCrowdNav(self,event):
        robot_x = self.tb3.pose.pose.position.x
        robot_y = self.tb3.pose.pose.position.y
        # goal
        goal_x = self.tb3.sub_goal.x
        goal_y = self.tb3.sub_goal.y
        # velocity
        robot_vx = self.tb3.vel.x
        robot_vy = self.tb3.vel.y
        # oriantation
        theta = self.tb3.angle_pose
        robot_radius = 0.3

        # set robot info
        self.robot.set(robot_x, robot_y, goal_x, goal_y, robot_vx, robot_vy, theta, robot_radius)

        obstacle_x = [-6.0,-6.0,-6.0,-6.0,-6.0]
        obstacle_y = [-6.0,-6.0,-6.0,-6.0,-6.0]
        # velocity
        obstacle_vx = [0.0,0.0,0.0,0.0,0.0]
        obstacle_vy = [0.0,0.0,0.0,0.0,0.0]
        obstacle_radius = 0.3

        # initial obstacle instances and set value
        for i in range(self.env_config.getint('sim','human_num')):
            self.env.humans[i].set(obstacle_x[i], obstacle_y[i], goal_x,goal_y, obstacle_vx[i], obstacle_vy[i], theta, obstacle_radius)
            self.ob[i]= self.env.humans[i].get_observable_state()

        # ************************************ Output ************************************
        # get action info
        action = self.robot.act(self.ob)

        self.tb3.update_action(action)
        self.update_angle2Action()


def run():
    
    # start node
    rospy.init_node("crowd_test", anonymous=False)
    # rospy.sleep(0.1) # sometimes node isnt recognized
    print('==================================\ncrowd-node started\n==================================')

    policy_name = "lstm"

    device = 'cpu'
    phase = 'test'

    select_policy = {"cadrl":CADRL(),"lstm":LstmRL(),"sarl":SARL()}
    # the path of training result which contains configs and rl mode
    env_config_file = 'crowd_nav/data/output/env.config'             #path beginging without slash
    policy_config_file = 'crowd_nav/data/output/policy.config'
    model_weights = 'crowd_nav/data/output/rl_model_'+policy_name+'.pth'
    # print(model_weights)
    # select policy
    policy = select_policy[policy_name]     #{SARL(),CADRL(),LstmRL()}
    policy_config = configparser.RawConfigParser()
    policy_config.read(policy_config_file)
    policy.configure(policy_config)
    policy.get_model().load_state_dict(torch.load(model_weights))
    policy.set_device(device)
    policy.set_phase(phase)

    # configure environment / obstacles
    env_config = configparser.RawConfigParser()
    env_config.read(env_config_file)
    env = gym.make('CrowdSim-v0')   #env is inherited from CrowdSim class in crowd_sim.py
    env.configure(env_config)


    test = TestNode(env,env_config,policy)
    rospy.spin()


if __name__ == '__main__':
    run()