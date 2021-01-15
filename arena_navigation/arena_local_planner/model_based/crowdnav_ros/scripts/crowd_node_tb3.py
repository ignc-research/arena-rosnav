#!/usr/bin/env python

import rospy
# import sys
# from std_msgs.msg import Float32, ColorRGBA, Int32, String
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point
from ford_msgs.msg import Clusters
# from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
# import numpy.matlib
# import pickle
# from matplotlib import cm
# import matplotlib.pyplot as plt
# import copy
# import os
# import time
# import random
# import math

# import rospkg
from nav_msgs.msg import Odometry

import configparser
import torch
import gym

from crowd_nav.policy.cadrl import CADRL
from crowd_nav.policy.lstm_rl import LstmRL
from crowd_nav.policy.sarl import SARL
from crowd_sim.envs.utils.robot import Robot


PED_RADIUS = 0.3
# angle_1 - angle_2
# contains direction in range [-3.14, 3.14]
def find_angle_diff(angle_1, angle_2):
    angle_diff_raw = angle_1 - angle_2
    angle_diff = (angle_diff_raw + np.pi) % (2 * np.pi) - np.pi
    return angle_diff

class NN_tb3():
    def __init__(self, env, env_config, policy):

        # 
        self.env = env
        self.env_config = env_config
        # configure robot
        self.robot = Robot(env_config, 'robot')
        self.robot.set_policy(policy)

        self.env.set_robot(self.robot)    #pass robot parameters into env
        self.ob = env.reset('test',1)     #intial some parameters from .config file such as time_step,success_reward for other instances
        self.policy = policy
        self.policy.set_env(env) 
        # for subscribers
        self.pose = PoseStamped()
        self.vel = Vector3()
        self.psi = 0.0

        # for publishers
        self.global_goal = PoseStamped()
        self.goal = PoseStamped()
        self.desired_position = PoseStamped()
        self.desired_action = np.zeros((2,))

        # # publishers
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1) 
        self.pub_pose_marker = rospy.Publisher('',Marker,queue_size=1)
        # self.pub_agent_markers = rospy.Publisher('~agent_markers',MarkerArray,queue_size=1)
        # self.pub_path_marker = rospy.Publisher('~path_marker',Marker,queue_size=1)
        # self.pub_goal_path_marker = rospy.Publisher('~goal_path_marker',Marker,queue_size=1)
        # # sub
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        self.sub_global_goal = rospy.Subscriber('/goal',PoseStamped, self.cbGlobalGoal)
        # self.sub_subgoal = rospy.Subscriber('~subgoal',PoseStamped, self.cbSubGoal)
        
        # subgoals
        self.sub_goal = Vector3()

        # self.sub_clusters = rospy.Subscriber('~clusters',Clusters, self.cbClusters)


        # control timer
        # self.control_timer = rospy.Timer(rospy.Duration(0.01),self.cbControl)
        self.nn_timer = rospy.Timer(rospy.Duration(0.3),self.cbComputeActionCrowdNav)

    def cbGlobalGoal(self,msg):
        self.stop_moving_flag = True
        self.new_global_goal_received = True
        self.global_goal = msg
        self.goal.pose.position.x = msg.pose.position.x
        self.goal.pose.position.y = msg.pose.position.y
        self.goal.header = msg.header

        # reset subgoals
        print("new goal: "+str([self.goal.pose.position.x,self.goal.pose.position.y])) 

    def cbSubGoal(self,msg):
        self.sub_goal.x = msg.pose.position.x
        self.sub_goal.y = msg.pose.position.y
        # print "new subgoal: "+str(self.sub_goal)

    def cbPose(self, msg):
        self.cbVel(msg)
        q = msg.pose.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        self.pose = msg.pose
        # self.visualize_pose(msg.pose.pose.position,msg.pose.pose.orientation)

    def cbVel(self, msg):
        self.vel = msg.twist.twist.linear

    def cbClusters(self,msg):
        other_agents = []

        xs = []; ys = []; radii = []; labels = []
        num_clusters = len(msg.labels)
  
        for i in range(num_clusters):
            index = msg.labels[i]
            x = msg.mean_points[i].x; y = msg.mean_points[i].y
            v_x = msg.velocities[i].x; v_y = msg.velocities[i].y
            radius = self.obst_rad


            xs.append(x); ys.append(y); radii.append(radius); labels.append(index)
            # self.visualize_other_agent(x,y,radius,msg.labels[i])
            # helper fields
            heading_angle = np.arctan2(v_y, v_x)
            pref_speed = np.linalg.norm(np.array([v_x, v_y]))
            goal_x = x + 5.0; goal_y = y + 5.0
            

            if pref_speed < 0.2:
                pref_speed = 0; v_x = 0; v_y = 0
            other_agents.append(agent.Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, index))
        self.visualize_other_agents(xs, ys, radii, labels)
        self.other_agents_state = other_agents

    def stop_moving(self):
        twist = Twist()
        self.pub_twist.publish(twist)

    def update_action(self, action):
        # print 'update action'
        self.desired_action = action
        self.desired_position.pose.position.x = self.pose.pose.position.x + 1*action[0]*np.cos(action[1])
        self.desired_position.pose.position.y = self.pose.pose.position.y + 1*action[0]*np.sin(action[1])

    def cbControl(self, event):

        if self.goal.header.stamp == rospy.Time(0) or self.stop_moving_flag and not self.new_global_goal_received:
            self.stop_moving()
            return
        elif self.operation_mode.mode==self.operation_mode.NN:
            desired_yaw = self.desired_action[1]
            yaw_error = desired_yaw - self.psi
            if abs(yaw_error) > np.pi:
                yaw_error -= np.sign(yaw_error)*2*np.pi

            gain = 1.3 # canon: 2
            vw = gain*yaw_error

            use_d_min = True
            if False: # canon: True
                # use_d_min = True
                # print "vmax:", self.find_vmax(self.d_min,yaw_error)
                vx = min(self.desired_action[0], self.find_vmax(self.d_min,yaw_error))
            else:
                vx = self.desired_action[0]
      
            twist = Twist()
            twist.angular.z = vw
            twist.linear.x = vx
            self.pub_twist.publish(twist)
            self.visualize_action(use_d_min)
            return

        elif self.operation_mode.mode == self.operation_mode.SPIN_IN_PLACE:
            print('Spinning in place.')
            self.stop_moving_flag = False
            angle_to_goal = np.arctan2(self.global_goal.pose.position.y - self.pose.pose.position.y, \
                self.global_goal.pose.position.x - self.pose.pose.position.x) 
            global_yaw_error = self.psi - angle_to_goal
            if abs(global_yaw_error) > 0.5:
                vx = 0.0
                vw = 1.0
                twist = Twist()
                twist.angular.z = vw
                twist.linear.x = vx
                self.pub_twist.publish(twist)
                # print twist
            else:
                print('Done spinning in place')
                self.operation_mode.mode = self.operation_mode.NN
                # self.new_global_goal_received = False
            return
        else:
            self.stop_moving()
            return

    def cbComputeActionCrowdNav(self, event):
        robot_x = self.pose.pose.position.x
        robot_y = self.pose.pose.position.y
        # goal
        goal_x = self.global_goal.pose.position.x
        goal_y = self.global_goal.pose.position.x
        # velocity
        robot_vx = self.vel.x
        robot_vy = self.vel.y
        # oriantation
        theta = self.psi
        robot_radius = 0.3

        # set robot info
        self.robot.set(robot_x, robot_y, goal_x, goal_y, robot_vx, robot_vy, theta, robot_radius)

        # obstacle: position, velocity, radius
        # position
        # obstacle_x = [0.1,0.2,0.3,0.4,0.5]
        # obstacle_y = [0.1,0.2,0.3,0.4,0.5]
        # # velocity
        # obstacle_vx = [0.1,0.2,0.3,0.4,0.5]
        # obstacle_vy = [0.1,0.2,0.3,0.4,0.5]

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
        position = self.robot.get_observable_state()
        print('\n---------\nrobot position (X,Y):', position.position)
        print(action)
        print(theta)
                    


        # self.update_action(action)

    def update_subgoal(self,subgoal):
        self.goal.pose.position.x = subgoal[0]
        self.goal.pose.position.y = subgoal[1]

    def visualize_pose(self,pos,orientation):
        # Yellow Box for Vehicle
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'agent'
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = pos
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.7,y=0.42,z=1)
        marker.color = ColorRGBA(r=1.0,g=1.0,a=1.0)
        marker.lifetime = rospy.Duration(1.0)
        self.pub_pose_marker.publish(marker)

        # Red track for trajectory over time
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'agent'
        marker.id = self.num_poses
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = pos
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.2,y=0.2,z=0.2)
        marker.color = ColorRGBA(r=1.0,a=1.0)
        marker.lifetime = rospy.Duration(10.0)
        self.pub_pose_marker.publish(marker)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down.")
        self.stop_moving()
        rospy.loginfo("Stopped %s's velocity.")

def run():

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
  
    rospy.init_node('crowdnav_tb3',anonymous=False)
    print('==================================\ncrowdnav node started')

    nn_tb3 = NN_tb3(env,env_config,policy)
    rospy.on_shutdown(nn_tb3.on_shutdown)

    rospy.spin()

if __name__ == '__main__':
    run()