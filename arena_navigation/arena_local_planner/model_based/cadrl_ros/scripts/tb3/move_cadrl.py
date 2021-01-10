#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float32, ColorRGBA, Int32
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point
from ford_msgs.msg import PedTrajVec, NNActions, PlannerMode, Clusters
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import numpy.matlib
import pickle
from matplotlib import cm
import matplotlib.pyplot as plt
import copy
import os
import time
import random
import math

import rospkg

import network
import agent
import util

from nav_msgs.msg import Odometry



class NN_tb3:
    def __init__(self, nn, actions):
        # tb3 -------------------------------------
        # radius 
        self.radius = 0.5
        # goal
        self.global_goal = PoseStamped()
        self.sub_goal = PoseStamped()
        self.pref_speed = 0.14
        #pose 
        self.pose = PoseStamped().pose
        # heading_angle
        self.psi = 0.0
        #vel
        self.vel = Vector3()
        self.t_past = 0
        # 
        self.new_global_goal_received = False 


        # obstacles
        self.other_agents_state = []

        self.possible_actions = network.Actions()
        self.num_actions = self.possible_actions.num_actions
        # load nn
        self.nn = nn
        self.actions = actions
        
        # subscribe
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.updatePose)
        self.sub_global_goal = rospy.Subscriber('/move_base_simple/goal',PoseStamped, self.updateGlobalGoal)

        # publisher
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        
        # print self.global_goal
        # self.global_goal.pose = self.pose

        # loop
        self.updateOtherAgents()
        self.nn_timer = rospy.Timer(rospy.Duration(0.1),self.cbComputeAction)


    # callback functions:

    # goal rviz
    def updateGlobalGoal(self,msg):

        self.new_global_goal_received = True
        self.global_goal = msg
        #self.operation_mode.mode = self.operation_mode.SPIN_IN_PLACE

        #self.goal.pose.position.x = msg.pose.position.x
        #self.goal.pose.position.y = msg.pose.position.y
        #self.goal.header = msg.header
        #self.new_subgoal_received = True
        goal_x = self.global_goal.pose.position.x
        goal_y = self.global_goal.pose.position.y
        goal_z = self.global_goal.pose.orientation.z
        print '\n==========\nnew goal: '+'['+str(goal_x)+', '+str(goal_y)+', '+str(goal_z)+']'
        #print self.global_goal.pose.position


    # goal reched ?
    def goalReached(self, tol,x,y,goal_x,goal_y):
        
        print "error: dx="+str(abs(x-goal_x))+", dy="+str(abs(y-goal_y))
        if abs(x-goal_x) < tol and abs(y-goal_y) < tol:
            self.new_global_goal_received = False
            self.tb3Move([0,0])  
        elif self.global_goal.pose.position.x != 0:
            self.new_global_goal_received = True  
            


    # update pose
    def updatePose(self,msg):
        # update vxy
        self.updateVel(msg)
        self.pose = msg.pose.pose
        # psi from [-pi, pi]
        q = self.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) 
        #print(self.pose)
        

    # update vel
    def updateVel(self, msg):
        self.vel = msg.twist.twist.linear
        # print self.vel

        


    # move (publish to cmd_vel)
    def tb3Move(self,action):
        
        twist = Twist()
        twist.linear.x = action[0]
        twist.angular.z = action[1]

        self.pub_twist.publish(twist)
        print('perform: '+str(action))
        # print type()
     



    def cbComputeAction(self,event):

        tol = 1e-1
        x = self.pose.position.x 
        y = self.pose.position.y

        goal_x = self.global_goal.pose.position.x 
        goal_y = self.global_goal.pose.position.y

        self.goalReached(tol,x,y,goal_x,goal_y)
        if self.new_global_goal_received:
            #tb3 state

            v_x = self.vel.x 
            v_y = self.vel.y

            heading_angle = self.psi

            host_agent = agent.Agent(x, y, goal_x, goal_y, self.radius, self.pref_speed, heading_angle, 0)
            host_agent.vel_global_frame = np.array([v_x, v_y])

            
            # Convert agent states into observation vector
            obs = host_agent.observe(self.other_agents_state)[1:]
            obs = np.expand_dims(obs, axis=0)

            predictions = self.nn.predict_p(obs)[0]
            raw_action = self.possible_actions.actions[np.argmax(predictions)]
            #action = np.array([host_agent.pref_speed*raw_action[0], util.wrap(raw_action[1] + host_agent.heading_global_frame)])
            action = np.array([self.pref_speed*raw_action[0], util.wrap(raw_action[1] + self.psi)])
            self.tb3Move(action)
            

        #print host_agent

    def updateOtherAgents(self):
        other_agents_x = [-1,-2,-3]
        other_agents_y = [2,3,4]
        other_agents_r = [0.5, 0.4, 0.3]
        other_agents_vx = [1.0, 0.6, 0.2]
        other_agents_vy = [0.0, 0.6, 0.8]
        num_other_agents = len(other_agents_x)

        goal_x = self.global_goal.pose.position.x 
        goal_y = self.global_goal.pose.position.y

        self.other_agents_state = []
        for i in range(num_other_agents):
            x = other_agents_x[i]; y = other_agents_y[i]
            v_x = other_agents_vx[i]; v_y = other_agents_vy[i]
            radius = other_agents_r[i]
            
            other_agent = agent.Agent(x, y, goal_x, goal_y, radius=radius, id=i+1)
            other_agent.vel_global_frame = np.array([v_x, v_y])
            self.other_agents_state.append(other_agent)
        print 'agents loaded'




# -----------------------------------------------------------------------------------------------


# run tb3 nn class
def run():

    rospack = rospkg.RosPack()
    a = network.Actions()
    actions = a.actions
    num_actions = a.num_actions
    nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
    nn.simple_load(rospack.get_path('cadrl_ros')+'/checkpoints/network_01900000')

    rospy.init_node('cadrl_tb3')
    tb3_nav = NN_tb3(nn,actions)
    rospy.spin()

if __name__ == '__main__':
    #run_once()
    run()
