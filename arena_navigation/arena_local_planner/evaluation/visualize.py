#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from ford_msgs.msg import Clusters
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import copy
import os
import time
import math

import rospkg
from nav_msgs.msg import Odometry, Path
# custom classes
# from moveBase_sendGoal import newGoal


class viz():
    def __init__(self):

        # rosnode.get_node_names()
        # canon 
        self.node_name = rospy.get_name()
        self.num_poses = 0

        # for publishers
        self.global_goal = PoseStamped()
        self.goal = PoseStamped()
        self.goal.pose.position.x = 0
        self.goal.pose.position.y = 0

        # visualization
        self.path_marker = Marker()

        # Clusters
        self.current_clusters = Clusters()

        # publishers
        # self.pub_agent_marker = rospy.Publisher('~agent_marker',Marker,queue_size=1)
        # self.pub_agent_markers = rospy.Publisher('~agent_markers',MarkerArray,queue_size=1)
        self.pub_path_marker = rospy.Publisher('/visualizer/path',Marker,queue_size=1)
        # self.pub_goal_path_marker = rospy.Publisher('/visualizer/path',Marker,queue_size=1)
        # sub
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        # self.sub_global_goal = rospy.Subscriber('~goal',PoseStamped, self.cbGlobalGoal)
        # self.sub_subgoal = rospy.Subscriber('~subgoal',PoseStamped, self.cbSubGoal)
        
    def on_shutdown(self):
        return

    def pubGoal(self,msg):
        goal = msg
        goal.header.frame_id = "map"
        self.pub_simple_goal.publish(msg)
        print msg

    def cbPose(self, msg):
        # self.sendGoals.updateVel(msg)
        self.num_poses += 1
        self.pose = msg.pose
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.visualize_path()

    def visualize_path(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'pose'
        marker.id = 0
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.pose.position = self.pose.pose.position
        marker.pose.orientation = self.pose.pose.orientation
        marker.scale = Vector3(x=1,y=0.15,z=0.0)
        marker.color = ColorRGBA(b=0.0,g=0,r=0,a=1)
        marker.lifetime = rospy.Duration(60)
        self.pub_path_marker.publish(marker)

        # Display BLUE DOT at NN desired position
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'path_trail'
        marker.id = self.num_poses
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = copy.deepcopy(self.pose.pose.position)
        marker.scale = Vector3(x=0.2,y=0.2,z=0.2)
        marker.color = ColorRGBA(g=0.0,r=0,b=1.0,a=0.3)
        marker.lifetime = rospy.Duration(60)
        self.pub_path_marker.publish(marker)
        # print marker

def run():
    print 'hello world from rviz.py'
    rospy.init_node('Rvisualizer')
    rate = rospy.Rate(10)
    visualize = viz()
    rospy.on_shutdown(visualize.on_shutdown)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    run()