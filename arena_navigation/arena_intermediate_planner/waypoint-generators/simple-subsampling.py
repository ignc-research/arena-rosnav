#!/usr/bin/env python
# ros
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Pose2D
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
# viz
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import math
from torch.nn.utils.rnn import pack_sequence
import torch
import numpy as np

# for transformations
from tf.transformations import *

class StateProvider():
    """A class that provides preprocessed state information for everyone to access.
    Currently, following information are provided 
        - robot position [PoseStamped]
        - goal position  [PoseStamped]
        - distance robot to goal [Float]
        - distance robot to subgoal
        - global plan [Path]
    """
    def __init__(self):

        self.list_of_lists = [[-0.3,11.5], [3, 10.7], [8.78, 9.78], [12.2, 9.1], [16.2, 8.3], [18.42, 6.96], [20.77, 5.47], [23, 0.95]]
        # 
        self._globalGoal = PoseStamped()
        self._globalPlan = Path()
        self._subgoal = PoseStamped()
        self._robot_pose = PoseStamped()

        #provide distance and angle to goal inside pose2D messages (x for distance, theta for the angle)
        self.distance2GlobalGoal = Pose2D()
        self.distance2subgoal = Pose2D()
        self._subgoal.header.frame_id = "map"
        self._subgoal.pose.orientation.w = 1

        self.firstTime = 0
        self.i = 0
        # subs
        self._robot_state_sub = rospy.Subscriber('/odom', Odometry, self.cbRobotPosition)
        self.globalGoal_sub = rospy.Subscriber('/goal', PoseStamped, self.cbGlobalGoal)
        
        # pubs

        self.subgoal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        

        self.distance2sub_pub = rospy.Publisher('/state_provider/distance2sub', Pose2D, queue_size=1)

    def cbRobotPosition(self,msg):
        self._robot_pose.pose.position.x = msg.pose.pose.position.x
        self._robot_pose.pose.position.y = msg.pose.pose.position.y
        self._robot_pose.pose.orientation = msg.pose.pose.orientation
        self.subgoal_pub.publish(self._subgoal)
        self.i = 0
        if self.i == 0:
            self._subgoal.header.frame_id = "map"
            self._subgoal.pose.orientation.w = 1
            self._subgoal.pose.position.x = self.list_of_lists[0][0]
            self._subgoal.pose.position.y = self.list_of_lists[0][1]
            self.subgoal_pub.publish(self._subgoal)
            print(self._subgoal)
            self.i += 1

        self.distance2subgoal.x = self.provideDistance2Goal(self._subgoal)[0]                 #distance to goal
        
    

        # publish subgoals 
        
        
        
        while (self.i < 7):
            if self.distance2subgoal.x < 0.6:
                self._subgoal.header.frame_id = "map"
                self._subgoal.pose.orientation.w = 1
                self._subgoal.pose.position.x = self.list_of_lists[i][0]
                self._subgoal.pose.position.y = self.list_of_lists[i][1]
                self.subgoal_pub.publish(self._subgoal)
                print("test2")
                self.i += 1
        
        if self.distance2GlobalGoal.x < 0.4:
            self.i = 0
      


    def cbSubGoal(self,msg):
        self._subgoal = msg
        self.distance2subgoal.x = self.provideDistance2Goal(msg)[0]                 #distance to goal
        self.distance2subgoal.theta = self.provideDistance2Goal(msg)[1]             #angle to goal
        self.distance2sub_pub.publish(self.distance2subgoal)                        #publish distance to topic /state_provider/distance2global

    def cbGlobalGoal(self,msg):
        self._globalGoal = msg
        self.distance2GlobalGoal.x = self.provideDistance2Goal(msg)[0]              #distance to goal
        self.distance2GlobalGoal.theta = self.provideDistance2Goal(msg)[1]  
        self.subgoal_pub.publish(self._subgoal)        #angle to goal
                    #publish distance to topic /state_provider/distance2global

    def cbglobalPlan(self,msg):
        self._globalPlan = msg
        self.globalPlan_pub.publish(self._globalPlan)
 


    
    def provideDistance2Goal(self, goal):
        robotdistance2d = self.pose3D_to_pose2D(self._robot_pose.pose)
        goal = self.pose3D_to_pose2D(goal.pose)
        return self._calc_distance(goal,robotdistance2d)
        
    @staticmethod
    def _calc_distance(goal_pos:Pose2D,robot_pos:Pose2D):
         y_relative = goal_pos.y - robot_pos.y
         x_relative = goal_pos.x - robot_pos.x
        #  d=np.array[x_relative,y_relative]
        #  dist = np.linalg.norm(d)
         rho =  (x_relative**2+y_relative**2)**0.5
         theta = (np.arctan2(y_relative,x_relative)-robot_pos.theta+4*np.pi)%(2*np.pi)-np.pi
         return rho,theta

    @staticmethod #static methods so can be used without instantiating this class, helpful for others
    def pose3D_to_pose2D(pose3d):
        pose2d=Pose2D()
        pose2d.x=pose3d.position.x
        pose2d.y=pose3d.position.y
        quaternion=(pose3d.orientation.x,pose3d.orientation.y,pose3d.orientation.z,pose3d.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta=yaw
        return pose2d


    def goalReached(self, distance:Pose2D):
        # how far away from goal?
        if distance.x > 0.3:
            return False
        else:
            return True
    

def run():

    rospy.init_node('simple_wp_provider',anonymous=False)

    print('==================================\nStateProvider Node Started\n==================================')

    state_provider = StateProvider()


    rospy.spin()

if __name__ == '__main__':
    run()