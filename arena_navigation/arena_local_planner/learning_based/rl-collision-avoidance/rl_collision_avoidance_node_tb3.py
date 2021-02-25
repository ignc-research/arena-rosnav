#!/usr/bin/env python

import rospy
# import sys
from std_msgs.msg import Float32, ColorRGBA, Int32, String
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point
from ford_msgs.msg import Clusters
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

# algorithm slef dependencies
import os
import numpy as np
import rospy
import torch
import torch.nn as nn
from mpi4py import MPI

from torch.optim import Adam
from collections import deque

from model.net import MLPPolicy, CNNPolicy
from circle_world import StageWorld
from model.ppo import generate_action_no_sampling, transform_buffer

# for stage world
from sensor_msgs.msg import LaserScan
import tf
import copy

# PED_RADIUS = 0.3
# # angle_1 - angle_2
# # contains direction in range [-3.14, 3.14]
# def find_angle_diff(angle_1, angle_2):
#     angle_diff_raw = angle_1 - angle_2
#     angle_diff = (angle_diff_raw + np.pi) % (2 * np.pi) - np.pi
#     return angle_diff

class NN_tb3():
    def __init__(self, policy, action_bound, OBS_SIZE, index, num_env):
        self.beam_mum = OBS_SIZE
        self.laser_cb_num = 0
        self.scan = None
        self.env={"index": 0}
  
        self.policy = policy
        self.action_bound = action_bound
        # for subscribers
        self.pose = PoseStamped()
        self.vel = Vector3()
        self.psi = 0.0

        # for publishers
        self.global_goal = PoseStamped()
        self.goal = PoseStamped()
        # self.desired_position = PoseStamped() #used for update_action()
        # self.desired_action = np.zeros((2,))

        # # publishers
        self.pub_twist = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        # self.pub_pose_marker = rospy.Publisher('',Marker,queue_size=1)  #used for visualize_pose()
        # self.pub_agent_markers = rospy.Publisher('~agent_markers',MarkerArray,queue_size=1)
        # self.pub_path_marker = rospy.Publisher('~path_marker',Marker,queue_size=1)
        # self.pub_goal_path_marker = rospy.Publisher('~goal_path_marker',Marker,queue_size=1)
        # # sub
        self.sub_pose = rospy.Subscriber('/odom',Odometry,self.cbPose)
        # self.sub_global_goal = rospy.Subscriber('/goal',PoseStamped, self.cbGlobalGoal)
        self.sub_subgoal = rospy.Subscriber('/plan_manager/subgoal',PoseStamped, self.cbSubGoal)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback)
        
        # subgoals
        self.sub_goal = Vector3()   # to calculate the distace between robot and goal

        # self.sub_clusters = rospy.Subscriber('~clusters',Clusters, self.cbClusters)

        # control timer
        # self.control_timer = rospy.Timer(rospy.Duration(0.01),self.cbControl)
        self.nn_timer = rospy.Timer(rospy.Duration(0.3),self.cbComputeAction)

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

    def laser_scan_callback(self, scan):
        self.scan_param = [scan.angle_min, scan.angle_max, scan.angle_increment, scan.time_increment,
                            scan.scan_time, scan.range_min, scan.range_max]
        self.scan = np.array(scan.ranges)
        self.laser_cb_num += 1

    def get_laser_observation(self):
        scan = copy.deepcopy(self.scan)
        scan[np.isnan(scan)] = 6.0
        scan[np.isinf(scan)] = 6.0
        raw_beam_num = len(scan)
        sparse_beam_num = self.beam_mum
        step = float(raw_beam_num) / sparse_beam_num
        sparse_scan_left = []
        index = 0.
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_left.append(scan[int(index)])
            index += step
        sparse_scan_right = []
        index = raw_beam_num - 1.
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_right.append(scan[int(index)])
            index -= step
        scan_sparse = np.concatenate((sparse_scan_left, sparse_scan_right[::-1]), axis=0)
        return scan_sparse / 6.0 - 0.5
    def control_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.
        move_cmd.linear.z = 0.
        move_cmd.angular.x = 0.
        move_cmd.angular.y = 0.
        move_cmd.angular.z = action[1]
        self.pub_twist(move_cmd)

    def control_pose(self, pose):
        pose_cmd = Pose()
        assert len(pose)==3
        pose_cmd.position.x = pose[0]
        pose_cmd.position.y = pose[1]
        pose_cmd.position.z = 0

        qtn = tf.transformations.quaternion_from_euler(0, 0, pose[2], 'rxyz')
        pose_cmd.orientation.x = qtn[0]
        pose_cmd.orientation.y = qtn[1]
        pose_cmd.orientation.z = qtn[2]
        pose_cmd.orientation.w = qtn[3]
        self.cmd_pose.publish(pose_cmd)

    def cbComputeAction(self, event):
        while self.scan is None:
            pass
        obs = self.get_laser_observation() #用stage_world1.py里lasertopic获取
        obs_stack = deque([obs, obs, obs])
        # ************************************ Input ************************************
        # goal
        goal = [self.sub_goal.x, self.sub_goal.y]   #可以直接用此文件的subgoal，stage中的get_local_goal函数没有对self变量进行改变
        self.goal_point = goal
        # position
        state = [self.pose.pose.position.x, self.pose.pose.position.y, self.psi]    # x, y, theta
        self.state = state  #get_self_stateGT get_self_state有很多影响
        # Velocity
        speed = [self.vel, self.psi]
        self.speed_GT = speed # linear v +angular
        
        obs_state_list = [obs_stack, goal, speed]

        # self.control_pose(state)


        # ************************************ Output ************************************
        # agent: postion(x,y,theta),velocity(v,angular)

        _,scaled_action =generate_action_no_sampling(self.env, obs_state_list, self.policy, self.action_bound)
        action = scaled_action[0]
        self.control_vel(action)
        print('velocity : ', action[0], 'velocity_angular : ', action[1])


        # env.control_vel(action)      #involke the ros system publisher to send velocity to ros system
        # rospy.sleep(0.01)
        # # the following output function is depending on callback function in ros.
        # print('positon: x, y, theta', env.get_self_state())
        # print('speed of agent: v, angular ', env.get_self_speed())
                    


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
    

    # Set parameters of env
    LASER_HIST = 3
    NUM_ENV = 1    # the number of agents in the environment
    OBS_SIZE = 360  # number of leaserbeam
    action_bound = [[0, -1], [1, 1]]    # the limitation of velocity

    # Set env and agent policy
    # env = StageWorld(OBS_SIZE, index=0, num_env=NUM_ENV)    #index is useful for parallel programming, 0 is for the first agent

    trained_model_file = os.path.dirname(__file__) + '/policy/stage2.pth'
    # trained_model_file = '/policy/stage2.pth'
    policy = CNNPolicy(frames=LASER_HIST, action_space=2) 
    policy.cpu()    # policy.cuda() for gpu
    state_dict = torch.load(trained_model_file,map_location=torch.device('cpu'))    #torch.load(trained_model_file) for gpu
    policy.load_state_dict(state_dict)
  
    rospy.init_node('rl_collision_avoidance_tb3',anonymous=False)
    print('==================================\nrl_collision_avoidance node started')

    nn_tb3 = NN_tb3(policy, action_bound, OBS_SIZE, index=0, num_env=NUM_ENV)
    rospy.on_shutdown(nn_tb3.on_shutdown)

    rospy.spin()

if __name__ == '__main__':
    run()