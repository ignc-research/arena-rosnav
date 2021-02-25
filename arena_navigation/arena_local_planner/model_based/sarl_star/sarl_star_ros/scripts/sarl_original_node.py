#!/usr/bin/python2.7
# Author: Keyu Li <kyli@link.cuhk.edu.hk>

from __future__ import division
import logging
import os
import torch
import numpy as np
from nav_msgs.msg import Odometry, OccupancyGrid
import configparser
import gym
import tf
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.state import ObservableState, FullState, JointState
import rospy
from geometry_msgs.msg import Point, Vector3, Twist, Pose, PoseStamped, PoseWithCovarianceStamped, TwistWithCovariance
from std_msgs.msg import Int32, ColorRGBA
from people_msgs.msg import Person, People
from visualization_msgs.msg import Marker, MarkerArray


HUMAN_RADIUS = 0.3
ROBOT_RADIUS = 0.3
ROBOT_V_PREF = 0.5
DISCOMFORT_DIST = 0.1
FAKE_HUMAN_PX = -1.7
FAKE_HUMAN_PY = 14.3
TIME_LIMIT = 120
GOAL_TOLERANCE = 0.5

def add(v1, v2):
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)

class Robot(object):
    def __init__(self):
        self.v_pref = ROBOT_V_PREF
        self.radius = ROBOT_RADIUS
        self.px = None
        self.py = None
        self.gx = None
        self.gy = None
        self.vx = None
        self.vy = None
        self.theta = None

    def set(self, px, py, gx, gy, vx, vy, theta):
        self.px = px
        self.py = py
        self.gx = gx
        self.gy = gy
        self.vx = vx
        self.vy = vy
        self.theta = theta

    def get_full_state(self):
        return FullState(self.px, self.py, self.vx, self.vy, self.radius, self.gx, self.gy, self.v_pref, self.theta)

    def get_position(self):
        return self.px, self.py

    def get_goal_position(self):
        return self.gx, self.gy

    def reached_destination(self):
        return np.linalg.norm(np.array(self.get_position()) - np.array(self.get_goal_position())) < GOAL_TOLERANCE
        #      || (position - goal position) ||


class Human(object):
    def __init__(self, px, py, vx, vy):
        self.radius = HUMAN_RADIUS
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
    def get_observable_state(self):
        return ObservableState(self.px, self.py, self.vx, self.vy, self.radius)

class RobotAction(object):
    def __init__(self):
        self.Is_goal_received = False
        self.IsAMCLReceived = False
        self.IsObReceived = False
        self.Is_gc_Received = False
        self.getStartPoint = False
        self.Is_goal_reached = False
        self.received_gx = None
        self.received_gy = None
        self.px = None
        self.py = None
        self.vx = None
        self.vy = None
        self.gx = None
        self.gy = None
        self.v_pref = None
        self.theta = None
        self.humans = None
        self.ob = None
        self.state = None
        self.cmd_vel = Twist()
        self.plan_counter = 0
        self.num_pos = 0
        self.start_px = None
        self.start_py = None

        # subscribers
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_robot_pos)
        self.robot_odom_sub = rospy.Subscriber('/odom', Odometry, self.robot_vel_on_map_calculator)
        self.people_sub = rospy.Subscriber('/people', People, self.update_humans)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal_on_map)
        self.global_costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.get_gc)
        # publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.goal_marker_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)
        self.action_marker_pub = rospy.Publisher('/action_marker', Marker, queue_size=1)
        self.trajectory_marker_pub = rospy.Publisher('/trajectory_marker', Marker, queue_size=1)
        self.vehicle_marker_pub = rospy.Publisher('/vehicle_marker', Marker, queue_size=1)

    def update_robot_pos(self, msg):
        self.IsAMCLReceived = True
        self.num_pos += 1
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) # bounded by [-pi, pi]
        if not self.getStartPoint:
            rospy.loginfo("Start point is:(%s,%s)" % (self.px,self.py))
            self.getStartPoint = True
        self.visualize_trajectory(position, orientation)

    def robot_vel_on_map_calculator(self, msg):
        vel_linear = msg.twist.twist.linear
        listener_v.waitForTransform('/map', '/base_footprint', rospy.Time(), rospy.Duration(4))
        trans, rot = listener_v.lookupTransform('/map', '/base_footprint', rospy.Time())
        # rotate vector 'vel_linear' by quaternion 'rot'
        q1 = rot
        q2 = list()
        q2.append(vel_linear.x)
        q2.append(vel_linear.y)
        q2.append(vel_linear.z)
        q2.append(0.0)
        output_vel = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]
        self.vx = output_vel[0]
        self.vy = output_vel[1]


    def update_humans(self, msg):
        # observable state: px,py,vx,vy,radius
        self.IsObReceived = True
        self.humans = list()
        self.ob = list()
        for p in msg.people:
            # dist = np.linalg.norm(np.array([self.px,self.py])-np.array([p.position.x,p.position.y]))
            human = Human(p.position.x, p.position.y, p.velocity.x, p.velocity.y)
            self.humans.append(human)
        for human in self.humans:
            self.ob.append(human.get_observable_state())

    def get_goal_on_map(self, msg):
        self.Is_goal_received = True
        self.received_gx = msg.pose.position.x
        self.received_gy = msg.pose.position.y

    def get_gc(self, msg):
        if not self.Is_gc_Received:
            policy.gc = msg.data
            policy.gc_resolution = msg.info.resolution
            policy.gc_width = msg.info.width
            policy.gc_ox = msg.info.origin.position.x
            policy.gc_oy = msg.info.origin.position.y
            # print(policy.gc_resolution, policy.gc_width, policy.gc_ox, policy.gc_oy)
            print("************ Global costmap is received. **************")
            self.Is_gc_Received = True

    def visualize_goal(self):
        # red cube for goal
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal"
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = self.gx
        marker.pose.position.y = self.gy
        marker.pose.position.z = 0.2
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker.color = ColorRGBA(r=1.0, a=1.0)
        marker.lifetime = rospy.Duration()
        self.goal_marker_pub.publish(marker)

    def visualize_trajectory(self, position, orientation):
        # Purple track for robot trajectory over time
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = '/map'
        marker.ns = 'robot'
        marker.id = self.num_pos
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.pose.position = position
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker.color = ColorRGBA(r=0.5, b=0.8, a=1.0)
        marker.lifetime = rospy.Duration()
        self.trajectory_marker_pub.publish(marker)

    def visualize_action(self):
        robot_pos = Point(x=self.px, y=self.py, z=0)
        next_theta = self.theta + self.cmd_vel.angular.z
        next_vx = self.cmd_vel.linear.x * np.cos(next_theta)
        next_vy = self.cmd_vel.linear.x * np.sin(next_theta)
        action = Vector3(x=next_vx, y=next_vy, z=0)
        # green arrow for action (command velocity)
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "/map"
        marker.ns = "action"
        marker.id = 0
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.points = [robot_pos, add(robot_pos, action)]
        marker.scale = Vector3(x=0.1, y=0.3, z=0)
        marker.color = ColorRGBA(g=1.0, a=1.0)
        marker.lifetime = rospy.Duration(0.5)
        self.action_marker_pub.publish(marker)

    def planner(self):
        # update robot
        robot.set(self.px, self.py, self.gx, self.gy, self.vx, self.vy, self.theta)
        dist_to_goal = np.linalg.norm(np.array(robot.get_position()) - np.array(robot.get_goal_position()))

        # compute command velocity
        if robot.reached_destination():
            self.cmd_vel.linear.x = 0
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = 0
            self.Is_goal_reached = True

        else:
            """
            self state: FullState(px, py, vx, vy, radius, gx, gy, v_pref, theta)
            ob:[ObservableState(px1, py1, vx1, vy1, radius1),
                ObservableState(px1, py1, vx1, vy1, radius1),
                   .......                    
                ObservableState(pxn, pyn, vxn, vyn, radiusn)]
            """
            if len(self.ob)==0:
                self.ob = [ObservableState(FAKE_HUMAN_PX, FAKE_HUMAN_PY, 0, 0, HUMAN_RADIUS)]

            self.state = JointState(robot.get_full_state(), self.ob)
            action = policy.predict(self.state)  # max_action
            self.cmd_vel.linear.x = action.v
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0
            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = action.r

        ########### for debug ##########
        # dist_to_goal = np.linalg.norm(np.array(robot.get_position()) - np.array(robot.get_goal_position()))
        # if self.plan_counter % 10 == 0:
        #     rospy.loginfo("robot position:(%s,%s)" % (self.px, self.py))
        #     rospy.loginfo("Distance to goal is %s" % dist_to_goal)
        #     rospy.loginfo("self state:\n %s" % self.state.self_state)
        #     for i in range(len(self.state.human_states)):
        #         rospy.loginfo("human %s :\n %s" % (i+1, self.state.human_states[i]))
        #     rospy.loginfo("%s-th action is planned: \n v: %s m/s \n r: %s rad/s"
        #                   % (self.plan_counter, self.cmd_vel.linear.x, self.cmd_vel.angular.z))


        # publish velocity
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.plan_counter += 1
        self.visualize_action()


if __name__ == '__main__':
    begin_travel = False
    # set file dirs
    model_dir = '/home/likeyu/sarl_ws/src/sarl_star/CrowdNav/crowd_nav/data/output/'
    env_config_file = '/home/likeyu/sarl_ws/src/sarl_star/CrowdNav/crowd_nav/data/output/env.config'
    policy_config_file = '/home/likeyu/sarl_ws/src/sarl_star/CrowdNav/crowd_nav/data/output/policy.config'
    if os.path.exists(os.path.join(model_dir, 'resumed_rl_model.pth')):
        model_weights = os.path.join(model_dir, 'resumed_rl_model.pth')
    else:
        model_weights = os.path.join(model_dir, 'rl_model.pth')

    # configure logging and device
    logging.basicConfig(level=logging.INFO, format='%(asctime)s, x%(levelname)s: %(message)s',
                        datefmt="%Y-%m-%d %H:%M:%S")
    device = torch.device("cpu")
    logging.info('Using device: %s', device)

    # configure RL policy
    policy = 'sarl'
    phase = 'test'
    env_config = configparser.RawConfigParser()
    env_config.read(env_config_file)
    env = gym.make('CrowdSim-v0')
    env.configure(env_config)
    env.discomfort_dist = DISCOMFORT_DIST
    policy = policy_factory[policy]()
    policy_config = configparser.RawConfigParser()
    policy_config.read(policy_config_file)
    policy.configure(policy_config)
    policy.with_costmap = True
    # use constant velocity model to predict next state
    policy.query_env = False 
    policy.get_model().load_state_dict(torch.load(model_weights))
    policy.set_phase(phase)
    policy.set_device(device)
    policy.set_env(env)
    policy.time_step = 0.25
    policy.gc = []
    robot = Robot()

    try:
        rospy.init_node('sarl_original_node', anonymous=True)
        rate = rospy.Rate(4)  # 4 Hz, time_step=0.25
        robot_act = RobotAction()
        listener_v = tf.TransformListener()

        while not rospy.is_shutdown():
            if robot_act.Is_goal_reached:
                finish_travel_time = rospy.get_time()
                t = finish_travel_time - begin_travel_time
                rospy.loginfo("Goal is reached. Travel time: %s s." % t)
                break

            # wait for msgs of goal, AMCL and ob
            if robot_act.Is_goal_received and robot_act.IsAMCLReceived and robot_act.IsObReceived:

                # travel time
                if not begin_travel:
                    begin_travel_time = rospy.get_time()
                    begin_travel = True

                # update goal (gx,gy)
                robot_act.gx = robot_act.received_gx
                robot_act.gy = robot_act.received_gy
                robot_act.visualize_goal()
                robot_act.planner()
                finish_travel_time = rospy.get_time()
                t = finish_travel_time - begin_travel_time
                if t > TIME_LIMIT:
                    rospy.loginfo("Timeout. Travel time: %s s." % t)
                    break
            rate.sleep()


    except rospy.ROSInterruptException, e:
        raise e



