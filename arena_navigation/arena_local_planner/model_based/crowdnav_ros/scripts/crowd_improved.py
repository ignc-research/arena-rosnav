#!/usr/bin/env python3
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
from crowd_nav.policy.policy_factory import policy_factory
# 
from crowd_nav.policy.cadrl import CADRL
from crowd_nav.policy.lstm_rl import LstmRL
from crowd_nav.policy.sarl import SARL
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.utils.state import JointState

# prototype
from visualization_msgs.msg import Marker, MarkerArray
from ford_msgs.msg import Clusters
from geometry_msgs.msg import Vector3,PoseStamped
from std_msgs.msg import ColorRGBA


class TestNode():
    def __init__(self, env, env_config, policy):
        self.tb3 = RosNav('/lp_goal','/lp_goal')
        # self.tb3 = RosNav('/goal','/plan_manager/subgoal')
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

        # prototype
        self.pub_path_marker = rospy.Publisher('/visualizer/path',Marker,queue_size=1)
        self.pub_agent_markers = rospy.Publisher('/other_agents_markers',MarkerArray,queue_size=1)
        self.sub_clusters = rospy.Subscriber('/obst_odom',Clusters, self.cbClusters)
        self.other_agents_state = {}
        self.desired_position = PoseStamped()

        self.pub_goal = rospy.Publisher('/lp_goal',PoseStamped,queue_size=1)

        # control loop
        rospy.Timer(rospy.Duration(0.01),self.cbControl)
        rospy.Timer(rospy.Duration(0.1),self.cbComputeActionCrowdNav)

    def cbClusters(self,msg):
        # print(msg)

        xs = []; ys = []; radii = []; labels = []
        num_clusters = len(msg.mean_points)
        vx = []; vy = []
        # print(num_clusters)
        for i in range(num_clusters):
            index = msg.labels[i]
            # if index > 24: #for static map
            x = msg.mean_points[i].x; y = msg.mean_points[i].y
            v_x = msg.velocities[i].x; v_y = msg.velocities[i].y
            vx.append(v_x)
            vy.append(v_y)

            inflation_factor = 1.5
            
            radius = msg.mean_points[i].z*inflation_factor

            xs.append(x); ys.append(y); radii.append(radius); labels.append(index); 

        self.visualize_other_agents(xs, ys, radii, labels)
        self.other_agents_state["pos"] = [xs, ys] 
        self.other_agents_state["v"] = [vx,vy]
        self.other_agents_state["r"] = radii

        # print("received")

    def max_yaw(self, a):
        phi = 0
        phi_tol = 0.1

        if 0 < a <= phi_tol:
            phi = -0.2
        if phi_tol < a <= 3*phi_tol:
            phi = -0.3
        if 3*phi_tol < a < 7*phi_tol:
            phi = -0.4
        if a >= 7*phi_tol:
            phi = -0.5
        # neg
        if 0 > a >= -phi_tol:
            phi = 0.2
        if -phi_tol > a >= -3*phi_tol:
            phi = 0.3
        if -3*phi_tol > a > -7*phi_tol:
            phi = 0.4
        if a <= -7*phi_tol:
            phi = 0.5

        # print(phi, a)
        return phi
    
    def cbControl(self,event):
        holonomic = True
        twist = Twist()
        if not self.tb3.goalReached():
            if holonomic:
                # abs(self.angle2Action) > 0.1 and
                vel = np.array([self.tb3.raw_action[0],self.tb3.raw_action[1]])
                if abs(self.angle2Action) < math.pi/2:
                    twist.linear.x = 0.3*np.linalg.norm(vel)
                else:
                    twist.linear.x = 0.1*np.linalg.norm(vel)

                twist.angular.z = self.max_yaw(self.angle2Action)
            else:
                 twist.linear.x = self.tb3.raw_action.v/3
                 twist.angular.z = self.tb3.raw_action.r 
        # print(twist)
        self.tb3.pub_twist.publish(twist)

    def update_angle2Action(self):
        # action vector
        v_a = np.array([self.tb3.raw_action[0], self.tb3.raw_action[1]])
        # pose direction
        phi = self.tb3.angle_pose
        e_dir = np.array([math.cos(phi), math.sin(phi)])
        # angle: <v_a, e_dir>
        self.angle2Action = np.math.atan2(np.linalg.det([v_a,e_dir]),np.dot(v_a,e_dir))
        self.desired_position
        self.visualize_action()

    def cbComputeActionCrowdNav(self,event):

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = -2.5
        goal.pose.position.y = -3
        self.pub_goal.publish(goal)

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
        obstacle_radius = 6
        if True:
            for prop in self.other_agents_state:
                if prop == "pos":
                    x_y = self.other_agents_state[prop]
                    for i in range(len(x_y[0])):
                        obstacle_x[i] = x_y[0][i]
                        obstacle_y[i] = x_y[1][i]
                if prop == "v":
                    v = self.other_agents_state[prop]
                    for i in range(len(v[0])):
                        obstacle_vx[i] = v[0][i]
                        obstacle_vy[i] = v[1][i]
            # print(math.sqrt(obstacle_vx[3]**2+obstacle_vy[3]**2))
            print("------------------------")
            for i in range(5):
                print("pos",i,":",obstacle_x[i]," ",obstacle_y[i])
                v = math.sqrt(obstacle_vx[i]**2 + obstacle_vy[i]**2)
                print("vel",i,":",obstacle_vx[i]," ",obstacle_vy[i],"|  v = ", v)
        else:
            if "v" in self.other_agents_state:
                v = self.other_agents_state["v"]
                for i in range(len(v[0])):
                    vm = math.sqrt(v[0][i]**2 + v[1][i]**2)
                    if vm>0:
                        print(i,vm)

        # initial obstacle instances and set value
        for i in range(self.env_config.getint('sim','human_num')):
            self.env.humans[i].set(obstacle_x[i], obstacle_y[i], goal_x,goal_y, obstacle_vx[i], obstacle_vy[i], theta, obstacle_radius)
            self.ob[i]= self.env.humans[i].get_observable_state()

        # ************************************ Output ************************************
        # get action info
        action = self.robot.act(self.ob)
        self.desired_position.pose.position.x = self.tb3.pose.pose.position.x + (action[0])
        self.desired_position.pose.position.y = self.tb3.pose.pose.position.y + (action[1])

        self.tb3.update_action(action)
        # print(action.v,action.r)
        print(action)
        self.update_angle2Action()


        # state   = JointState(self.robot.get_full_state(), self.ob)
        # action  = self.policy.predict(state)


    def visualize_other_agents(self,xs,ys,radii,labels):
            markers = MarkerArray()
            for i in range(len(xs)):
                # Orange box for other agent
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = 'map'
                marker.ns = 'other_agent'
                marker.id = labels[i]
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.pose.position.x = xs[i]
                marker.pose.position.y = ys[i]
                # marker.pose.orientation = orientation
                marker.scale = Vector3(x=2*radii[i],y=2*radii[i],z=1)
                if labels[i] <= 23: # for static map
                    # print sm
                    marker.color = ColorRGBA(r=0.5,g=0.4,a=1.0)
                else:
                    marker.color = ColorRGBA(r=1.0,g=0.4,a=1.0)
                marker.lifetime = rospy.Duration(0.5)
                markers.markers.append(marker)

            self.pub_agent_markers.publish(markers)

    def visualize_action(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = 'map'
        marker.ns = 'action_dir'
        marker.id = 0
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.points.append(self.tb3.pose.pose.position)
        marker.points.append(self.desired_position.pose.position)
        marker.scale = Vector3(x=0.1,y=0.2,z=0.2)
        marker.color = ColorRGBA(b=1.0,a=1.0)
        marker.lifetime = rospy.Duration(1)
        self.pub_path_marker.publish(marker)

def run():
    
    # start node
    rospy.init_node("crowd_impr", anonymous=False)
    # rospy.sleep(0.1) # sometimes node isnt recognized
    print('==================================\ncrowd-node started\n==================================')

    policy_name = "sarl"

    device  = 'cpu'
    phase   = 'test'

    select_policy       = {"cadrl":CADRL(),"lstm":LstmRL(),"sarl":SARL()}
    # the path of training result which contains configs and rl mode
    env_config_file     = 'crowd_nav/data/output/env.config'             #path beginging without slash
    policy_config_file  = 'crowd_nav/data/output/policy.config'
    model_weights       = 'crowd_nav/data/output/rl_model_'+policy_name+'.pth'
    policy_config       =  configparser.RawConfigParser()
    policy_config.read(policy_config_file)

    policy = select_policy[policy_name]     #{SARL(),CADRL(),LstmRL()}
    policy.configure(policy_config)
    policy.get_model().load_state_dict(torch.load(model_weights))
    policy.set_device(device)
    policy.set_phase(phase)

    # configure environment / obstacles
    env_config  = configparser.RawConfigParser()
    env_config.read(env_config_file)
    env         = gym.make('CrowdSim-v0')   #env is inherited from CrowdSim class in crowd_sim.py
    env.configure(env_config)


    test = TestNode(env,env_config,policy)
    rospy.spin()


if __name__ == '__main__':
    run()