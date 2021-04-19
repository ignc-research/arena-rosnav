#! /usr/bin/env python
from operator import is_
from random import randint
import gym
from gym import spaces
from gym.spaces import space
from typing import Union
from stable_baselines3.common.env_checker import check_env
import yaml
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.utils.observation_collector import ObservationCollector
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.utils.reward import RewardCalculator
from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.utils.debug import timeit
from task_generator.tasks import ABSTask 
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from flatland_msgs.srv import StepWorld, StepWorldRequest
from nav_msgs.msg import Odometry, Path
import time
# for transformations
from tf.transformations import *
import subprocess

from arena_plan_msgs.msg import RobotState, RobotStateStamped

class wp3Env(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self, task: ABSTask, robot_yaml_path: str, settings_yaml_path: str, is_action_space_discrete, safe_dist: float = None, goal_radius: float = 0.1, max_steps_per_episode=100):
        """Default env
        Flatland yaml node check the entries in the yaml file, therefore other robot related parameters cound only be saved in an other file.
        TODO : write an uniform yaml paser node to handel with multiple yaml files.



        Args:
            task (ABSTask): [description]
            robot_yaml_path (str): [description]
            setting_yaml_path ([type]): [description]
            is_action_space_discrete (bool): [description]
            safe_dist (float, optional): [description]. Defaults to None.
            goal_radius (float, optional): [description]. Defaults to 0.1.
        """
        super(wp3Env, self).__init__()
       
        # Define action and observation space
        # They must be gym.spaces objects
        self._is_action_space_discrete = is_action_space_discrete
        self.setup_by_configuration(robot_yaml_path, settings_yaml_path)
        # observation collector
        self.observation_collector = ObservationCollector(
            self._laser_num_beams, self._laser_max_range)
        self.observation_space = self.observation_collector.get_observation_space()

        # reward calculator
        if safe_dist is None:
            safe_dist = 1.6*self._robot_radius

        self.reward_calculator = RewardCalculator(
             robot_radius=self._robot_radius, safe_dist=1.2*self._robot_radius, goal_radius=goal_radius)
        #subscriber to infer callback and out of sleep loop
        #sub robot position and sub global goal 
        self._robot_state_sub = rospy.Subscriber('/odom', Odometry, self.cbRobotPosition)            # for robot orientation
        self._ref_wp_sub = rospy.Subscriber('/plan_manager/subgoal', PoseStamped, self.cbRefWp)                   # for subgoal position (reference point for circle)
        self._globalGoal = rospy.Subscriber('/goal', PoseStamped, self.cbGlobalGoal)                 # to set wp on global goal if circle touces it
        self._globalPlan_sub = rospy.Subscriber('/plan_manager/globalPlan', Path, self.cbglobalPlan)
        self._twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.cbTwist)                          # to calculate distance traveled 
        self._wp4train_reached =False

        # action agent publisher
        self.agent_action_pub = rospy.Publisher('/plan_manager/wp4train', PoseStamped, queue_size=1)        # agent publishes a waypoint on a circle 
        self.circle_pub = rospy.Publisher('/zViz', Path, queue_size=1)                               # create the circle on which the wp are placed for visualization


        # service clients
        self._is_train_mode = rospy.get_param("train_mode")
        if self._is_train_mode:
            self._service_name_step = '/step_world'
            self._sim_step_client = rospy.ServiceProxy(
            self._service_name_step, StepWorld)
        self.task = task

        self._max_steps_per_episode = max_steps_per_episode
        #global variables for subscriber callbacks
        self.range_circle = 1.5
        self._steps_curr_episode = 0
        self._robot_pose = PoseStamped()
        self._globalPlan = Path()
        self._subgoal = Pose2D()
        self._globalGoal = Pose2D()
        self._ref_wp = PoseStamped()
        self._action_msg = PoseStamped()
        self._viz_msg = Path()
        self._viz_points = PoseStamped()

        self._robot_twist = [0]*2
        self.firstTime = 0
        self._previous_time = 0
        self._step_counter = 0
        # for reward to calculate how many actions relative to path length
        self.goal_len = 0
        self._action_count = 0
        # # get observation
        # obs=self.observation_collector.get_observations()

    def cbRobotPosition(self,msg):
        self._robot_pose.pose.position.x = msg.pose.pose.position.x
        self._robot_pose.pose.position.y = msg.pose.pose.position.y
        self._robot_pose.pose.orientation = msg.pose.pose.orientation
 
        
    def cbTwist(self,msg):
        self._robot_twist[0]= msg.linear.x
        self._robot_twist[1] = msg.angular.z    

    def cbglobalPlan(self,msg):
        self._globalPlan = msg

    def cbRefWp(self,msg):
        self._ref_wp = msg

    def cbGlobalGoal(self,msg):
        self._globalGoal.x = msg.pose.position.x
        self._globalGoal.y = msg.pose.position.y

    def setup_by_configuration(self, robot_yaml_path: str, settings_yaml_path: str):
        """get the configuration from the yaml file, including robot radius, discrete action space and continuous action space.

        Args:
            robot_yaml_path (str): [description]
        """
        with open(robot_yaml_path, 'r') as fd:
            robot_data = yaml.safe_load(fd)
            # get robot radius
            for body in robot_data['bodies']:
                if body['name'] == "base_footprint":
                    for footprint in body['footprints']:
                        if footprint['type'] == 'circle':
                            self._robot_radius = footprint.setdefault(
                                'radius', 0.3)*1.05
                        if footprint['radius']:
                            self._robot_radius = footprint['radius']*1.05
            # get laser related information
            for plugin in robot_data['plugins']:
                if plugin['type'] == 'Laser':
                    laser_angle_min = plugin['angle']['min']
                    laser_angle_max = plugin['angle']['max']
                    laser_angle_increment = plugin['angle']['increment']
                    self._laser_num_beams = int(
                        round((laser_angle_max-laser_angle_min)/laser_angle_increment)+1)
                    self._laser_max_range = plugin['range']

        with open(settings_yaml_path, 'r') as fd:
            setting_data = yaml.safe_load(fd)
            if self._is_action_space_discrete:
                # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
                self._discrete_acitons = setting_data['robot']['discrete_actions']
                self.action_space = spaces.Discrete(
                    len(self._discrete_acitons))
            else:

                angular_range = setting_data['robot']['continuous_actions']['angular_range']
                self.action_space = spaces.Box(low=np.array([angular_range[0]]),
                                               high=np.array([angular_range[1]]), dtype=np.float)


    def clear_costmaps(self):
        bashCommand = "rosservice call /move_base/clear_costmaps"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        return output, error

    def _calc_distance(self, goal_pos:Pose2D,robot_pos:Pose2D):
         y_relative = goal_pos.y - robot_pos.y
         x_relative = goal_pos.x - robot_pos.x
         rho =  (x_relative**2+y_relative**2)**0.5
         theta = (np.arctan2(y_relative,x_relative)-robot_pos.theta+4*np.pi)%(2*np.pi)-np.pi
         return rho,theta

    def _pub_action(self, action):

        _, obs_dict = self.observation_collector.get_observations()
        dist_robot_goal = obs_dict['goal_in_robot_frame']
        dist_global_sub = obs_dict['global_in_subgoal_frame']
        #transform action which is a waypoint to 2d to calculate distance robot-wp
        wp2d = Pose2D()
        wp2d.x = self._action_msg.pose.position.x
        wp2d.y = self._action_msg.pose.position.y

        robot_pos =  Pose2D()
        robot_pos.x = self._robot_pose.pose.position.x 
        robot_pos.y = self._robot_pose.pose.position.y
    
        #calculate distance between robot and waypoint
        dist_robot_wp = self._calc_distance(wp2d, robot_pos)
        self._action_msg.pose.orientation.z =  0 
        self._action_msg.pose.orientation.w = 1
        self._action_msg.header.frame_id ="map"

        #create circle on which waypoints can be spawned 
        circle = Path()
        circle.header.frame_id ="map"
        circle.header.stamp = rospy.Time.now()
        
        ## Draw circle, bound by action space [-2,2] radians, [-3.1,3.1] --> [-pi,pi]
        i = -2
        j = 0
        while (i < 2):
            point = PoseStamped()
            q = self._robot_pose.pose.orientation
            robot_angle = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angle_grad = i + robot_angle 
            point.pose.position.x = self._ref_wp.pose.position.x + (self.range_circle*math.cos(angle_grad))         
            point.pose.position.y = self._ref_wp.pose.position.y + (self.range_circle*math.sin(angle_grad))
            point.pose.orientation.w=1
            point.header.frame_id="map"
            circle.poses.append(point)

            # if circle touches the global goal, immediately place subgoal on global goal
            if ((math.isclose(circle.poses[j].pose.position.x, self._globalGoal.x, rel_tol=0.2)) and (math.isclose(circle.poses[j].pose.position.y, self._globalGoal.y, rel_tol=0.2))):    
                self._action_msg.pose.position.x = self._globalGoal.x 
                self._action_msg.pose.position.y = self._globalGoal.y 
                self._action_msg.pose.orientation.w = 1
                self.agent_action_pub.publish(self._action_msg)
                self._action_count += 1
            i += 0.2
            j += 1
        
        #visualize circle by publishing all poses to /zViz topic
        self.circle_pub.publish(circle)
       
       # First spawn 
        if self.firstTime < 1:
            q = self._robot_pose.pose.orientation
            robot_angle = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            angle_grad = action[0] + robot_angle 
            self._action_msg.pose.position.x = self._ref_wp.pose.position.x + (self.range_circle*math.cos(angle_grad))         
            self._action_msg.pose.position.y = self._ref_wp.pose.position.y + (self.range_circle*math.sin(angle_grad))
            self.agent_action_pub.publish(self._action_msg)
            self.firstTime +=1
            self._action_count += 1
            print("distance robot to wp: {}".format(dist_robot_wp[0]))
            
        #wait for robot to reach the waypoint first
        #if self._step_counter - self._previous_time > 30:
        if dist_robot_wp[0] < 0.6:
            self._previous_time = self._step_counter
            _, obs_dict = self.observation_collector.get_observations()
            dist_robot_goal = obs_dict['goal_in_robot_frame']
            
            #dist_robot_goal = np.array([self._robot_pose.x - self._subgoal.x, self._robot_pose.y - self._subgoal.y])
            dist_rg = np.linalg.norm(dist_robot_goal)            
            #todo consider the distance to global path when choosing next optimal waypoint
            #send a goal message (posestamped) as action, remeber to normalize the quaternions (put orientationw as 1) and set the frame id of the goal to "map"! 
            if dist_robot_goal[0] < 2:
                self._action_msg.pose.position.x = self._globalGoal.x 
                self._action_msg.pose.position.y = self._globalGoal.y 
                self._action_msg.pose.orientation.w = 1
                self.agent_action_pub.publish(self._action_msg)
                self._action_count += 1
            else:
                q = self._robot_pose.pose.orientation
                robot_angle = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                angle_grad = action[0] + robot_angle 
                self._action_msg.pose.position.x = self._ref_wp.pose.position.x + (self.range_circle*math.cos(angle_grad))         
                self._action_msg.pose.position.y = self._ref_wp.pose.position.y + (self.range_circle*math.sin(angle_grad))   
                self._action_msg.pose.orientation.w = 1
                print("action message looks like {}".format(self._action_msg))
                self.agent_action_pub.publish(self._action_msg)
                print(angle_grad)
                
                self._action_count += 1

    def step(self, action):
        """
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
        """
        self._step_counter += 1
        self._pub_action(action)
        self._steps_curr_episode += 1

        ##todo: wait for robot_pos = goal pos

        #in each step, get the robots cmd velocity to get action for reward distance traveled
        new_action = [0]*2
        new_action[0] = self._robot_twist[0]
        new_action[1] = self._robot_twist[1]
        
        # wait for new observations
        s = time.time()
        merged_obs, obs_dict = self.observation_collector.get_observations()
        # print("get observation: {}".format(time.time()-s))

        # get global path once new episode started and round to integer for reward waypoints_set relative to goal distance
        if self.firstTime < 2:
            self.goal_len = int(round(obs_dict['goal_in_robot_frame'][0]))
        # provide reward calculator with the data to calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'],
            robot_pose=obs_dict['robot_pose'], 
            globalPlan=self._globalPlan, 
            action=new_action, 
            goal_len=self.goal_len, 
            action_count= self._action_count,
            subgoal=self._action_msg.pose.position)

        done = reward_info['is_done']
        print("reward:  {}".format(reward))
        # info
        info = {}
        if done:
            info['done_reason'] = reward_info['done_reason']
            info['goal_len'] = self.goal_len
            self.reward_calculator.kdtree = None

        if self._steps_curr_episode > self._max_steps_per_episode:
            done = True
            info['done_reason'] = 0
            info['is_success'] = 0
            self.reward_calculator.kdtree = None

        return merged_obs, reward, done, info

    def reset(self):
        self.clear_costmaps()

        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self.agent_action_pub.publish(PoseStamped())
        if self._is_train_mode:
            self._sim_step_client()
        self.task.reset()
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        self.firstTime = 0
        self._action_count = 0
        obs, _ = self.observation_collector.get_observations()
        return obs  # reward, done, info can't be included

    def close(self):
        pass


if __name__ == '__main__':

    rospy.init_node('wp3_gym_env', anonymous=True)
    print("start")

    wp3_env = wp3Env()
    check_env(wp3_env, warn=True)

    # init env
    obs = wp3_env.reset()

    # run model
    n_steps = 200
    for step in range(n_steps):
        # action, _states = model.predict(obs)
        action = wp3_env.action_space.sample()

        obs, rewards, done, info = wp3_env.step(action)

        time.sleep(0.1)
