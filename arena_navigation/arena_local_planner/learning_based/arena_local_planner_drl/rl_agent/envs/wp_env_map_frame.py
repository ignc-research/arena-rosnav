#! /usr/bin/env python
from random import randint
import gym
from gym import spaces
from gym.spaces import space
from typing import Callable, Iterable, Optional, Union

from numpy.lib.function_base import angle
from rospy.core import rospywarn
from stable_baselines3.common.env_checker import check_env
import yaml
from rl_agent.utils.observation_collector import ObservationCollectorWP
from rl_agent.utils.debug import timeit
from task_generator.tasks import ABSTask
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from flatland_msgs.srv import StepWorld, StepWorldRequest
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as nav_Path
from std_msgs.msg import Bool
import time
import math
from rl_agent.utils.debug import timeit

from arena_plan_msgs.msg import RobotState, RobotStateStamped

from ..config import configurable
from .build import CfgNode, ENV_REGISTRY
from ..utils.reward import build_reward_calculator, REWARD_REGISTRY


@ENV_REGISTRY.register()
class WPEnvMapFrame(gym.Env):
    """A Environment whose action is waypoint"""

    INFO_KEYS = ("event", "done")

    @configurable
    def __init__(
        self,
        ns: str,
        cfg: CfgNode,
        task: Union["task_generator.ABCTask", Callable, None],
        robot_yaml_path,
        waypoint_generator_actions_yaml_path,
        robot_waypoint_min_dist: float,
        robot_start_pos_goal_pos_min_dist: float,
        goal_radius: float,
        debug: bool,
        is_action_space_discrete: bool,
        step_timeout: float,
        max_steps_per_episode,
    ):
        """Default env
        Flatland yaml node check the entries in the yaml file, therefore other robot related parameters cound only be saved in an other file.
        TODO : write an uniform yaml paser node to handel with multiple yaml files.


        Args:
            task (ABSTask): [description]
            train_mode (bool): bool to differ between train and eval env during training
            goal_radius (float, optional): [description]. Defaults to 0.1.
        """
        super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        self.ns = ns
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            ns_int = int(ns.split("_")[1])
            self._ns_int = ns_int
            time.sleep(ns_int * 2)
        except Exception:
            rospy.logwarn(
                f"Can't not determinate the number of the environment, training script may crash!"
            )
            pass

        # process specific namespace in ros system
        self.ns_prefix = "" if (ns == "" or ns is None) else "/" + ns + "/"
        self._is_action_space_discrete = is_action_space_discrete
        if debug:
            log_level = rospy.INFO
        else:
            log_level = rospy.WARN

        self._is_train_mode = rospy.get_param("/train_mode")

        if self._is_train_mode:
            # TODO change it back to info
            rospy.init_node(
                f"train_env_{self.ns}", disable_signals=True, log_level=log_level
            )
        else:
            rospy.init_node(
                f"deploy_env_{self.ns}", disable_signals=True, log_level=log_level
            )

        self.setup_by_configuration(
            robot_yaml_path, waypoint_generator_actions_yaml_path
        )
        rospy.set_param("/laser_num_beams", self._laser_num_beams)
        self._robot_waypoint_min_dist = robot_waypoint_min_dist
        self._robot_obstacle_min_dist = self._robot_radius

        # observation collector
        self.observation_collector = ObservationCollectorWP(
            self.ns,
            self._is_train_mode,
            self._laser_num_beams,
            self._laser_angle_increment,
            self._laser_max_range,
            self._robot_waypoint_min_dist,
            self._robot_obstacle_min_dist,
        )
        self.observation_space = self.observation_collector.get_observation_space()
        self.reward_calculator = build_reward_calculator(
            cfg, self.observation_collector, self
        )
        self._step_counter = 0
        self._waypoint_x = None
        self._waypoint_y = None
        self.is_waypoint_set_to_global_goal = False
        self._goal_radius = goal_radius
        self._robot_start_pos_goal_pos_min_dist = robot_start_pos_goal_pos_min_dist
        self._step_timeout = step_timeout
        self._curr_steps_count = 0
        self._max_steps_per_episode = max_steps_per_episode

        self.agent_action_pub = rospy.Publisher(
            f"{self.ns_prefix}waypoint", PoseStamped, queue_size=1, tcp_nodelay=True
        )

        if callable(task):
            task = task()
        self.task = task

    @classmethod
    def from_config(cls, cfg: CfgNode, task, ns, train_mode, debug):

        # TODO The code here is very dirty, later on we will put reward related
        # stuffes in other places
        robot_yaml_path = cfg.ROBOT.FLATLAND_DEF
        waypoint_generator_actions_yaml_path = cfg.WAYPOINT_GENERATOR.ACTIONS_DEF
        robot_waypoint_min_dist = cfg.WAYPOINT_GENERATOR.ROBOT_WAYPOINT_MIN_DIST
        goal_radius = cfg.WAYPOINT_GENERATOR.GOAL_RADIUS
        step_timeout = cfg.WAYPOINT_GENERATOR.STEP_TIMEOUT
        is_action_space_discrete = cfg.WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE
        robot_start_pos_goal_pos_min_dist = (
            cfg.TRAINING.ROBOT_START_POS_GOAL_POS_MIN_DIST
        )
        max_steps_per_episode = cfg.TRAINING.MAX_STEPS_PER_EPISODE
        return dict(
            ns=ns,
            cfg=cfg,
            task=task,
            robot_yaml_path=robot_yaml_path,
            waypoint_generator_actions_yaml_path=waypoint_generator_actions_yaml_path,
            robot_waypoint_min_dist=robot_waypoint_min_dist,
            goal_radius=goal_radius,
            robot_start_pos_goal_pos_min_dist=robot_start_pos_goal_pos_min_dist,
            debug=debug,
            is_action_space_discrete=is_action_space_discrete,
            step_timeout=step_timeout,
            max_steps_per_episode=max_steps_per_episode,
        )

    def setup_by_configuration(
        self, robot_yaml_path: str, waypoint_generator_actions_yaml_path: str
    ):
        """get the configuration from the yaml file, including robot radius, discrete action space and continuous action space.

        Args:
            robot_yaml_path (str): [description]
        """
        with open(robot_yaml_path, "r") as fd:
            robot_data = yaml.safe_load(fd)
            # get robot radius
            for body in robot_data["bodies"]:
                if body["name"] == "base_footprint":
                    for footprint in body["footprints"]:
                        if footprint["type"] == "circle":
                            self._robot_radius = (
                                footprint.setdefault("radius", 0.3) * 1.05
                            )
                        if footprint["radius"]:
                            self._robot_radius = footprint["radius"] * 1.05
            # get laser related information
            for plugin in robot_data["plugins"]:
                if plugin["type"] == "Laser":
                    laser_angle_min = plugin["angle"]["min"]
                    laser_angle_max = plugin["angle"]["max"]
                    laser_angle_increment = plugin["angle"]["increment"]
                    self._laser_num_beams = int(
                        round(
                            (laser_angle_max - laser_angle_min) / laser_angle_increment
                        )
                        + 1
                    )
                    self._laser_max_range = plugin["range"]
                    self._laser_angle_increment = laser_angle_increment

        with open(waypoint_generator_actions_yaml_path, "r") as fd:
            setting_data = yaml.safe_load(fd)
            if self._is_action_space_discrete:
                # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
                discrete_acitons_def = setting_data["waypoint_generator"][
                    "discrete_actions"
                ]
                self._discrete_acitons = []
                for actions_def in discrete_acitons_def:
                    if isinstance(actions_def["angular"], list):
                        linear = actions_def["linear"]
                        angular_def = list(
                            map(
                                lambda x: eval(x) if isinstance(x, str) else x,
                                actions_def["angular"],
                            )
                        )
                        angulars = np.linspace(*angular_def)
                        self._discrete_acitons += list(
                            map(
                                lambda angular, linear=linear: [linear, angular],
                                angulars,
                            )
                        )
                    else:
                        self._discrete_acitons.append(
                            [
                                actions_def["linear"],
                                eval(actions_def["angular"])
                                if isinstance(actions_def["angular"], str)
                                else actions_def["angular"],
                            ]
                        )
                self.action_space = spaces.Discrete(len(self._discrete_acitons))
            else:
                # TODO  read the range data from unified config file
                linear_range = setting_data["waypoint_generator"]["continuous_actions"][
                    "linear_range"
                ]
                angular_range = setting_data["waypoint_generator"][
                    "continuous_actions"
                ]["angular_range"]
                self.action_space = spaces.Box(
                    low=np.array([linear_range[0], angular_range[0]]),
                    high=np.array([linear_range[1], angular_range[1]]),
                    dtype=np.float,
                )

    @staticmethod
    def _calc_distance(goal_pos: Pose2D, robot_pos: Pose2D):
        y_relative = goal_pos.y - robot_pos.y
        x_relative = goal_pos.x - robot_pos.x
        rho = (x_relative ** 2 + y_relative ** 2) ** 0.5
        theta = (np.arctan2(y_relative, x_relative) - robot_pos.theta + 4 * np.pi) % (
            2 * np.pi
        ) - np.pi
        return rho, theta

    def _set_and_pub_waypoint(self, action):
        # DEBUG
        # print(f"action: {action}")
        curr_sub_goal = self.observation_collector.get_subgoal_pos()
        curr_global_goal = self.observation_collector.get_globalgoal_pos()
        _, curr_sub_goal_theta = self.observation_collector.get_subgoal_in_map_frame()
        # TODO set same to "read the range data from unified config file"
        assert curr_global_goal is not None
        assert curr_sub_goal is not None
        if self._is_action_space_discrete:
            rho = self._discrete_acitons[int(action)][0]
            theta = self._discrete_acitons[int(action)][1]
        else:
            rho, theta = action

        self._waypoint_x = curr_sub_goal.x + rho * np.cos(theta + curr_sub_goal_theta)
        self._waypoint_y = curr_sub_goal.y + rho * np.sin(theta + curr_sub_goal_theta)
        # check the new waypoint close to the goal or not
        if (self._waypoint_x - curr_global_goal.x) ** 2 + (
            curr_sub_goal.y - curr_global_goal.y
        ) ** 2 < self._goal_radius ** 2:
            self.is_waypoint_set_to_global_goal = True
            self._waypoint_x = curr_global_goal.x
            self._waypoint_y = curr_global_goal.y

        rospy.loginfo(f"sub goal pos: [x,y] = {curr_sub_goal.x} {curr_sub_goal.y} ")
        rospy.loginfo(f"waypoint pos: [x,y] = {self._waypoint_x} {self._waypoint_y}")

        action_msg = PoseStamped()
        action_msg.pose.position.x = self._waypoint_x
        action_msg.pose.position.y = self._waypoint_y
        action_msg.pose.orientation.w = 1
        action_msg.header.frame_id = "map"
        self.agent_action_pub.publish(action_msg)
        # return new_action

    def _pub_robot_pos_as_waypoint(self, robot_pos):
        """publish current robot's position as the waypoint to stop the robot to move (actually rlca can not!)
        """
        if isinstance(robot_pos, Pose2D):
            robot_x, robot_y = robot_pos.x, robot_pos.y
        else:
            robot_x, robot_y, _ = robot_pos
        action_msg = PoseStamped()
        action_msg.pose.position.x = robot_x
        action_msg.pose.position.y = robot_y
        action_msg.pose.orientation.w = 1
        action_msg.header.frame_id = "map"
        self.agent_action_pub.publish(action_msg)

    def step(self, action):
        """
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
        """
        self._set_and_pub_waypoint(action)
        # clear cache of the observation collector
        self.observation_collector.clear_on_step_start()
        # set waypoint on the observation collector and it will check whether the
        # robot already reach it.
        self.observation_collector.set_waypoint(self._waypoint_x, self._waypoint_y)
        if self._is_train_mode:
            # run the simulation and check the event and record important event
            self.observation_collector.wait_for_step_end(timeout=self._step_timeout)
        # prepare the input of the NN
        else:
            # it supposed to see new waypoint arrival there.
            self.observation_collector.wait_for_new_event()

        laserscans = self.observation_collector.get_laserscans_in_map_frame(
            num_laserscans=1
        )
        (
            subgoal_rho,
            subgoal_theta,
        ) = self.observation_collector.get_subgoal_in_map_frame()
        (
            globalgoal_rho,
            globalgoal_theta,
        ) = self.observation_collector.get_globalgoal_in_map_frame()
        merged_obs = np.concatenate(
            [
                laserscans.flatten(),
                np.array(
                    [subgoal_rho, subgoal_theta, globalgoal_rho, globalgoal_theta]
                ),
            ]
        )
        info = {"event": None,"done":None,"is_success":None}
        # if the waypoint is set to the global goal,
        # the predicted action will do nothing, so we need to set the reward to 0
        if self.is_waypoint_set_to_global_goal:
            reward = self.reward_calculator.get_reward_global_goal_reached()
            info["event"] = "GlobalGoalReached"
            done = True
            info["done"] = True
            self.reward_calculator.save_info_on_episode_end()
            info['is_success'] = True
        else:
            if (
                self.observation_collector.important_event
                == self.observation_collector.Event.TIMEOUT
            ):
                info["event"] = "Timeout"
                done = True
                info["done"] = True
                self.reward_calculator.save_info_on_episode_end()
                info['is_success'] = False
            elif (
                self.observation_collector.important_event
                == self.observation_collector.Event.COLLISIONDETECTED
            ):
                info["event"] = "Collision"
                info["done"] = True
                done = True
                self.reward_calculator.save_info_on_episode_end()
                info['is_success'] = False
            else:
                done = False
                info["done"] = False
            reward= self.reward_calculator.cal_reward()
        self._curr_steps_count += 1
        if self._curr_steps_count > self._max_steps_per_episode:
            print(f"ENV:{self.ns} STEPS {self. _curr_steps_count}")
            done = True
            info["done"] = True
            info["event"] = "MaxStepsExceed"
        reward_info = self.reward_calculator.get_reward_info()
        info.update(reward_info)

        return merged_obs, reward, done, info

    def reset(self):
        self._curr_steps_count = 0
        self.reward_calculator.reset_on_episode_start()
        if self.observation_collector.important_event is not None:
            rospy.loginfo(f"{self.ns}: {self.observation_collector.important_event}")
        if self._is_train_mode:
            remaining_try_times = 5
        else:
            remaining_try_times = 1

        while remaining_try_times >= 0:
            remaining_try_times -= 1
            self.is_waypoint_set_to_global_goal = False
            # we must clear the observation collector first to receive the new observation when reseting the task
            self.observation_collector.clear_on_episode_start()
            # Take care reset will call step world service.
            reset_info = self.task.reset(
                min_dist_start_pos_goal_pos=self._robot_start_pos_goal_pos_min_dist
            )
            robot_start_pos = reset_info["robot_start_pos"]
            robot_goal_pos = reset_info["robot_goal_pos"]
            # set the waypoint to the start position of the robot so that the robot will be freezed as we call step_world for updating the laser and other info from sensors
            self._pub_robot_pos_as_waypoint(robot_start_pos)
            # if succeed
            if self.observation_collector.reset():
                break
            print(
                f"{self.ns} reset remaining try times {remaining_try_times},curr robot: {[robot_start_pos.x,robot_start_pos.y]} goal: {[robot_goal_pos.x,robot_goal_pos.y]}"
            )
        laserscans = self.observation_collector.get_laserscans_in_map_frame(
            num_laserscans=1
        )
        (
            subgoal_rho,
            subgoal_theta,
        ) = self.observation_collector.get_subgoal_in_map_frame()
        (
            globalgoal_rho,
            globalgoal_theta,
        ) = self.observation_collector.get_globalgoal_in_map_frame(robot_goal_pos)
        merged_obs = np.concatenate(
            [
                laserscans.flatten(),
                np.array(
                    [subgoal_rho, subgoal_theta, globalgoal_rho, globalgoal_theta]
                ),
            ]
        )
        # DEBUG
        rospy.loginfo("wp_env reset done")
        return merged_obs

    def close(self):
        pass


@staticmethod
def get_distance(pose_1: Pose2D, pose_2: Pose2D):
    return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)


if __name__ == "__main__":

    rospy.init_node("wp3_gym_env", anonymous=True, disable_signals=False)
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
