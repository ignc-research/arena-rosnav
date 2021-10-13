#! /usr/bin/env python
from random import randint
import gym
from gym import spaces
from gym.utils import colorize
from gym.spaces import space
from typing import Callable, Iterable, Optional, Union
import threading
from numpy.lib.function_base import angle
from rospy.core import rospywarn
from stable_baselines3.common.env_checker import check_env
import yaml
from rl_agent.utils.observation_collector import (
    ObservationCollectorWP2,
    ObservationCollectorWP3,
)
from rl_agent.utils.debug import timeit
from task_generator.tasks import ABSTask
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from flatland_msgs.srv import StepWorld, StepWorldRequest
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as nav_Path
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
import time
import math
from rl_agent.utils.debug import timeit, NPPSERVER

from arena_plan_msgs.msg import RobotState, RobotStateStamped

from ..config import configurable
from .build import CfgNode, ENV_REGISTRY
from ..utils.reward import build_reward_calculator, REWARD_REGISTRY


class ExpertActionCollector(object):
    """[summary]

    Args:
        object ([type]): [description]
    """

    def __init__(self, env: "WPEnvMapFrame3"):
        if env._is_action_space_discrete:
            self._discrete_actions = env._discrete_acitons_np
            self._discrete_actions_theta = np.arctan2(
                self._discrete_actions[:, 1], self._discrete_actions[:, 0]
            )
        else:
            self._action_x_range = env._action_x_range
            self._action_y_range = env._action_y_range
            raise NotImplementedError("Not fully implemented")
        self._expert_action_sub = rospy.Subscriber(
            "/expert_action", PoseStamped, self.expert_action_callback, tcp_nodelay=True
        )
        self._expert_action_vis_pub = rospy.Publisher(
            "/anno_vis/expert_action", Marker, tcp_nodelay=True
        )
        self.raw_expert_pos_x, self.raw_expert_pos_y = 0, 0
        self.action_cv = threading.Condition()

    def get_expert_action(self, robot_x, robot_y) -> int:
        """[summary]

        Args:
            robot_x ([type]): [description]
            robot_y ([type]): [description]

        Returns:
            int: [description]
        """
        # TODO_D
        try_times = 600
        while try_times > 0:
            try_times -= 1

            with self.action_cv:
                if not self.action_cv.wait(timeout=10):
                    print(
                        colorize(
                            "Please press <e> and left-click the map to assign the expert action",
                            "green",
                        )
                    )
                else:
                    raw_expert_action_x, raw_expert_action_y = (
                        self.raw_expert_pos_x - robot_x,
                        self.raw_expert_pos_y - robot_y,
                    )
                    idx = np.argmin(
                        np.abs(
                            self._discrete_actions_theta
                            - np.arctan2(raw_expert_action_y, raw_expert_action_x)
                        )
                    )
                    x, y = self._discrete_actions[idx]

                    self.draw_expert_action(robot_x + x, robot_y + y)
                    return raw_expert_action_x, raw_expert_action_y, idx
        raise ValueError("User no reaction on rviz")

    def expert_action_callback(self, pos_msg: PoseStamped):
        with self.action_cv:
            self.raw_expert_pos_x = pos_msg.pose.position.x
            self.raw_expert_pos_y = pos_msg.pose.position.y
            self.action_cv.notify()

    def draw_expert_action(self, x, y):
        marker = Marker()
        marker = Marker()
        marker.header.stamp = rospy.get_rostime()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.id = 1000

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(100)

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 235
        marker.color.g = 52
        marker.color.b = 204
        marker.color.a = 1

        marker.pose.position.x = x
        marker.pose.position.y = y

        marker.scale.z = 1

        # marker.text = "expert_action_pos"
        self._expert_action_vis_pub.publish(marker)


@ENV_REGISTRY.register()
class WPEnvMapFrame3(gym.Env):
    """A ENV use global path, laser scan obstatles states as observation
    """

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
        stage_dynamic_obstacle,
        num_poses_global_path,
        is_pretrain_mode_on: bool = False,
        terminate_on_timeout:bool = True
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
        self._is_pretrain_mode_on = is_pretrain_mode_on
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            ns_int = int(ns.split("_")[-1])
            self._ns_int = ns_int
            time.sleep(ns_int * 2)
        except Exception:
            if not is_pretrain_mode_on:
                rospy.logwarn(
                    f"Can't not determinate the number of the environment, training script may crash!"
                )

        # process specific namespace in ros system
        self.ns_prefix = "" if (ns == "" or ns is None) else "/" + ns + "/"
        self._is_action_space_discrete = is_action_space_discrete
        if debug:
            log_level = rospy.INFO
        else:
            log_level = rospy.WARN

        self._is_train_mode = rospy.get_param("/train_mode")

        if callable(task):
            task = task()
        self.task: "ABSTask" = task

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
        self._terminate_on_timeout = terminate_on_timeout
        if self._is_pretrain_mode_on:
            self.expert_action_collector = ExpertActionCollector(self)

        # observation collector
        self.observation_collector = ObservationCollectorWP3(
            self.ns,
            self._is_train_mode,
            self._laser_num_beams,
            self._laser_angle_increment,
            self._laser_max_range,
            self._robot_waypoint_min_dist,
            self._robot_obstacle_min_dist,
            stage_dynamic_obstacle,
            0.3,  # hardcoded
            self._robot_radius,
            10,
            self._is_pretrain_mode_on,
            num_poses_global_path,
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
        if self._is_pretrain_mode_on:
            self._last_dist_subgoal_goal_squre = None
            self._pretrain_early_stop_curr_episode = False

        self.agent_action_pub = rospy.Publisher(
            f"{self.ns_prefix}waypoint", PoseStamped, queue_size=1, tcp_nodelay=True
        )
        # #DEBUG_LASER
        # if len(ns) and ns[-1]>='0' and ns[-1]<='9'and int(ns[-1]) == 1:
        #     self.laser_server = NPPSERVER(35555)
        # else:
        #     self.laser_server = None

    @classmethod
    def from_config(cls, cfg: CfgNode, task, ns, train_mode, debug, **kwargs):

        # TODO The code here is very dirty, later on we will put reward related
        # stuffes in other places
        assert not cfg.INPUT.NORM, "Pretrained VAE unsupport normalized input"
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
        stage_dynamic_obstacle = cfg.EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE
        terminate_on_timeout = cfg.TRAINING.TERMINATE_ON_TIMEOUT
        if "is_pretrain_mode_on" in kwargs:
            is_pretrain_mode_on = kwargs["is_pretrain_mode_on"]
        else:
            is_pretrain_mode_on = False
        num_poses_global_path = 10
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
            stage_dynamic_obstacle=stage_dynamic_obstacle,
            is_pretrain_mode_on=is_pretrain_mode_on,
            num_poses_global_path=num_poses_global_path,
            terminate_on_timeout = terminate_on_timeout,
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
                                lambda angular, linear=linear: [
                                    linear * np.cos(angular),
                                    linear * np.sin(angular),
                                ],
                                angulars,
                            )
                        )
                    else:
                        raise NotImplementedError("")
                        # self._discrete_acitons.append(
                        #     [
                        #         actions_def["linear"]*np.cos(),
                        #         eval(actions_def["angular"])
                        #         if isinstance(actions_def["angular"], str)
                        #         else actions_def["angular"],
                        #     ]
                        # )
                # np cache
                self._discrete_acitons_np = np.array(self._discrete_acitons)
                self.action_space = spaces.Discrete(len(self._discrete_acitons))
            else:
                # TODO  read the range data from unified config file
                x_range = setting_data["waypoint_generator"]["continuous_actions"][
                    "x_range"
                ]
                y_range = setting_data["waypoint_generator"]["continuous_actions"][
                    "y_range"
                ]
                self.action_space = spaces.Box(
                    low=np.array([x_range[0], y_range[0]]),
                    high=np.array([x_range[1], y_range[1]]),
                    dtype=np.float,
                )
                self._action_x_range = x_range
                self._action_y_range = y_range

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
        robot_pose2d = self.observation_collector.robot_pose2d
        ref_pos = self.observation_collector._ref_pos
        assert robot_pose2d is not None and ref_pos is not None
        robot_x, robot_y = robot_pose2d.x, robot_pose2d.y
        if self._is_action_space_discrete:
            x = self._discrete_acitons[int(action)][0]
            y = self._discrete_acitons[int(action)][1]
        else:
            x, y = action

        self._waypoint_x = robot_x + x
        self._waypoint_y = robot_y + y
        # check the new waypoint close to the goal or not
        if (self._waypoint_x - ref_pos.x) ** 2 + (
            self._waypoint_y - ref_pos.y
        ) ** 2 < self._goal_radius ** 2:
            self.is_waypoint_set_to_global_goal = True
            self._waypoint_x = ref_pos.x
            self._waypoint_y = ref_pos.y

        # rospy.loginfo(f"sub goal pos: [x,y] = {curr_sub_goal.x} {curr_sub_goal.y} ")
        # rospy.loginfo(f"waypoint pos: [x,y] = {self._waypoint_x} {self._waypoint_y}")

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

    def _pretrain_mode_move_robot_to_sub_goal(self, min_dist_subgoal_goal=0.5):

        global_goal = self.goal_pos
        sub_goal = self.observation_collector._subgoal
        assert global_goal is not None
        assert sub_goal is not None
        dist_squre = (sub_goal.x - global_goal.x) ** 2 + (
            sub_goal.y - global_goal.y
        ) ** 2
        if (
            self._last_dist_subgoal_goal_squre is not None
            and abs(dist_squre - self._last_dist_subgoal_goal_squre) < 0.1
        ):
            self._pretrain_early_stop_curr_episode = True
        self._last_dist_subgoal_goal_squre = dist_squre
        if dist_squre < min_dist_subgoal_goal ** 2:
            sub_goal = global_goal
            self.is_waypoint_set_to_global_goal = True
        robot_manager = self.task.robot_manager
        robot_manager.move_robot(sub_goal)
        self._waypoint_x = sub_goal.x
        self._waypoint_y = sub_goal.y

    def step(self, action):
        """
        done_reasons:   0   -   exceeded max steps
                        1   -   collision with obstacle
                        2   -   goal reached
        """
        # something wrong with pretraining, don't have time to check it 
        action = (action+10)%20
        if not self._is_pretrain_mode_on:
            self._set_and_pub_waypoint(action)
        else:
            # waypoint is also set to subgoal
            (
                raw_expert_action_x,
                raw_expert_action_y,
                idx_action,
            ) = self.expert_action_collector.get_expert_action(
                self._last_robot_x, self._last_robot_y
            )
            expert_action_x, expert_action_y = self._discrete_acitons_np[idx_action]
            self._waypoint_x, self._waypoint_y = (
                expert_action_x + self._last_robot_x,
                expert_action_y + self._last_robot_y,
            )
            self._set_and_pub_waypoint(idx_action)

        # set waypoint on the observation collector and it will check whether the
        # robot already reach it.
        self.observation_collector.set_waypoint(self._waypoint_x, self._waypoint_y)
        if self._is_pretrain_mode_on or self._is_train_mode:
            # run the simulation and check the event and record important event
            self.observation_collector.wait_for_step_end(timeout=self._step_timeout)
        # prepare the input of the NN
        else:
            # it supposed to see new waypoint arrival there.
            self.observation_collector.wait_for_new_event()
        info = {"event": None, "done": None, "is_success": None}
        merged_obs = self.observation_collector.get_observation()

        if self._is_pretrain_mode_on:
            robot_curr_pos = self.observation_collector.get_robot_pos()
            self._last_robot_x, self._last_robot_y = robot_curr_pos.x, robot_curr_pos.y
            if not self._is_action_space_discrete:
                raise NotImplementedError()
            else:
                info["expert_action"] = [
                    raw_expert_action_x,
                    raw_expert_action_y,
                    idx_action,
                ]

        # if the waypoint is set to the global goal,
        # the predicted action will do nothing, so we need to set the reward to 0
        if (
            self.is_waypoint_set_to_global_goal
            or self._is_pretrain_mode_on
            and self._pretrain_early_stop_curr_episode
        ):
            reward = self.reward_calculator.get_reward_goal_reached()
            info["event"] = "GlobalGoalReached"
            done = True
            info["done"] = True
            self.reward_calculator.save_info_on_episode_end()
            info["is_success"] = True
        else:
            if (
                self.observation_collector.important_event
                == self.observation_collector.Event.TIMEOUT
            ):
                # DEBUG
                info["event"] = "Timeout"
                if self._terminate_on_timeout:     
                    done = True
                    info["done"] = True
                    self.reward_calculator.save_info_on_episode_end()
                else:
                    done = False
                    info["done"] = False
                info["is_success"] = False
            elif (
                self.observation_collector.important_event
                == self.observation_collector.Event.COLLISIONDETECTED
            ):
                info["event"] = "Collision"
                # DEBUG
                # info["done"] = False
                # info["done"] = False
                # done = False
                info["done"] = True
                done = True
                self.reward_calculator.save_info_on_episode_end()
                info["is_success"] = False
            else:
                done = False
                info["done"] = False

            reward = self.reward_calculator.cal_reward()
        self._curr_steps_count += 1
        if self._curr_steps_count > self._max_steps_per_episode:
            print(f"ENV:{self.ns} STEPS {self. _curr_steps_count}")
            done = True
            info["done"] = True
            info["event"] = "MaxStepsExceed"
        reward_info = self.reward_calculator.get_reward_info()
        info.update(reward_info)
        # DEBUG_LASER
        # if self.laser_server:
        #     self.laser_server.send_nparray(merged_obs[:360])

        return merged_obs, reward, done, info

    def reset(self):
        self._curr_steps_count = 0
        self.reward_calculator.reset_on_episode_start()

        if self._is_train_mode:
            if self._is_pretrain_mode_on:
                self._last_dist_subgoal_goal_squre = None
                self._pretrain_early_stop_curr_episode = False
                remaining_try_times = 20
            else:
                remaining_try_times = 5
        else:
            remaining_try_times = 1

        while remaining_try_times >= 0:
            remaining_try_times -= 1
            self.observation_collector.clear_on_episode_start()
            self.is_waypoint_set_to_global_goal = False
            # Take care reset will call step world service.
            reset_info = self.task.reset(
                min_dist_start_pos_goal_pos=self._robot_start_pos_goal_pos_min_dist
            )
            robot_start_pos = reset_info["robot_start_pos"]
            robot_goal_pos = reset_info["robot_goal_pos"]
            # plan manager is not stable, the publication of the goal is sometimes missing
            self.goal_pos = robot_goal_pos
            self.observation_collector._ref_pos = Pose2D(
                robot_goal_pos.x, robot_goal_pos.y, robot_goal_pos.theta
            )
            # set the waypoint to the start position of the robot so that the robot will be freezed as we call step_world for updating the laser and other info from sensors
            self._pub_robot_pos_as_waypoint(robot_start_pos)

            # if succeed
            if self._is_pretrain_mode_on and remaining_try_times < 5:
                super_long_step = True
            else:
                super_long_step = False
            if self.observation_collector.reset(super_long_step=super_long_step):
                break
            if remaining_try_times < 2:
                print(
                    f"{self.ns} reset remaining try times {remaining_try_times},curr robot: {[robot_start_pos.x,robot_start_pos.y]} goal: {[robot_goal_pos.x,robot_goal_pos.y]}"
                )
        if self._is_pretrain_mode_on:
            self._last_robot_x, self._last_robot_y = (
                robot_start_pos.x,
                robot_start_pos.y,
            )
        self.observation_collector.get_global_path(
            robot_start_pos.x, robot_start_pos.y, robot_goal_pos.x, robot_goal_pos.y
        )
        merged_obs = self.observation_collector.get_observation()
        # DEBUG
        rospy.loginfo("wp_env reset done")
        # DEBUG_LASER
        # if self.laser_server:
        #     self.laser_server.send_nparray(merged_obs[:360])
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
