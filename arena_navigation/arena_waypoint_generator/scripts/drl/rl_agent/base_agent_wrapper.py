from abc import ABC, abstractmethod
from typing import Tuple

import json
import numpy as np
import os
import rospy
import rospkg
import yaml
import math

from gym import spaces

from geometry_msgs.msg import Pose2D, PoseStamped

from arena_navigation.arena_waypoint_generator.scripts.drl.rl_agent.envs.observation import Observation
from arena_navigation.arena_waypoint_generator.scripts.drl.rl_agent.envs.reward import Reward

robot_model = rospy.get_param("model")
ROOT_ROBOT_PATH = os.path.join(
    rospkg.RosPack().get_path("simulator_setup"), "robot"
)
DEFAULT_HYPERPARAMETER = os.path.join(
    rospkg.RosPack().get_path("arena_waypoint_generator"),
    "scripts/drl/configs",
    "hyperparameters",
    "default.json",
)
DEFAULT_NUM_LASER_BEAMS, DEFAULT_LASER_RANGE = 360, 3.5
GOAL_RADIUS = 0.33


class BaseDRLAgent(ABC):
    def __init__(
        self,
        ns: str = None,
        robot_name: str = None,
        hyperparameter_path: str = DEFAULT_HYPERPARAMETER,
        *args,
        **kwargs,
    ) -> None:
        self._is_train_mode = rospy.get_param("/train_mode")

        self._ns = "" if ns is None or ns == "" else ns + "/"
        self._ns_robot = (
            self._ns if robot_name is None else self._ns + robot_name + "/"
        )
        self._robot_sim_ns = robot_name

        self.load_hyperparameters(path=hyperparameter_path)
        robot_setting_path = os.path.join(
            ROOT_ROBOT_PATH, self.robot_config_name + ".model.yaml"
        )
        self.read_setting_files(robot_setting_path)
        self.setup_action_space()
        self.setup_reward_calculator()

        self.observation = Observation(
            self._ns_robot, self._num_laser_beams, self._laser_range
        )

        # for time controlling in train mode
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")

        self._action_pub = rospy.Publisher(f"{self.ns_prefix}subgoal", PoseStamped, queue_size=1)
        self._subgoal = Pose2D()
        self.planing_horizon = 4.5
        self.subgoal_tolerance = self._agent_params["goal_radius"]

    @abstractmethod
    def setup_agent(self) -> None:
        raise NotImplementedError

    def load_hyperparameters(self, path: str) -> None:
        assert os.path.isfile(
            path
        ), f"Hyperparameters file cannot be found at {path}!"

        with open(path, "r") as file:
            hyperparams = json.load(file)

        self._agent_params = hyperparams
        self._get_robot_name_from_params()

    def read_setting_files(self, robot_setting_yaml: str) -> None:
        self._num_laser_beams = None
        self._laser_range = None
        self._robot_radius = rospy.get_param("radius") * 1.05
        with open(robot_setting_yaml, "r") as fd:
            robot_data = yaml.safe_load(fd)

            # get laser related information
            for plugin in robot_data["plugins"]:
                if plugin["type"] == "Laser":
                    laser_angle_min = plugin["angle"]["min"]
                    laser_angle_max = plugin["angle"]["max"]
                    laser_angle_increment = plugin["angle"]["increment"]
                    self._num_laser_beams = int(
                        round(
                            (laser_angle_max - laser_angle_min)
                            / laser_angle_increment
                        )
                        + 1
                    )
                    self._laser_range = plugin["range"]

        if self._num_laser_beams is None:
            self._num_laser_beams = DEFAULT_NUM_LASER_BEAMS
            print(
                f"{self._robot_sim_ns}:"
                "Wasn't able to read the number of laser beams."
                "Set to default: {DEFAULT_NUM_LASER_BEAMS}"
            )
        if self._laser_range is None:
            self._laser_range = DEFAULT_LASER_RANGE
            print(
                f"{self._robot_sim_ns}:"
                "Wasn't able to read the laser range."
                "Set to default: {DEFAULT_LASER_RANGE}"
            )

    def _get_robot_name_from_params(self):
        assert self._agent_params and self._agent_params["robot"]
        self.robot_config_name = self._agent_params["robot"]

    def setup_action_space(self) -> None:
        self._action_space = spaces.MultiDiscrete([self._agent_params["n"],2])

    def setup_reward_calculator(self) -> None:
        assert self._agent_params and "reward_fnc" in self._agent_params
        self.reward_calculator = Reward(         
            safe_dist=1.6 * self._robot_radius, 
            goal_radius=GOAL_RADIUS, 
            extended_eval=False, 
            planing_horizon=self._laser_range, 
            rule=self._agent_params["reward_fnc"],
            length=self._agent_params["n"],
        )

    @property
    def action_space(self) -> spaces.Box:
        return self._action_space

    @property
    def observation_space(self) -> spaces.Box:
        return self.observation.observation_space

    def get_observations(self) -> Tuple[np.ndarray, dict]:
        merged_obs, obs_dict = self.observation.get_observations()
        if self._agent_params["normalize"]:
            merged_obs = self.normalize_observations(merged_obs)
        return merged_obs, obs_dict

    def normalize_observations(self, merged_obs: np.ndarray) -> np.ndarray:
        assert self._agent_params["normalize"] and hasattr(
            self, "_obs_norm_func"
        )
        return self._obs_norm_func(merged_obs)

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        assert self._agent, "Agent model not initialized!"
        action = self._agent.predict(obs, deterministic=True)[0]
        return action

    def get_reward(self, obs_dict: dict) -> float:
        return self.reward_calculator.get_reward(*obs_dict)

    def publish_action(self, action: np.ndarray) -> None:
        subgoal = Pose2D()
        action_msg = PoseStamped()
        action_msg.header.stamp = rospy.Time.now()
        action_msg.header.frame_id = "map"

        _, obs_dict = self.observation.get_observations()
        robot_pose = obs_dict["robot_pose"]
        goal_pose = obs_dict["goal_pose"]
        global_plan = obs_dict["global_plan"]

        start = np.pi*self._agent_params["start"]
        a = np.linspace(-start, start, self._agent_params["n"])
        ind = int(self._agent_params["n"]/2)
        l = np.flip(a[:ind])
        r = np.array(a[ind+1:])
        angles = a[ind]
        for i, j in zip(l, r):
            angles = np.hstack((angles, [i,j]))

        if self.get_distance(goal_pose, robot_pose) <= self.planing_horizon:
            subgoal = goal_pose
        else:
            temp = self.updateSubgoalSpacialHorizon(robot_pose, global_plan)
            dist = self.get_distance(temp, robot_pose)
            alpha = np.arctan2(temp.y-robot_pose.y, temp.x-robot_pose.x)
            action_angle = alpha - angles
            points = np.array([robot_pose.x + dist*np.cos(action_angle), robot_pose.y + dist*np.sin(action_angle)])

            if self._subgoal == None:
                self._subgoal = temp
    
            if action[1] == 0:
                subgoal = self._subgoal
            elif action[1] == 1:
                subgoal.x = points[0][action[0]]
                subgoal.y = points[1][action[0]]
                self._subgoal = subgoal
        
        action_msg.pose.position.x = subgoal.x
        action_msg.pose.position.y = subgoal.y
        self._action_pub.publish(action_msg)

    def updateSubgoalSpacialHorizon(self, robot_pose: Pose2D, global_plan):
        subgoal = Pose2D()
        if len(global_plan) > 0:
            subgoal_id = 0
            for i in range(len(global_plan)):
                wp = Pose2D()
                wp.x = global_plan[i][0]
                wp.y = global_plan[i][1]
                dist_to_robot = self.get_distance(wp, robot_pose)
                if (dist_to_robot<self.planing_horizon+self.subgoal_tolerance) and (dist_to_robot>self.planing_horizon-self.subgoal_tolerance):
                    if i > subgoal_id:
                        subgoal_id = i

            subgoal.x = global_plan[subgoal_id][0]
            subgoal.y = global_plan[subgoal_id][1]
        else:
            subgoal = robot_pose

        return subgoal

    @staticmethod
    def get_distance(pose_1: Pose2D, pose_2: Pose2D):
        return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)