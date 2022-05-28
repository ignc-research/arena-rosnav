"""Reward Calculator for DRL"""
import numpy as np
import scipy.spatial

from geometry_msgs.msg import Pose2D
from typing import Dict, Tuple, Union


class RewardCalculator:
    def __init__(
        self,
        robot_radius: float,
        safe_dist: float,
        goal_radius: float,
        rule: str = "rule_00",
        extended_eval: bool = False,
    ):
        """A facotry class for reward calculation. Holds various reward functions.

        An overview of the reward functions can be found under:
        https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/DRL-Training.md#reward-functions

        Possible reward functions: "_rule_00_", "_rule_01_", "_rule_02_", "_rule_03_", "_rule_04_"

        Args:
            robot_radius (float): Robots' radius in meters.
            safe_dist (float): Robots' safe distance in meters.
            goal_radius (float): Radius of the goal.
            rule (str, optional): The desired reward function name. Defaults to "rule_00".
            extended_eval (bool, optional): Extended evaluation mode. Defaults to False.
        """
        self.curr_reward = 0
        # additional info will be stored here and be returned alonge with reward.
        self.info = {}
        self.robot_radius = robot_radius
        self.goal_radius = goal_radius
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.last_action = None
        self.safe_dist = robot_radius + safe_dist
        self._extended_eval = extended_eval

        self.kdtree = None

        self._cal_funcs = {
            "rule_00": RewardCalculator._cal_reward_rule_00,
            "rule_01": RewardCalculator._cal_reward_rule_01,
            "rule_02": RewardCalculator._cal_reward_rule_02,
            "rule_03": RewardCalculator._cal_reward_rule_03,
            "rule_04": RewardCalculator._cal_reward_rule_04,
        }
        self.cal_func = self._cal_funcs[rule]

    def reset(self) -> None:
        """Resets variables related to the episode."""
        self.last_goal_dist = None
        self.last_dist_to_path = None
        self.last_action = None
        self.kdtree = None

    def _reset(self) -> None:
        """Resets variables related to current step."""
        self.curr_reward = 0
        self.info = {}

    def get_reward(
        self,
        laser_scan: np.ndarray,
        goal_in_robot_frame: Tuple[float, float],
        *args,
        **kwargs
    ) -> Tuple[float, Dict[str, Union[str, int, bool]]]:
        """Returns reward and info to the gym environment.

        Args:
            laser_scan (np.ndarray): 2D laser scan data.
            goal_in_robot_frame (Tuple[float, float]): Position (rho, theta) of the goal in the robot frame (polar coordinate).

        Returns:
            Tuple[float, Dict[str, Union[str, int, bool]]]: Tuple of calculated rewards for the current step, \
                and the reward information dictionary.
        """
        self._reset()
        self.cal_func(self, laser_scan, goal_in_robot_frame, *args, **kwargs)
        return self.curr_reward, self.info

    def _cal_reward_rule_00(
        self,
        laser_scan: np.ndarray,
        goal_in_robot_frame: Tuple[float, float],
        *args,
        **kwargs
    ):
        """Reward function: '_rule\_00_'

        Description:
            "rule_00" incorporates the most instinctive characteristics for learning navigation into its \
            reward calculation. The reward function is made up of only 4 summands, namely the success \
            reward, the collision reward, the danger reward and the progress reward. Similar reward functions \
            were utilized in numerous research projects and produced promising results. Thus, this \
            rule is chosen to be the basis for further experiments with extended versions of it. \

        Args:
            laser_scan (np.ndarray): 2D laser scan data.
            goal_in_robot_frame (Tuple[float, float]): Position (rho, theta) of the goal in the robot frame (polar coordinate).
        """
        self._reward_goal_reached(goal_in_robot_frame)
        self._reward_safe_dist(laser_scan, punishment=0.25)
        self._reward_collision(laser_scan)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4
        )

    def _cal_reward_rule_01(
        self,
        laser_scan: np.ndarray,
        goal_in_robot_frame: Tuple[float, float],
        *args,
        **kwargs
    ):
        """Reward function: '_rule\_01_'
        
        Description:
            This reward function extends "rule 00" by adding a penalty factor that affects the current \
            reward like an abstract fuel consumption factor. In principle, a certain penalty is applied \
            for each action taken depending on the velocity and thus imposes a severer punishment for \
            dissipated driving.

        Args:
            laser_scan (np.ndarray): 2D laser scan data.
            goal_in_robot_frame (Tuple[float, float]): Position (rho, theta) of the goal in the robot frame (polar coordinate).
        """
        self._reward_distance_traveled(
            kwargs["action"], consumption_factor=0.0075
        )
        self._reward_goal_reached(goal_in_robot_frame, reward=15)
        self._reward_safe_dist(laser_scan, punishment=0.25)
        self._reward_collision(laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4
        )

    def _cal_reward_rule_02(
        self,
        laser_scan: np.ndarray,
        goal_in_robot_frame: Tuple[float, float],
        *args,
        **kwargs
    ):
        """Reward function: '_rule\_02_'
        
        Description:
            Previous reward functions required only basic information from the simulation. For this rule, \
            which builds on the reward function "rule 01", we introduced the assessment of the progress \
            regarding the global plan. The additional summand essentially rewards the agent for following \
            the global plan. It was implemented in order to test the effect of including the global plan in \
            the reward calculation. \
            Since "rule 02" shares almost the same reward function composition as "rule 01", similar performance \
            was expected to some extent. The desired behavior for this agent was to learn faster and \
            to drive more goal-oriented than the agent of "rule 01", as this rule was provided the global plan. \

        Args:
            laser_scan (np.ndarray): 2D laser scan data.
            goal_in_robot_frame (Tuple[float, float]): Position (rho, theta) of the goal in the robot frame (polar coordinate).
        """
        self._reward_distance_traveled(
            kwargs["action"], consumption_factor=0.0075
        )
        self._reward_following_global_plan(
            kwargs["global_plan"], kwargs["robot_pose"]
        )
        self._reward_goal_reached(goal_in_robot_frame, reward=15)
        self._reward_safe_dist(laser_scan, punishment=0.25)
        self._reward_collision(laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4
        )

    def _cal_reward_rule_03(
        self,
        laser_scan: np.ndarray,
        goal_in_robot_frame: Tuple[float, float],
        *args,
        **kwargs
    ):
        """Reward function: '_rule\_03_'

        Description:
            The base of this rule is made up of summands from "rule 00". The two extra factors were \
            introduced in order to further leverage the global plan information for reward generation. \
            One that rewards the following of the global path and one for valuing the agents’ action - \
            positively, when it approaches the global plan - negatively when the robot distances itself \
            from the path. \

        Args:
            laser_scan (np.ndarray): 2D laser scan data. \
            goal_in_robot_frame (Tuple[float, float]): Position (rho, theta) of the goal in the robot frame (polar coordinate). \
        """
        self._reward_following_global_plan(
            kwargs["global_plan"], kwargs["robot_pose"], kwargs["action"]
        )
        if laser_scan.min() > self.safe_dist:
            self._reward_distance_global_plan(
                kwargs["global_plan"],
                kwargs["robot_pose"],
                reward_factor=0.2,
                penalty_factor=0.3,
            )
        else:
            self.last_dist_to_path = None
        self._reward_goal_reached(goal_in_robot_frame, reward=15)
        self._reward_safe_dist(laser_scan, punishment=0.25)
        self._reward_collision(laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4
        )

    def _cal_reward_rule_04(
        self,
        laser_scan: np.ndarray,
        goal_in_robot_frame: Tuple[float, float],
        *args,
        **kwargs
    ):
        """Reward function: '_rule\_04_'

        Description:
            This reward function extends "rule 03" with an additional term that punishes the agent for \
            abruptly changing the direction. Previous test runs, conducted right after the implementation, \
            evidenced that although the agent performed well on different tasks, the robot tended to drive \
            in tail motion. It was aimed to adjust this behavior by including this additional penalty term. \

        Args:
            laser_scan (np.ndarray): 2D laser scan data.
            goal_in_robot_frame (Tuple[float, float]): Position (rho, theta) of the goal in the robot frame (polar coordinate).
        """
        self._reward_abrupt_direction_change(kwargs["action"])
        self._reward_following_global_plan(
            kwargs["global_plan"], kwargs["robot_pose"], kwargs["action"]
        )
        if laser_scan.min() > self.safe_dist:
            self._reward_distance_global_plan(
                kwargs["global_plan"],
                kwargs["robot_pose"],
                reward_factor=0.2,
                penalty_factor=0.3,
            )
        else:
            self.last_dist_to_path = None
        self._reward_goal_reached(goal_in_robot_frame, reward=15)
        self._reward_safe_dist(laser_scan, punishment=0.25)
        self._reward_collision(laser_scan, punishment=10)
        self._reward_goal_approached(
            goal_in_robot_frame, reward_factor=0.3, penalty_factor=0.4
        )

    def _reward_goal_reached(
        self, goal_in_robot_frame: Tuple[float, float], reward: float = 15
    ):
        """Reward for reaching the goal.

        Args:
            goal_in_robot_frame (Tuple[float, float], optional): Position (rho, theta) of the goal in the robot frame (polar coordinate).
            reward (float, optional): Reward amount for reaching the goal. Defaults to 15.
        """
        if goal_in_robot_frame[0] < self.goal_radius:
            self.curr_reward = reward
            self.info["is_done"] = True
            self.info["done_reason"] = 2
            self.info["is_success"] = 1
        else:
            self.info["is_done"] = False

    def _reward_goal_approached(
        self,
        goal_in_robot_frame=Tuple[float, float],
        reward_factor: float = 0.3,
        penalty_factor: float = 0.5,
    ):
        """Reward for approaching the goal.

        Args:
            goal_in_robot_frame ([type], optional): Position (rho, theta) of the goal in the robot frame (polar coordinate). Defaults to Tuple[float, float].
            reward_factor (float, optional): Factor to be multiplied when the difference between current distance to goal and the previous one is positive. \
                Defaults to 0.3.
            penalty_factor (float, optional): Factor to be multiplied when the difference between current distance to goal and the previous one is negative. Defaults to 0.5.
        """
        if self.last_goal_dist is not None:
            # goal_in_robot_frame : [rho, theta]

            # higher negative weight when moving away from goal
            # (to avoid driving unnecessary circles when train in contin. action space)
            if (self.last_goal_dist - goal_in_robot_frame[0]) > 0:
                w = reward_factor
            else:
                w = penalty_factor
            reward = w * (self.last_goal_dist - goal_in_robot_frame[0])

            # print("reward_goal_approached:  {}".format(reward))
            self.curr_reward += reward
        self.last_goal_dist = goal_in_robot_frame[0]

    def _reward_collision(self, laser_scan: np.ndarray, punishment: float = 10):
        """Reward for colliding with an obstacle.

        Args:
            laser_scan (np.ndarray): 2D laser scan data.
            punishment (float, optional): Punishment amount for collisions. Defaults to 10.
        """
        if laser_scan.min() <= self.robot_radius:
            self.curr_reward -= punishment

            if not self._extended_eval:
                self.info["is_done"] = True
                self.info["done_reason"] = 1
                self.info["is_success"] = 0
            else:
                self.info["crash"] = True

    def _reward_safe_dist(
        self, laser_scan: np.ndarray, punishment: float = 0.15
    ):
        """Reward for undercutting safe distance.

        Args:
            laser_scan (np.ndarray): 2D laser scan data.
            punishment (float, optional): Punishment amount. Could be applied in consecutive timesteps. \
                Defaults to 0.15.
        """
        if laser_scan.min() < self.safe_dist:
            self.curr_reward -= punishment

            if self._extended_eval:
                self.info["safe_dist"] = True

    def _reward_not_moving(
        self, action: np.ndarray = None, punishment: float = 0.01
    ):
        """Reward for not moving. 

        Args:
            action (np.ndarray, optional): Array of shape (2,). First entry, linear velocity. \
                Second entry, angular velocity. Defaults to None.
            punishment (float, optional): Punishment for not moving. Defaults to 0.01.
            
        Note:
            Only applies half of the punishment amount when angular velocity is larger than zero.
        """
        if action is not None and action[0] == 0.0:
            self.curr_reward -= (
                punishment if action[1] == 0.0 else punishment / 2
            )

    def _reward_distance_traveled(
        self,
        action: np.array = None,
        punishment: float = 0.01,
        consumption_factor: float = 0.005,
    ):
        """Reward for driving a certain distance. Supposed to represent "fuel consumption".

        Args:
            action (np.array, optional): Array of shape (2,). First entry, linear velocity. \
                Second entry, angular velocity. Defaults to None.
            punishment (float, optional): Punishment when action can't be retrieved. Defaults to 0.01.
            consumption_factor (float, optional): Factor for the weighted velocity punishment. Defaults to 0.005.
        """
        if action is None:
            self.curr_reward -= punishment
        else:
            lin_vel = action[0]
            ang_vel = action[1]
            reward = (lin_vel + (ang_vel * 0.001)) * consumption_factor
        self.curr_reward -= reward

    def _reward_distance_global_plan(
        self,
        global_plan: np.array,
        robot_pose: Pose2D,
        reward_factor: float = 0.1,
        penalty_factor: float = 0.15,
    ):
        """Reward for approaching/veering away the global plan.

        Description:
            Weighted difference between prior distance to global plan and current distance to global plan.

        Args:
            global_plan (np.array): Array containing 2D poses.
            robot_pose (Pose2D): Robot position.
            reward_factor (float, optional): Factor to be multiplied when the difference between current \
                distance to global plan and the previous one is positive. Defaults to 0.1.
            penalty_factor (float, optional): Factor to be multiplied when the difference between current \
                distance to global plan and the previous one is negative. Defaults to 0.15.
        """
        if global_plan is not None and len(global_plan) != 0:
            curr_dist_to_path, idx = self.get_min_dist2global_kdtree(
                global_plan, robot_pose
            )

            if self.last_dist_to_path is not None:
                if curr_dist_to_path < self.last_dist_to_path:
                    w = reward_factor
                else:
                    w = penalty_factor

                self.curr_reward += w * (
                    self.last_dist_to_path - curr_dist_to_path
                )
            self.last_dist_to_path = curr_dist_to_path

    def _reward_following_global_plan(
        self,
        global_plan: np.array,
        robot_pose: Pose2D,
        action: np.array = None,
        dist_to_path: float = 0.5,
    ):
        """Reward for travelling along the global plan.

        Args:
            global_plan (np.array): Array containing 2D poses.
            robot_pose (Pose2D): Robot position.
            action (np.array, optional): action (np.ndarray, optional): Array of shape (2,). First entry, linear velocity. \
                Second entry, angular velocity. Defaults to None.
            dist_to_path (float, optional): Minimum distance to the global path. Defaults to 0.5.
        """
        if (
            global_plan is not None
            and len(global_plan) != 0
            and action is not None
        ):
            curr_dist_to_path, idx = self.get_min_dist2global_kdtree(
                global_plan, robot_pose
            )

            if curr_dist_to_path <= dist_to_path:
                self.curr_reward += 0.1 * action[0]

    def get_min_dist2global_kdtree(
        self, global_plan: np.array, robot_pose: Pose2D
    ) -> Tuple[float, int]:
        """Calculates minimal distance to global plan using kd-tree-search.

        Args:
            global_plan (np.array): Array containing 2D poses.
            robot_pose (Pose2D): Robot position.

        Returns:
            Tuple[float, int]: Distance to the closes pose and index of the closes pose.
        """
        if self.kdtree is None:
            self.kdtree = scipy.spatial.cKDTree(global_plan)

        dist, index = self.kdtree.query([robot_pose.x, robot_pose.y])
        return dist, index

    def _reward_abrupt_direction_change(self, action: np.array = None):
        """Applies a penalty when an abrupt change of direction occured.

        Args:
            action (np.array, optional): Array of shape (2,). First entry, linear velocity. \
                Second entry, angular velocity. Defaults to None.
        """
        if self.last_action is not None:
            curr_ang_vel = action[1]
            last_ang_vel = self.last_action[1]

            vel_diff = abs(curr_ang_vel - last_ang_vel)
            self.curr_reward -= (vel_diff ** 4) / 2500
        self.last_action = action
