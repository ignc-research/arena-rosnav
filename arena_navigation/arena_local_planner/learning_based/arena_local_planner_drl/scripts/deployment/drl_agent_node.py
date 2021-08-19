#!/usr/bin/env python
from typing import Tuple

import json
import numpy as np
import os
import pickle
import rospy
import rospkg
import sys
import yaml

from stable_baselines3 import PPO

from geometry_msgs.msg import Twist

from rl_agent.utils.observation_collector import ObservationCollector


""" TEMPORARY GLOBAL CONSTANTS """
NS_PREFIX = ""
MODELS_DIR = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"), "agents"
)
ACTIONS_SETTINGS_PATH = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "config",
    "default_settings.yaml",
)
LASER_NUM_BEAMS, LASER_MAX_RANGE = 360, 3.5


class DRLAgent:
    def __init__(self, name: str, ns: str = None) -> None:
        """Initialization procedure for the DRL agent node.

        Args:
            name (str): Agent name (directory has to be of the same name)
            ns (str): Agent-specific ROS namespace
        """
        rospy.init_node(f"DRL_local_planner", anonymous=True)

        self.name = name
        self._ns = "" if ns is None or ns == "" else "/" + ns + "/"

        self.setup_agent()

        self.observation_collector = ObservationCollector(
            ns, LASER_NUM_BEAMS, LASER_MAX_RANGE
        )

        # action agent publisher
        self._is_train_mode = rospy.get_param("/train_mode")
        if self._is_train_mode:
            # w/o action publisher node
            self._action_pub = rospy.Publisher(
                f"{self._ns}cmd_vel", Twist, queue_size=1
            )
        else:
            # w/ action publisher node
            # (controls action rate being published on '../cmd_vel')
            self._action_pub = rospy.Publisher(
                f"{self._ns}cmd_vel_pub", Twist, queue_size=1
            )

    def setup_agent(self) -> None:
        """Loads the trained policy and when required the VecNormalize object"""
        model_file = os.path.join(MODELS_DIR, self.name, "best_model.zip")
        vecnorm_file = os.path.join(MODELS_DIR, self.name, "vec_normalize.pkl")
        model_params_file = os.path.join(MODELS_DIR, self.name, "hyperparameters.json")

        assert os.path.isfile(
            model_file
        ), f"Compressed model cannot be found at {model_file}!"
        assert os.path.isfile(
            vecnorm_file
        ), f"VecNormalize file cannot be found at {vecnorm_file}!"
        assert os.path.isfile(
            model_params_file
        ), f"Hyperparameter file cannot be found at {vecnorm_file}!"

        with open(vecnorm_file, "rb") as file_handler:
            vec_normalize = pickle.load(file_handler)
        with open(model_params_file, "r") as file:
            hyperparams = json.load(file)

        self._agent = PPO.load(model_file).policy
        self._obs_norm_func = vec_normalize.normalize_obs
        self._agent_params = hyperparams

        if self._agent_params["discrete_action_space"]:
            with open(ACTIONS_SETTINGS_PATH, "r") as fd:
                setting_data = yaml.safe_load(fd)
                self._discrete_actions = setting_data["robot"]["discrete_actions"]

    def get_observations(self) -> Tuple[np.ndarray, dict]:
        obs = self.observation_collector.get_observations()[0]
        if self._agent_params["normalize"]:
            obs = self._obs_norm_func(obs)
        return obs

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        action = self._agent.predict(obs, deterministic=True)[0]
        if self._agent_params["discrete_action_space"]:
            action = self._get_disc_action(action)
        return action

    def publish_action(self, action: np.ndarray) -> None:
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        self._action_pub.publish(action_msg)

    def run(self) -> None:
        while not rospy.is_shutdown():
            obs = self.get_observations()
            action = self.get_action(obs)
            self.publish_action(action)

    def _get_disc_action(self, action: int):
        return np.array(
            [
                self._discrete_actions[action]["linear"],
                self._discrete_actions[action]["angular"],
            ]
        )


if __name__ == "__main__":
    AGENT_NAME = sys.argv[1]
    AGENT = DRLAgent(AGENT_NAME, NS_PREFIX)

    try:
        AGENT.run()
    except rospy.ROSInterruptException:
        pass
