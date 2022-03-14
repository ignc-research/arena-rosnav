from abc import ABC, abstractmethod
from typing import Tuple

import json
import numpy as np
import os
import rospy
import rospkg
import yaml

from gym import spaces

from geometry_msgs.msg import Twist

from rl_agent.utils.observation_collector import ObservationCollector
from rl_agent.utils.reward import RewardCalculator

robot_model = rospy.get_param("model")
ROOT_ROBOT_PATH = os.path.join(
    rospkg.RosPack().get_path("simulator_setup"), "robot"
)
DEFAULT_ACTION_SPACE = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    f"default_settings_{robot_model}.yaml",
)
DEFAULT_HYPERPARAMETER = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
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
        action_space_path: str = DEFAULT_ACTION_SPACE,
        *args,
        **kwargs,
    ) -> None:
        """[summary]

        Args:
            ns (str, optional):
                Agent name (directory has to be of the same name). Defaults to None.
            robot_name (str, optional):
                Robot specific ROS namespace extension. Defaults to None.
            hyperparameter_path (str, optional):
                Path to json file containing defined hyperparameters.
                Defaults to DEFAULT_HYPERPARAMETER.
            action_space_path (str, optional):
                Path to yaml file containing action space settings.
                Defaults to DEFAULT_ACTION_SPACE.
        """
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
        self.read_setting_files(robot_setting_path, action_space_path)
        self.setup_action_space()
        self.setup_reward_calculator()

        self.observation_collector = ObservationCollector(
            self._ns_robot, self._num_laser_beams, self._laser_range
        )

        # for time controlling in train mode
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")

        if self._is_train_mode:
            # w/o action publisher node
            self._action_pub = rospy.Publisher(
                f"{self._ns_robot}cmd_vel", Twist, queue_size=1
            )
        else:
            # w/ action publisher node
            # (controls action rate being published on '../cmd_vel')
            self._action_pub = rospy.Publisher(
                f"{self._ns_robot}cmd_vel_pub", Twist, queue_size=1
            )

    @abstractmethod
    def setup_agent(self) -> None:
        """Sets up the new agent / loads a pretrained one.

        Raises:
            NotImplementedError: Abstract method.
        """
        raise NotImplementedError

    def load_hyperparameters(self, path: str) -> None:
        """Loads the hyperparameters from a json file.

        Args:
            path (str): Path to the json file.
        """
        assert os.path.isfile(
            path
        ), f"Hyperparameters file cannot be found at {path}!"

        with open(path, "r") as file:
            hyperparams = json.load(file)

        self._agent_params = hyperparams
        self._get_robot_name_from_params()
        rospy.set_param(
            "actions_in_obs",
            self._agent_params.get("actions_in_observationspace", False),
        )
        import rl_agent.model.custom_policy
        import rl_agent.model.custom_sb3_policy

    def read_setting_files(
        self, robot_setting_yaml: str, action_space_yaml: str
    ) -> None:
        """Retrieves the robot radius (in 'self._robot_radius'), \
            laser scan range (in 'self._laser_range') and \
            the action space from respective yaml file.

        Args:
            robot_setting_yaml (str): 
                Yaml file containing the robot specific settings. 
            action_space_yaml (str): 
                Yaml file containing the action space configuration. 
        """
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

        with open(action_space_yaml, "r") as fd:
            setting_data = yaml.safe_load(fd)

            self._holonomic = setting_data["robot"]["holonomic"]
            self._discrete_actions = setting_data["robot"]["discrete_actions"]
            self._cont_actions = {
                "linear_range": setting_data["robot"]["continuous_actions"][
                    "linear_range"
                ],
                "angular_range": setting_data["robot"]["continuous_actions"][
                    "angular_range"
                ],
            }

    def _get_robot_name_from_params(self):
        """Retrives the agent-specific robot name from the dictionary loaded\
            from respective 'hyperparameter.json'.    
        """
        assert self._agent_params and self._agent_params["robot"]
        self.robot_config_name = self._agent_params["robot"]

    def setup_action_space(self) -> None:
        """Sets up the action space. (spaces.Box)"""
        assert self._discrete_actions or self._cont_actions
        assert (
            self._agent_params and "discrete_action_space" in self._agent_params
        )

        if self._agent_params["discrete_action_space"]:
            # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
            assert (
                not self._holonomic
            ), "Discrete action space currently not supported for holonomic robots"

            self.action_space = spaces.Discrete(len(self._discrete_actions))
        else:
            linear_range = self._cont_actions["linear_range"].copy()
            angular_range = self._cont_actions["angular_range"].copy()

            if not self._holonomic:
                self._action_space = spaces.Box(
                    low=np.array([linear_range[0], angular_range[0]]),
                    high=np.array([linear_range[1], angular_range[1]]),
                    dtype=np.float32,
                )
            else:
                linear_range_x, linear_range_y = (
                    linear_range["x"],
                    linear_range["y"],
                )
                self._action_space = spaces.Box(
                    low=np.array(
                        [
                            linear_range_x[0],
                            linear_range_y[0],
                            angular_range[0],
                        ]
                    ),
                    high=np.array(
                        [
                            linear_range_x[1],
                            linear_range_y[1],
                            angular_range[1],
                        ]
                    ),
                    dtype=np.float,
                )

    def setup_reward_calculator(self) -> None:
        """Sets up the reward calculator."""
        assert self._agent_params and "reward_fnc" in self._agent_params
        self.reward_calculator = RewardCalculator(
            holonomic=self._holonomic,
            robot_radius=self._robot_radius,
            safe_dist=1.6 * self._robot_radius,
            goal_radius=GOAL_RADIUS,
            rule=self._agent_params["reward_fnc"],
            extended_eval=False,
        )

    @property
    def action_space(self) -> spaces.Box:
        """Returns the DRL agent's action space.

        Returns:
            spaces.Box: Agent's action space
        """
        return self._action_space

    @property
    def observation_space(self) -> spaces.Box:
        """Returns the DRL agent's observation space.

        Returns:
            spaces.Box: Agent's observation space
        """
        return self.observation_collector.observation_space

    def get_observations(self) -> Tuple[np.ndarray, dict]:
        """Retrieves the latest synchronized observation.

        Returns:
            Tuple[np.ndarray, dict]: 
                Tuple, where first entry depicts the observation data concatenated \
                into one array. Second entry represents the observation dictionary.
        """
        merged_obs, obs_dict = self.observation_collector.get_observations()
        if self._agent_params["normalize"]:
            merged_obs = self.normalize_observations(merged_obs)
        return merged_obs, obs_dict

    def normalize_observations(self, merged_obs: np.ndarray) -> np.ndarray:
        """Normalizes the observations with the loaded VecNormalize object.

        Note:
            VecNormalize object from Stable-Baselines3 is agent specific\
            and integral part in order to map right actions.\

        Args:
            merged_obs (np.ndarray):
                observation data concatenated into one array.

        Returns:
            np.ndarray: Normalized observations array.
        """
        assert self._agent_params["normalize"] and hasattr(
            self, "_obs_norm_func"
        )
        return self._obs_norm_func(merged_obs)

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        """Infers an action based on the given observation.

        Args:
            obs (np.ndarray): Merged observation array.

        Returns:
            np.ndarray:
                Action in [linear velocity, angular velocity]
        """
        assert self._agent, "Agent model not initialized!"
        action = self._agent.predict(obs, deterministic=True)[0]
        if self._agent_params["discrete_action_space"]:
            action = self._get_disc_action(action)
        else:
            # clip action
            action = np.maximum(
                np.minimum(self._action_space.high, action),
                self._action_space.low,
            )
        return action

    def get_reward(self, action: np.ndarray, obs_dict: dict) -> float:
        """Calculates the reward based on the parsed observation

        Args:
            action (np.ndarray):
                Velocity commands of the agent\
                in [linear velocity, angular velocity].
            obs_dict (dict):
                Observation dictionary where each key makes up a different \
                kind of information about the environment.
        Returns:
            float: Reward amount
        """
        return self.reward_calculator.get_reward(action=action, **obs_dict)

    def publish_action(self, action: np.ndarray) -> None:
        """Publishes an action on 'self._action_pub' (ROS topic).

        Args:
            action (np.ndarray):
                Action in [linear velocity, angular velocity]
        """
        action_msg = (
            self._get_hol_action_msg(action)
            if self._holonomic
            else self._get_nonhol_action_msg(action)
        )
        self._action_pub.publish(action_msg)

    def _get_disc_action(self, action: int) -> np.ndarray:
        """Returns defined velocity commands for parsed action index.\
            (Discrete action space)

        Args:
            action (int): Index of the desired action.

        Returns:
            np.ndarray: Velocity commands corresponding to the index.
        """
        return np.array(
            [
                self._discrete_actions[action]["linear"],
                self._discrete_actions[action]["angular"],
            ]
        )

    def _get_hol_action_msg(self, action: np.ndarray):
        assert (
            len(action) == 3
        ), "Holonomic robots require action arrays to have 3 entries."
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]
        return action_msg

    def _get_nonhol_action_msg(self, action: np.ndarray):
        assert (
            len(action) == 2
        ), "Non-holonomic robots require action arrays to have 2 entries."
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        return action_msg
