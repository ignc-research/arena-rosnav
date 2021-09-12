from typing import Tuple

import numpy as np
import os
import rospkg

from rl_agent.base_agent_wrapper import BaseDRLAgent

from geometry_msgs.msg import Twist


DEFAULT_ACTION_SPACE = os.path.join(
    rospkg.RosPack().get_path("arena_local_planner_drl"),
    "configs",
    "default_settings.yaml",
)


class TrainingDRLAgent(BaseDRLAgent):
    def __init__(
        self,
        ns: str,
        robot_name: str,
        hyperparameter_path: str,
        action_space_path: str = DEFAULT_ACTION_SPACE,
        *args,
        **kwargs
    ) -> None:
        """DRL Agent Wrapper Class for training.

        Description:
            The DNN is actually loaded by the PPO module of Stable-Baselines3.
            This class rather holds other agent specific properties like \
            the observation space, action space, reward function calculator.
            These are essentially specified in dedicated setting files.

        For Gym/PettingZoo-Env relevant methods; 
        (in order to interact with PPO of SB3) 
            - action_space(self) -> spaces.Box
            - observation_space(self) -> spaces.Box
            - get_observations(self) -> Tuple[np.ndarray, dict]
            - get_reward(self, action: np.ndarray, obs_dict: dict) -> float
            - publish_action(self, action: np.ndarray) -> None
            - _get_disc_action(self, action: int) -> np.ndarray

        Note:
            Robot model settings can be specified in the hyperparameters.json\
            under "robot" - where the given name depicts the yaml file to be loaded\
            from '../simulation_setup/robot/*.model.yaml'

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
        super().__init__(
            ns=ns,
            robot_name=robot_name,
            hyperparameter_path=hyperparameter_path,
            action_space_path=action_space_path,
            *args,
            **kwargs
        )

    def setup_agent(self) -> None:
        pass

    def get_observations(self) -> Tuple[np.ndarray, dict]:
        """Retrieves the latest synchronized observation.

        Returns:
            Tuple[np.ndarray, dict]: 
                Tuple, where first entry depicts the observation data concatenated \
                into one array. Second entry represents the observation dictionary.
        """
        return self.observation_collector.get_observations()

    def publish_action(self, action: np.ndarray) -> None:
        """Publishes an action on 'self._action_pub' (ROS topic).

        Args:
            action (np.ndarray):
                Action in [linear velocity, angular velocity]
        """
        if self._agent_params["discrete_action_space"]:
            action = self._get_disc_action(action)

        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        self._action_pub.publish(action_msg)
