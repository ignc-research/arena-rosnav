"""PettingZoo Environment for Single-/Multi Agent Reinforcement Learning"""
from time import sleep
from typing import List, Tuple, Dict, Any, Union, Callable

import numpy as np
import rospy

from gym import spaces
from pettingzoo import *
from pettingzoo.utils import wrappers, from_parallel, to_parallel
import supersuit as ss
from stable_baselines3.common.vec_env import VecEnv

from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.training_agent_wrapper import TrainingDRLAgent
from task_generator.task_generator.marl_tasks import get_MARL_task

from flatland_msgs.srv import StepWorld, StepWorldRequest

from rl_agent.utils.supersuit_utils import *
# from rl_agent.utils.supersuit_utils import MarkovVectorEnv_patched


def env_fn(**kwargs: Dict[str, Any]) -> VecEnv:
    """
    The env function wraps the environment in 3 wrappers by default. These
    wrappers contain logic that is common to many pettingzoo environments.
    We recommend you use at least the OrderEnforcingWrapper on your own environment
    to provide sane error messages. You can find full documentation for these methods
    elsewhere in the developer documentation.
    """
    env = FlatlandPettingZooEnv(**kwargs)
    env = from_parallel(env)
    env = ss.pad_action_space_v0(env)
    env = ss.pad_observations_v0(env)
    env = to_parallel(env)
    env = MarkovVectorEnv_patched(env, black_death=True)
    return env


class FlatlandPettingZooEnv(ParallelEnv):
    """The SuperSuit Parallel environment steps every live agent at once."""

    def __init__(
        self,
        num_agents: int,
        agent_list_fn: Callable[
            [int, str, str, str, str], List[TrainingDRLAgent]
        ],
        PATHS: dict,
        ns: str = None,
        task_mode: str = "staged",
        max_num_moves_per_eps: int = 1000,
    ) -> None:
        """Initialization method for the Arena-Rosnav Pettingzoo Environment.

        Args:
            num_agents (int): Number of possible agents.
            agent_list_fn (Callable[ [int, str, str, str, str], List[TrainingDRLAgent] ]): Initialization function for the agents. \
                Returns a list of agent instances.
            ns (str, optional): Environments' ROS namespace. There should only be one env per ns. Defaults to None.
            task_mode (str, optional): Navigation task mode for the agents. Modes to chose from: ['random', 'staged']. \
                Defaults to "random".
            max_num_moves_per_eps (int, optional): Maximum number of moves per episode. Defaults to 1000.
            
        Note:
            These attributes should not be changed after initialization:
            - possible_agents
            - action_spaces
            - observation_spaces
        """
        self._ns = "" if ns is None or ns == "" else ns + "/"
        self._is_train_mode = rospy.get_param("/train_mode")
        self.metadata = {"render.modes": ["human"], "name": "rps_v2"}

        agent_list = agent_list_fn(num_agents, ns=ns)

        self.agents = []
        self.possible_agents = [a._robot_sim_ns for a in agent_list]
        self.agent_name_mapping = dict(
            zip(self.possible_agents, list(range(len(self.possible_agents))))
        )
        self.agent_object_mapping = dict(zip(self.possible_agents, agent_list))
        self._robot_sim_ns = [a._robot_sim_ns for a in agent_list]
        self.terminal_observation = {}

        self._validate_agent_list()

        # task manager
        self.task_manager = get_MARL_task(
            ns=ns,
            mode=task_mode,
            robot_names=self._robot_sim_ns,
            PATHS=PATHS,
        )

        # service clients
        if self._is_train_mode:
            self._service_name_step = f"{self._ns}step_world"
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld
            )

        self._max_num_moves = max_num_moves_per_eps

    def observation_space(self, agent: str) -> spaces.Box:
        """Returns specific agents' observation space.

        Args:
            agent (str): Agent name as given in ``self.possible_agents``.

        Returns:
            spaces.Box: Observation space of type _gym.spaces_.
        """
        return self.agent_object_mapping[agent].observation_space

    def action_space(self, agent: str) -> spaces.Box:
        """Returns specific agents' action space.

        Args:
            agent (str): Agent name as given in ``self.possible_agents``.

        Returns:
            spaces.Box: Action space of type _gym.spaces_.
        """
        return self.agent_object_mapping[agent].action_space

    def _validate_agent_list(self) -> None:
        """Validates the agent list.

        Description:
            Checks if all agents are named differently. That means each robot adresses its own namespace.
        """
        assert len(self.possible_agents) == len(
            set(self.possible_agents)
        ), "Robot names and thus their namespaces, have to be unique!"

    def reset(self) -> Dict[str, np.ndarray]:
        """Resets the environment and returns the new set of observations (keyed by the agent name)

        Description:
            This method is called when all agents reach an end criterion. End criterions are: exceeding the \
            max number of steps per episode, crash or reaching the end criterion.
            The scene is then reseted.
            
        Returns:
            Dict[str, np.ndarray]: Observations dictionary in {_agent name_: _respective observations_}.
        """
        self.agents = self.possible_agents[:]
        self.num_moves = 0
        self.terminal_observation = {}

        # reset the reward calculator
        for agent in self.agents:
            self.agent_object_mapping[agent].reward_calculator.reset()

        # reset the task manager
        self.task_manager.reset()
        # step one timestep in the simulation to update the scene
        if self._is_train_mode:
            self._sim_step_client()

        # get first observations for the next episode
        observations = {
            agent: self.agent_object_mapping[agent].get_observations()[0]
            for agent in self.agents
        }

        return observations

    def step(
        self, actions: Dict[str, np.ndarray]
    ) -> Tuple[
        Dict[str, np.ndarray],
        Dict[str, float],
        Dict[str, bool],
        Dict[str, Dict[str, Any]],
    ]:
        """Simulates one timestep and returns the most recent environment information.

        Description:
            This function takes in velocity commands and applies those to the simulation.
            Afterwards, agents' observations are retrieved from the current timestep and \
            the reward is calculated. \
            Proceeding with the ``RewardCalculator`` processing the observations and detecting certain events like \
            if a crash occured, a goal was reached. Those informations are returned in the '*reward\_info*' \
            which itself is a dictionary. \
            Eventually, dictionaries containing every agents' observations, rewards, done flags and \
            episode information is returned.

        Args:
            actions (Dict[str, np.ndarray]): Actions dictionary in {_agent name_: _respective observations_}.

        Returns:
            Tuple[ Dict[str, np.ndarray], Dict[str, float], Dict[str, bool], Dict[str, Dict[str, Any]], ]: Observations, \
                rewards, done flags and episode informations dictionary.
        
        Note:
            Done reasons are mapped as follows: __0__ - episode length exceeded, __1__ - agent crashed, \
                __2__ - agent reached its goal.
        """
        # If a user passes in actions with no agents, then just return empty observations, etc.
        if not actions:
            self.agents = []
            return {}, {}, {}, {}

        # actions
        for agent in self.possible_agents:
            if agent in actions:
                self.agent_object_mapping[agent].publish_action(actions[agent])
            else:
                noop = np.zeros(shape=self.action_space(agent).shape)
                self.agent_object_mapping[agent].publish_action(noop)

        # fast-forward simulation
        self.call_service_takeSimStep()
        self.num_moves += 1

        merged_obs, rewards, reward_infos = {}, {}, {}

        for agent in actions:
            # observations
            merged, _dict = self.agent_object_mapping[agent].get_observations()
            merged_obs[agent] = merged

            # rewards and infos
            reward, reward_info = self.agent_object_mapping[agent].get_reward(
                action=actions[agent], obs_dict=_dict
            )
            rewards[agent], reward_infos[agent] = reward, reward_info

        # dones & infos
        dones, infos = self._get_dones(reward_infos), self._get_infos(
            reward_infos
        )

        # remove done agents from the active agents list
        self.agents = [agent for agent in self.agents if not dones[agent]]

        for agent in self.possible_agents:
            # agent is done in this episode
            if agent in dones and dones[agent]:
                self.terminal_observation[agent] = merged_obs[agent]
                infos[agent]["terminal_observation"] = merged_obs[agent]
            # agent is done since atleast last episode
            elif agent not in self.agents:
                if agent not in infos:
                    infos[agent] = {}
                infos[agent][
                    "terminal_observation"
                ] = self.terminal_observation[agent]

        return merged_obs, rewards, dones, infos

    @property
    def max_num_agents(self):
        return len(self.agents)

    def call_service_takeSimStep(self, t: float = None):
        """Fast-forwards the simulation.

        Description:
            Simulates the Flatland simulation for a certain amount of seconds.
            
        Args:
            t (float, optional): Time in seconds. When ``t`` is None, time is forwarded by ``step_size`` s \
                (ROS parameter). Defaults to None.
        """
        request = StepWorldRequest() if t is None else StepWorldRequest(t)

        try:
            response = self._sim_step_client(request)
            rospy.logdebug("step service=", response)
        except rospy.ServiceException as e:
            rospy.logdebug("step Service call failed: %s" % e)

    def _get_dones(
        self, reward_infos: Dict[str, Dict[str, Any]]
    ) -> Dict[str, bool]:
        """Extracts end flags from the reward information dictionary.

        Args:
            reward_infos (Dict[str, Dict[str, Any]]): Episode information from the ``RewardCalculator`` in \
                {_agent name_: _reward infos_}.

        Returns:
            Dict[str, bool]: Dones dictionary in {_agent name_: _done flag_}
            
        Note:
            Relevant dictionary keys are: "is_done", "is_success", "done_reason"
        """
        return (
            {agent: reward_infos[agent]["is_done"] for agent in self.agents}
            if self.num_moves < self._max_num_moves
            else {agent: True for agent in self.agents}
        )

    def _get_infos(
        self, reward_infos: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Dict[str, Any]]:
        """Extracts the current episode information from the reward information dictionary.

        Args:
            reward_infos (Dict[str, Dict[str, Any]]): Episode information from the ``RewardCalculator`` in \
                {_agent name_: _reward infos_}.

        Returns:
            Dict[str, Dict[str, Any]]: Info dictionary in {_agent name_: _done flag_}
            
        Note:
            Relevant dictionary keys are: "is_done", "is_success", "done_reason"
        """
        infos = {agent: {} for agent in self.agents}
        for agent in self.agents:
            if reward_infos[agent]["is_done"]:
                infos[agent] = reward_infos[agent]
            elif self.num_moves >= self._max_num_moves:
                infos[agent] = {
                    "done_reason": 0,
                    "is_success": 0,
                }
        return infos
