import random

import gym
import numpy as np
import rospy
import yaml
from flatland_msgs.srv import StepWorld
from gym import spaces
from rl_agent.utils.all_in_one_planner.local_planner_manager import LocalPlannerManager
from rl_agent.utils.all_in_one_planner.logger import Logger
from rl_agent.utils.all_in_one_planner.observation_collector import ObservationCollectorAllInOne
from rl_agent.utils.all_in_one_planner.reward_calculator import RewardCalculator
from rl_agent.utils.all_in_one_planner.step_processor import StepProcessor
from rl_agent.utils.all_in_one_planner.task_manager import TaskManager
from rl_agent.utils.all_in_one_planner.visualizer import AllInOneVisualizer


class AllInOneEnv(gym.Env):

    def __init__(self,
                 ns: str,
                 robot_yaml_path: str,
                 settings_yaml_path: str,
                 reward_fnc: str,
                 safe_dist: float = None,
                 goal_radius: float = 0.1,
                 max_steps_per_episode=1000,
                 train_mode: bool = True,
                 debug: bool = False,
                 paths: dict = None,
                 drl_server: str = None,
                 evaluation: bool = False,
                 evaluation_episodes: int = 40,
                 seed: int = 1,
                 extended_eval: bool = False,
                 run_scenario: bool = False
                 ):

        super(AllInOneEnv, self).__init__()

        self.ns = ns

        # process specific namespace in ros system
        if ns is not None or ns != "":
            self.ns_prefix = '/' + ns + '/'
        else:
            self.ns_prefix = '/'

        if train_mode:
            rospy.init_node(f'train_env_{self.ns}', disable_signals=False)
        else:
            rospy.init_node(f'eval_env_{self.ns}', disable_signals=False)

        self._debug = debug
        self._evaluation = evaluation
        self._extended_eval = extended_eval
        self._is_train_mode = rospy.get_param("/train_mode")

        self._setup_robot_configuration(robot_yaml_path, settings_yaml_path)

        self._action_bounds = [self._linear_low, self._linear_high, self._angular_low, self._angular_high]

        # define safe distance based on robot radius if not defined
        if safe_dist is None:
            safe_dist = 1.6 * self._robot_radius

        self.reward_calculator = RewardCalculator(
            robot_radius=self._robot_radius, safe_dist=safe_dist, goal_radius=goal_radius,
            rule=reward_fnc, extended_eval=self._extended_eval)

        # service clients
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld)

        # instantiate task manager
        # TODO make reset interval parameter
        if self._evaluation:
            map_update_freq = 1
        else:
            map_update_freq = 20
        self.task_manager = TaskManager(self.ns, map_update_freq, paths, run_scenario)
        self._seed = seed

        if self._evaluation:
            self._current_eval_iteration = 0
            self._evaluation_episodes = evaluation_episodes

        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode

        self._local_planner_manager = LocalPlannerManager(drl_server, paths, self.ns, self._is_train_mode)

        self.action_space = spaces.Discrete(self._local_planner_manager.get_numb_models())

        # set up visualizer
        self._visualizer = AllInOneVisualizer(self._evaluation, self._local_planner_manager.get_model_names(),
                                              self.ns_prefix)

        self._step_processor = StepProcessor(self._is_train_mode, self.ns_prefix, paths['all_in_one_parameters'],
                                             self._max_steps_per_episode,
                                             self._local_planner_manager, self.reward_calculator, self._action_bounds)

        self._run_all_agents_each_iteration, self._all_in_one_planner_frequency, self._global_planner_frequency = \
            self._step_processor.get_step_parameters()

        self.reward_calculator.set_global_planner_frequency(self._global_planner_frequency)

        self._logger = Logger(extended_eval, evaluation, max_steps_per_episode,
                              self._local_planner_manager.get_numb_models(), self._all_in_one_planner_frequency)

        # observation collector
        self.observation_collector = ObservationCollectorAllInOne(
            self.ns, self._laser_num_beams, self._laser_max_range, self._local_planner_manager.get_numb_models(),
            self._local_planner_manager.get_required_observations(),
            include_model_actions=self._run_all_agents_each_iteration)

        self._step_processor.set_observation_collector(self.observation_collector)

        self.observation_space = self.observation_collector.get_observation_space()
        self._last_obs_dict = dict()
        self._last_merged_obs = np.zeros(shape=self.observation_space.shape)

        if self._is_train_mode:
            self._local_planner_manager.wait_for_agents(self._sim_step_client)
            self.reset()

        rospy.loginfo("Environment " + self.ns + ": All agents are loaded - Gym environment is ready!")

    def step(self, action: int):
        action_model, self._last_obs_dict, self._last_merged_obs, reward, reward_info = \
            self._step_processor.process_step(action)

        done = reward_info['is_done']

        # info
        info, in_crash = self._logger.get_step_info(done, reward_info, self._last_obs_dict, reward, action,
                                                    self._steps_curr_episode)

        self._visualizer.visualize_step(action, in_crash, self._last_obs_dict['robot_pose'])

        self._steps_curr_episode += 1

        return np.float32(self._last_merged_obs), reward, done, info

    def reset(self):
        self._local_planner_manager.reset_planners()
        self.observation_collector.reset()

        if self._is_train_mode:
            self._sim_step_client()

        # reset task manager
        if self._evaluation:
            seed = (self._current_eval_iteration % self._evaluation_episodes) * self._seed
        else:
            seed = random.randint(0, 1000000)
        self.task_manager.reset(seed)

        self.reward_calculator.reset()
        self._logger.reset()
        self._step_processor.reset()
        self._visualizer.reset_visualizer()

        if self._evaluation:
            self._current_eval_iteration += 1
        self._steps_curr_episode = 0

        return self._last_merged_obs

    def close(self):
        self._local_planner_manager.close_planners()
        self.observation_collector.close()

    def render(self, mode='human'):
        pass

    def get_number_models(self) -> int:
        return self._local_planner_manager.get_numb_models()

    def get_model_names(self) -> [str]:
        return self._local_planner_manager.get_model_names()

    def get_all_in_one_planner_frequency(self):
        return self._all_in_one_planner_frequency

    def _setup_robot_configuration(self, robot_yaml_path: str, settings_yaml_path: str):
        """get the configuration from the yaml file, including robot radius.

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
                                'radius', 0.3) * 1.05
                        if footprint['radius']:
                            self._robot_radius = footprint['radius'] * 1.05
            # get laser related information
            for plugin in robot_data['plugins']:
                if plugin['type'] == 'Laser':
                    laser_angle_min = plugin['angle']['min']
                    laser_angle_max = plugin['angle']['max']
                    laser_angle_increment = plugin['angle']['increment']
                    self._laser_num_beams = int(
                        round((laser_angle_max - laser_angle_min) / laser_angle_increment) + 1)
                    # set rosparam
                    rospy.set_param("/laser_num_beams", self._laser_num_beams)
                    self._laser_max_range = plugin['range']

        # set up velocity limits
        with open(settings_yaml_path, 'r') as fd:
            setting_data = yaml.safe_load(fd)
            linear_range = setting_data['robot']['continuous_actions']['linear_range']
            angular_range = setting_data['robot']['continuous_actions']['angular_range']
            self._angular_low = angular_range[0]
            self._angular_high = angular_range[1]
            self._linear_low = linear_range[0]
            self._linear_high = linear_range[1]
