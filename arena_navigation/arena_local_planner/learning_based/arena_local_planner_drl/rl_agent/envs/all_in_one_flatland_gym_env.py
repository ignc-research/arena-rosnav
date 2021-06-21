import math
import time
import warnings
from shutil import copyfile

import gym
import rosservice
from gym import spaces

from flatland_msgs.srv import StepWorld
from geometry_msgs.msg import Twist
from rl_agent.envs.all_in_one_models.drl.drl_agent import DrlAgent, DrlAgentClient
from rl_agent.envs.all_in_one_models.rlca_agent import RLCAAgent
from rl_agent.envs.all_in_one_models.teb.teb_all_in_one_interface_agent import TebAllinOneInterfaceAgent
from rl_agent.envs.all_in_one_models.teb.teb_move_base_agent import TebMoveBaseAgent
from rl_agent.utils.all_in_one_observation_collector import ObservationCollectorAllInOne
from rl_agent.utils.reward import RewardCalculator
from std_srvs.srv import Empty
from task_generator.task_generator.tasks import *
from task_generator.task_generator.tasks import RandomTask


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
                 extended_eval: bool = False):

        super(AllInOneEnv, self).__init__()

        self.ns = ns
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            ns_int = int(ns.split("_")[1])
            time.sleep(ns_int * 2)
        except Exception:
            rospy.logwarn(f"Can't not determinate the number of the environment, training script may crash!")
            pass

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

        # set rosparam
        rospy.set_param("/laser_num_beams", self._laser_num_beams)

        # observation collector
        self.observation_collector = ObservationCollectorAllInOne(
            self.ns, self._laser_num_beams, self._laser_max_range)

        self.observation_space = self.observation_collector.get_observation_space()
        self._last_obs_dict = dict()
        self._last_merged_obs = np.zeros(shape=self.observation_space.shape)

        # reward calculator
        if safe_dist is None:
            safe_dist = 1.6 * self._robot_radius

        self.reward_calculator = RewardCalculator(
            robot_radius=self._robot_radius, safe_dist=safe_dist, goal_radius=goal_radius,
            rule=reward_fnc, extended_eval=self._extended_eval)

        # action agent publisher
        if self._is_train_mode:
            self.agent_action_pub = rospy.Publisher(f'{self.ns_prefix}cmd_vel', Twist, queue_size=1)
        else:
            self.agent_action_pub = rospy.Publisher(f'{self.ns_prefix}cmd_vel_pub', Twist, queue_size=1)

        # service clients
        if self._is_train_mode:
            self._service_name_step = f'{self.ns_prefix}step_world'
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld)

        # instantiate task manager
        # TODO read parameters from config file
        # self.task = get_predefined_task(ns, mode='random', prob_dynamic_obst=0.75, numb_obst=25)
        self.task = self._get_random_task(numb_obst=20, prob_dyn_obst=0.6)

        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode

        # for extended eval
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")
        self._last_robot_pose = None
        self._distance_travelled = 0
        self._safe_dist_counter = 0
        self._collisions = 0
        self._in_crash = False

        # set up model parameters
        self._uses_move_base = False
        self._clear_costmaps_service = None
        self._simulation_started = False
        self._drl_server = drl_server
        self._setup_model_configuration(paths)
        self.action_space = spaces.Discrete(len(self._models))
        self._model_names = []
        for i in self._models:
            self._model_names.append(i.get_name())

        self._last_actions = np.zeros(shape=(len(self._models, )))

        # wait till all agents are ready
        all_ready = False
        while not all_ready:
            all_ready = True
            for i in self._models:
                all_ready = all_ready and i.wait_for_agent()
            if not all_ready:
                self.reset()
                self._sim_step_client()
                print("Not all agents ready yet! Wait...")


    def step(self, action: int):
        # get action from chosen model
        action_model = self._models[action].get_next_action(self._last_obs_dict)

        # clip action
        action_model[0] = np.clip(a=action_model[0], a_min=self._linear_low, a_max=self._linear_high)
        action_model[1] = np.clip(a=action_model[1], a_min=self._angular_low, a_max=self._angular_high)

        # print(action_model)

        self._pub_action(action_model)

        self._steps_curr_episode += 1

        # wait for new observations
        merged_obs, obs_dict = self.observation_collector.get_observations()
        self._last_obs_dict = obs_dict
        self._last_merged_obs = merged_obs

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict['laser_scan'], obs_dict['goal_in_robot_frame'],
            action=action_model, global_plan=obs_dict['global_plan'],
            robot_pose=obs_dict['robot_pose'])

        done = reward_info['is_done']

        # extended eval info
        if self._extended_eval:
            self._update_eval_statistics(obs_dict, reward_info, action)

        # info
        info = {}

        if done:
            info['done_reason'] = reward_info['done_reason']
            info['is_success'] = reward_info['is_success']

        if self._steps_curr_episode > self._max_steps_per_episode:
            done = True
            info['done_reason'] = 0
            info['is_success'] = 0

        if done and self._evaluation:
            print("Done reason: " + str(info['done_reason']) + " --- Is success: " + str(
                info['is_success']) + " --- Iterations: " + str(self._steps_curr_episode))

        # for logging
        if self._extended_eval:
            if done:
                info['collisions'] = self._collisions
                info['distance_travelled'] = round(self._distance_travelled, 2)
                info['time_safe_dist'] = self._safe_dist_counter * self._action_frequency
                info['time'] = self._steps_curr_episode * self._action_frequency
                info['model_distribution'] = self._last_actions / np.sum(self._last_actions)

        return np.float32(merged_obs), reward, done, info

    def reset(self):
        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self.agent_action_pub.publish(Twist())
        if self._is_train_mode or self._evaluation:
            self._sim_step_client()
        if self._uses_move_base and self._simulation_started:
            if f'{self.ns_prefix}move_base/clear_costmaps' in rosservice.get_service_list():
                self._clear_costmaps_service()
            else:
                warnings.warn("Couldn't access move base service. Can be fine in first episode.")
        if not self._simulation_started:
            self._simulation_started = True
        self.task.reset()

        self.reward_calculator.reset()
        self._steps_curr_episode = 0

        # extended eval info
        if self._extended_eval:
            self._last_robot_pose = None
            self._distance_travelled = 0
            self._safe_dist_counter = 0
            self._collisions = 0
            self._last_actions = np.zeros(shape=(len(self._models, )))

        merged_obs, obs_dict = self.observation_collector.get_observations()
        self._last_obs_dict = obs_dict
        self._last_merged_obs = merged_obs
        return np.float32(merged_obs)

    def close(self):
        pass

    def render(self, mode='human'):
        pass

    def get_number_models(self) -> int:
        return len(self._models)

    def get_model_names(self) -> [str]:
        return [model.get_name() for model in self._models]

    def _get_random_task(self, numb_obst: int, prob_dyn_obst: float):
        service_client_get_map = rospy.ServiceProxy('/static_map', GetMap)
        map_response = service_client_get_map()
        models_folder_path = rospkg.RosPack().get_path('simulator_setup')
        robot_manager = RobotManager(self.ns, map_response.map, os.path.join(
            models_folder_path, 'robot', "myrobot.model.yaml"))
        obstacles_manager = ObstaclesManager(self.ns, map_response.map)
        rospy.set_param("/task_mode", "random")
        obstacles_manager.register_random_obstacles(numb_obst, prob_dyn_obst)
        return RandomTask(obstacles_manager, robot_manager)

    def _setup_model_configuration(self, paths: dict):
        config_path = paths['all_in_one_parameters']
        models = []
        with open(config_path, 'r') as model_json:
            config_data = json.load(model_json)

        assert config_data is not None, "Error: All in one parameter file cannot be found!"

        config_model = config_data['models']

        # set up rlca agent
        if 'rlca' in config_model and config_model['rlca']:
            rlca_model = RLCAAgent()
            models.append(rlca_model)

        # set up drl agents
        if 'drl' in config_model:
            agent_dir_names = config_model['drl']

            for i, agent_dir, in enumerate(agent_dir_names):
                if self._drl_server is not None:
                    # Doesnt load model directly into memory, use requests instead
                    drl_model = DrlAgentClient(self._drl_server, config_model['drl_names'][i])
                else:
                    # Loads model directly
                    drl_model = DrlAgent(os.path.join(paths['drl_agents'], agent_dir), config_model['drl_names'][i])

                models.append(drl_model)

            self._drl_agent_aliases = config_model['drl_names']

        # set up teb agents which are based on move base
        if 'teb_move_base' in config_model and config_model['teb_move_base']:
            self._uses_move_base = True
            self._clear_costmaps_service = rospy.ServiceProxy(f'{self.ns_prefix}move_base/clear_costmaps',
                                                              Empty)
            for teb_name in config_model['teb']:
                teb_model = TebMoveBaseAgent(teb_name, self.ns_prefix)
                models.append(teb_model)

        # set up teb agents based on the teb_all_in_one package
        if 'teb' in config_model:
            teb_names = config_model['teb_names']

            base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
            for i, teb_config_path in enumerate(config_model['teb']):
                teb_config_path_full = os.path.join(base_dir, 'configs', 'all_in_one_hyperparameters', teb_config_path)
                models.append(TebAllinOneInterfaceAgent(teb_names[i], self.ns, teb_config_path_full))

        self._models = models

        # copy hyperparameter file and teb config files
        if self._is_train_mode and 'model' in paths:
            doc_location = os.path.join(paths['model'], "all_in_one_parameters.json")
            if not os.path.exists(doc_location):
                with open(doc_location, "w", encoding='utf-8') as target:
                    json.dump(config_data, target, ensure_ascii=False, indent=4)
            for teb_config_path in config_model['teb']:
                full_dest_path = os.path.join(paths['model'], teb_config_path)
                base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
                if not os.path.exists(full_dest_path):
                    source_file_path = os.path.join(base_dir, 'configs', 'all_in_one_hyperparameters', teb_config_path)
                    copyfile(source_file_path, full_dest_path)

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

    def _update_eval_statistics(self, obs_dict: dict, reward_info: dict, action: int):
        """
        Updates the metrics for extended eval mode

        param obs_dict (dict): observation dictionary from ObservationCollector.get_observations(),
            necessary entries: 'robot_pose'
        param reward_info (dict): dictionary containing information returned from RewardCalculator.get_reward(),
            necessary entries: 'crash', 'safe_dist'
        """
        # distance travelled
        if self._last_robot_pose is not None:
            self._distance_travelled += AllInOneEnv.get_distance(
                self._last_robot_pose, obs_dict['robot_pose'])

        # collision detector
        if 'crash' in reward_info:
            if reward_info['crash'] and not self._in_crash:
                self._collisions += 1
                # when crash occures, robot strikes obst for a few consecutive timesteps
                # we want to count it as only one collision
                self._in_crash = True
        else:
            self._in_crash = False

        # safe dist detector
        if 'safe_dist' in reward_info:
            if reward_info['safe_dist']:
                self._safe_dist_counter += 1

        self._last_robot_pose = obs_dict['robot_pose']

        self._last_actions[action] += 1

    def _pub_action(self, action):
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        self.agent_action_pub.publish(action_msg)

    @staticmethod
    def get_distance(pose_1: Pose2D, pose_2: Pose2D):
        return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)
