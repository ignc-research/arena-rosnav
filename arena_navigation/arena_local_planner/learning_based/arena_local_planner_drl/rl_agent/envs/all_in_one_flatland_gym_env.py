import json
import math
import os
import random
import time
from shutil import copyfile

import gym
import numpy as np
import rospkg
import rospy
import rosservice
import yaml
from flatland_msgs.srv import StepWorld
from geometry_msgs.msg import Twist, Pose2D
from gym import spaces
from nav_msgs.srv import GetMap

from rl_agent.envs.all_in_one_models.drl.drl_agent import DrlAgent, DrlAgentClient
from rl_agent.envs.all_in_one_models.rlca.rlca_agent import RLCAAgent
from rl_agent.envs.all_in_one_models.teb.teb_all_in_one_interface_agent import (
    TebAllinOneInterfaceAgent,
)
from rl_agent.envs.all_in_one_models.teb.teb_move_base_agent import TebMoveBaseAgent
from rl_agent.utils.all_in_one_observation_collector import ObservationCollectorAllInOne
from rl_agent.utils.all_in_one_visualizer import AllInOneVisualizer
from rl_agent.utils.reward import RewardCalculator
from std_srvs.srv import Empty
from task_generator.robot_manager import RobotManager
from task_generator.tasks import RandomTask
from task_generator.obstacles_manager import ObstaclesManager


class AllInOneEnv(gym.Env):
    def __init__(
        self,
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
        no_ros_node: bool = False,
    ):

        super(AllInOneEnv, self).__init__()

        self.ns = ns
        try:
            # given every environment enough time to initialize, if we dont put sleep,
            # the training script may crash.
            ns_int = int(ns.split("_")[1])
            time.sleep(ns_int * 2)
        except Exception:
            rospy.logwarn(
                f"Can't not determinate the number of the environment, training script may crash!"
            )
            pass

        # process specific namespace in ros system
        if ns is not None or ns != "":
            self.ns_prefix = "/" + ns + "/"
        else:
            self.ns_prefix = "/"

        if not no_ros_node:
            if train_mode:
                rospy.init_node(f"train_env_{self.ns}", disable_signals=False)
            else:
                rospy.init_node(f"eval_env_{self.ns}", disable_signals=False)

        self._debug = debug
        self._evaluation = evaluation
        self._extended_eval = extended_eval
        self._is_train_mode = rospy.get_param("/train_mode")

        self._setup_robot_configuration(robot_yaml_path, settings_yaml_path)

        # set rosparam
        rospy.set_param("/laser_num_beams", self._laser_num_beams)

        # reward calculator
        if safe_dist is None:
            safe_dist = 1.6 * self._robot_radius

        self.reward_calculator = RewardCalculator(
            robot_radius=self._robot_radius,
            safe_dist=safe_dist,
            goal_radius=goal_radius,
            rule=reward_fnc,
            extended_eval=self._extended_eval,
        )
        #DEBUG
        print(f"{self.ns_prefix}cmd_vel")
        # action agent publisher
        if self._is_train_mode:
            self.agent_action_pub = rospy.Publisher(
                f"{self.ns_prefix}cmd_vel", Twist, queue_size=1
            )
        else:
            self.agent_action_pub = rospy.Publisher(
                f"{self.ns_prefix}cmd_vel_pub", Twist, queue_size=1
            )

        # service clients
        if self._is_train_mode:
            self._service_name_step = f"{self.ns_prefix}step_world"
            self._sim_step_client = rospy.ServiceProxy(
                self._service_name_step, StepWorld
            )

        # instantiate task manager
        self.task = self._get_random_task(paths)

        self._steps_curr_episode = 0
        self._max_steps_per_episode = max_steps_per_episode

        # for extended eval
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")
        self._last_robot_pose = None
        self._distance_travelled = 0
        self._safe_dist_counter = 0
        self._collisions = 0
        self._in_crash = False
        self._seed = seed
        self._episode_reward = 0

        if self._evaluation:
            self._current_eval_iteration = 0
            self._evaluation_episodes = evaluation_episodes

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

        # set up visualizer
        self._visualizer = AllInOneVisualizer(
            self._evaluation, self._model_names, self.ns_prefix
        )

        # observation collector
        self.observation_collector = ObservationCollectorAllInOne(
            self.ns,
            self._laser_num_beams,
            self._laser_max_range,
            len(self._models),
            self._run_all_agents_each_iteration,
        )

        self.observation_space = self.observation_collector.get_observation_space()
        self._last_obs_dict = dict()
        self._last_merged_obs = np.zeros(shape=self.observation_space.shape)

        self._last_actions = np.zeros(shape=(len(self._models), 2))
        self._required_observations = dict()
        for model in self._models:
            self._required_observations.update(model.get_observation_info())
        if (
            "laser_3" in self._required_observations
            and self._required_observations["laser_3"]
        ):
            self._last_three_laser_scans = np.zeros((self._laser_num_beams, 3))

        # wait till all agents are ready
        all_ready = False
        while not all_ready:
            all_ready = True
            for i in self._models:
                all_ready = all_ready and i.wait_for_agent()
            if not all_ready:
                self._sim_step_client()
                rospy.loginfo("Not all agents ready yet! Wait...")
            else:
                self.reset()
                evaluation_episodes = 0
                rospy.loginfo("All agents are loaded - Gym environment is ready!")

    def step(self, action: int):

        # get action from chosen model
        if self._run_all_agents_each_iteration:
            action_model = self._last_actions[action, :]
        else:
            action_model = self._models[action].get_next_action(self._last_obs_dict)
            # clip action
            action_model[0] = np.clip(
                a=action_model[0], a_min=self._linear_low, a_max=self._linear_high
            )
            action_model[1] = np.clip(
                a=action_model[1], a_min=self._angular_low, a_max=self._angular_high
            )

        self._pub_action(action_model)

        self._steps_curr_episode += 1

        # wait for new observations
        merged_obs, obs_dict = self.observation_collector.get_observations()

        # if necessary add last 3 laser scans to obs dict
        if (
            "laser_3" in self._required_observations
            and self._required_observations["laser_3"]
        ):
            self._last_three_laser_scans[:, 1:2] = self._last_three_laser_scans[:, 0:1]
            self._last_three_laser_scans[:, 0] = self._last_obs_dict["laser_scan"]
            obs_dict["laser_3"] = self._last_three_laser_scans

        self._last_obs_dict = obs_dict

        if self._run_all_agents_each_iteration:
            self._last_merged_obs[: -2 * len(self._models)] = merged_obs
        else:
            self._last_merged_obs = merged_obs

        if self._run_all_agents_each_iteration:
            for i, agent in enumerate(self._models):
                current_action = agent.get_next_action(self._last_obs_dict)
                # clip action
                current_action[0] = np.clip(
                    a=current_action[0], a_min=self._linear_low, a_max=self._linear_high
                )
                current_action[1] = np.clip(
                    a=current_action[1],
                    a_min=self._angular_low,
                    a_max=self._angular_high,
                )
                self._last_actions[i, :] = current_action
            # add last actions to observation
            self._last_obs_dict["last_actions"] = self._last_actions
            self._last_merged_obs[
                : 2 * len(self._models)
            ] = self._last_actions.flatten()

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            obs_dict["laser_scan"],
            obs_dict["goal_in_robot_frame"],
            action=action_model,
            global_plan=obs_dict["global_plan"],
            robot_pose=obs_dict["robot_pose"],
        )

        done = reward_info["is_done"]

        # extended eval info
        if self._extended_eval:
            self._update_eval_statistics(obs_dict, reward_info, reward, action)

        # info
        info = {}

        if done:
            info["done_reason"] = reward_info["done_reason"]
            info["is_success"] = reward_info["is_success"]

        if self._steps_curr_episode > self._max_steps_per_episode:
            done = True
            info["done_reason"] = 0
            info["is_success"] = 0

        if done and self._evaluation:
            print(
                "Done reason: "
                + str(info["done_reason"])
                + " - Is success: "
                + str(info["is_success"])
                + " - Iterations: "
                + str(self._steps_curr_episode)
                + " - Collisions: "
                + str(self._collisions)
                + " - Episode reward: "
                + str(self._episode_reward)
            )

        # for logging
        if self._extended_eval:
            if done:
                info["collisions"] = self._collisions
                info["distance_travelled"] = round(self._distance_travelled, 2)
                info["time_safe_dist"] = (
                    self._safe_dist_counter * self._action_frequency
                )
                info["time"] = self._steps_curr_episode * self._action_frequency
                info["model_distribution"] = self._last_actions_switch / np.sum(
                    self._last_actions_switch
                )
                if np.sum(self._last_actions_switch_close_obst_dist) != 0:
                    info["model_distribution_close_obst_dist"] = (
                        self._last_actions_switch_close_obst_dist
                        / np.sum(self._last_actions_switch_close_obst_dist)
                    )
                else:
                    info[
                        "model_distribution_close_obst_dist"
                    ] = self._last_actions_switch_close_obst_dist
                if np.sum(self._last_actions_switch_medium_obst_dist) != 0:
                    info["model_distribution_medium_obst_dist"] = (
                        self._last_actions_switch_medium_obst_dist
                        / np.sum(self._last_actions_switch_medium_obst_dist)
                    )
                else:
                    info[
                        "model_distribution_medium_obst_dist"
                    ] = self._last_actions_switch_medium_obst_dist
                if np.sum(self._last_actions_switch_large_obst_dist) != 0:
                    info["model_distribution_large_obst_dist"] = (
                        self._last_actions_switch_large_obst_dist
                        / np.sum(self._last_actions_switch_large_obst_dist)
                    )
                else:
                    info[
                        "model_distribution_large_obst_dist"
                    ] = self._last_actions_switch_large_obst_dist

        self._visualizer.visualize_step(
            action, self._in_crash, self._last_obs_dict["robot_pose"]
        )

        return np.float32(self._last_merged_obs), reward, done, info

    def reset(self):
        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self.agent_action_pub.publish(Twist())
        if self._is_train_mode or self._evaluation:
            self._sim_step_client()
        if self._uses_move_base and self._simulation_started:
            if (
                f"{self.ns_prefix}move_base/clear_costmaps"
                in rosservice.get_service_list()
            ):
                self._clear_costmaps_service()
            else:
                rospy.logwarn(
                    "Couldn't access move base service. Can be fine in first episode."
                )
        if not self._simulation_started:
            self._simulation_started = True

        if self._evaluation:
            random.seed(
                (self._current_eval_iteration % self._evaluation_episodes) * self._seed
            )
        else:
            random.seed()

        self.task.reset()

        self.reward_calculator.reset()
        self._steps_curr_episode = 0

        # extended eval info
        if self._extended_eval:
            self._last_robot_pose = None
            self._distance_travelled = 0
            self._safe_dist_counter = 0
            self._collisions = 0
            self._episode_reward = 0
            self._last_actions_switch = np.zeros(shape=(len(self._models),))
            self._last_actions_switch_close_obst_dist = np.zeros(
                shape=(len(self._models),)
            )
            self._last_actions_switch_medium_obst_dist = np.zeros(
                shape=(len(self._models),)
            )
            self._last_actions_switch_large_obst_dist = np.zeros(
                shape=(len(self._models),)
            )

        merged_obs, obs_dict = self.observation_collector.get_observations()
        self._last_obs_dict = obs_dict

        if (
            "laser_3" in self._required_observations
            and self._required_observations["laser_3"]
        ):
            self._last_three_laser_scans = np.array(
                [
                    self._last_obs_dict["laser_scan"],
                    self._last_obs_dict["laser_scan"],
                    self._last_obs_dict["laser_scan"],
                ]
            ).transpose()
            self._last_obs_dict["laser_3"] = self._last_three_laser_scans

        if self._run_all_agents_each_iteration:
            self._last_merged_obs[: -2 * len(self._models)] = merged_obs
        else:
            self._last_merged_obs = merged_obs

        if self._run_all_agents_each_iteration:
            for i, agent in enumerate(self._models):
                current_action = agent.get_next_action(self._last_obs_dict)
                # clip action
                current_action[0] = np.clip(
                    a=current_action[0], a_min=self._linear_low, a_max=self._linear_high
                )
                current_action[1] = np.clip(
                    a=current_action[1],
                    a_min=self._angular_low,
                    a_max=self._angular_high,
                )
                self._last_actions[i, :] = current_action
            # add last actions to observation
            self._last_obs_dict["last_actions"] = self._last_actions
            self._last_merged_obs[
                : len(self._models) * 2
            ] = self._last_actions.flatten()

        for agent in self._models:
            agent.reset()

        self._visualizer.reset_visualizer()

        if self._evaluation:
            self._current_eval_iteration += 1

        return self._last_merged_obs

    def close(self):
        for m in self._models:
            m.close()

    def render(self, mode="human"):
        pass

    def get_number_models(self) -> int:
        return len(self._models)

    def get_model_names(self) -> [str]:
        return [model.get_name() for model in self._models]

    def _get_random_task(self, paths: dict):
        config_path = paths["all_in_one_parameters"]
        with open(config_path, "r") as params_json:
            config_data = json.load(params_json)

        assert (
            config_data is not None
        ), "Error: All in one parameter file cannot be found!"

        if "map" in config_data:
            map_params = config_data["map"]
            numb_static_obst = map_params["numb_static_obstacles"]
            numb_dyn_obst = map_params["numb_dynamic_obstacles"]
        else:
            rospy.logwarn(
                "No map parameters found in config file. Use 18 dynamic and 6 static obstacles."
            )
            numb_static_obst = 6
            numb_dyn_obst = 18

        service_client_get_map = rospy.ServiceProxy("/static_map", GetMap)
        map_response = service_client_get_map()
        models_folder_path = rospkg.RosPack().get_path("simulator_setup")
        robot_manager = RobotManager(
            self.ns,
            map_response.map,
            os.path.join(models_folder_path, "robot", "myrobot.model.yaml"),
        )
        obstacles_manager = ObstaclesManager(self.ns, map_response.map)
        rospy.set_param("/task_mode", "random")
        numb_obst = numb_static_obst + numb_dyn_obst
        prob_dyn_obst = float(numb_dyn_obst) / numb_obst
        obstacles_manager.register_random_obstacles(numb_obst, prob_dyn_obst)
        return RandomTask(obstacles_manager, robot_manager)

    def _setup_model_configuration(self, paths: dict):
        config_path = paths["all_in_one_parameters"]
        models = []
        with open(config_path, "r") as model_json:
            config_data = json.load(model_json)

        assert (
            config_data is not None
        ), "Error: All in one parameter file cannot be found!"

        config_model = config_data["models"]

        # set up rlca agent
        if "rlca" in config_model and config_model["rlca"]:
            rlca_model = RLCAAgent()
            models.append(rlca_model)

        # set up drl agents
        if "drl" in config_model:
            agent_dir_names = config_model["drl"]

            for i, agent_dir, in enumerate(agent_dir_names):
                if self._drl_server is not None:
                    # Doesnt load model directly into memory, use requests instead
                    drl_model = DrlAgentClient(
                        self._drl_server, config_model["drl_names"][i]
                    )
                else:
                    # Loads model directly
                    drl_model = DrlAgent(
                        os.path.join(paths["drl_agents"], agent_dir),
                        config_model["drl_names"][i],
                    )

                models.append(drl_model)

            self._drl_agent_aliases = config_model["drl_names"]

        # set up teb agents which are based on move base
        if "teb_move_base" in config_model and config_model["teb_move_base"]:
            self._uses_move_base = True
            self._clear_costmaps_service = rospy.ServiceProxy(
                f"{self.ns_prefix}move_base/clear_costmaps", Empty
            )
            for teb_name in config_model["teb"]:
                teb_model = TebMoveBaseAgent(teb_name, self.ns_prefix)
                models.append(teb_model)

        # set up teb agents based on the teb_all_in_one package
        if "teb" in config_model:
            teb_names = config_model["teb_names"]

            base_dir = rospkg.RosPack().get_path("arena_local_planner_drl")
            for i, teb_config_path in enumerate(config_model["teb"]):
                teb_config_path_full = os.path.join(
                    base_dir, "configs", "all_in_one_hyperparameters", teb_config_path
                )
                models.append(
                    TebAllinOneInterfaceAgent(
                        teb_names[i], self.ns, teb_config_path_full
                    )
                )

        self._models = models

        # check if all agents should be run before model selection
        if (
            "run_all_agents_each_iteration" in config_data
            and config_data["run_all_agents_each_iteration"]
        ):
            self._run_all_agents_each_iteration = True
        else:
            self._run_all_agents_each_iteration = False

        # copy hyperparameter file and teb config files
        if self._is_train_mode and "model" in paths:
            doc_location = os.path.join(paths["model"], "all_in_one_parameters.json")
            if not os.path.exists(doc_location):
                with open(doc_location, "w", encoding="utf-8") as target:
                    json.dump(config_data, target, ensure_ascii=False, indent=4)
            for teb_config_path in config_model["teb"]:
                full_dest_path = os.path.join(paths["model"], teb_config_path)
                base_dir = rospkg.RosPack().get_path("arena_local_planner_drl")
                if not os.path.exists(full_dest_path):
                    source_file_path = os.path.join(
                        base_dir,
                        "configs",
                        "all_in_one_hyperparameters",
                        teb_config_path,
                    )
                    copyfile(source_file_path, full_dest_path)

    def _setup_robot_configuration(self, robot_yaml_path: str, settings_yaml_path: str):
        """get the configuration from the yaml file, including robot radius.

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

        # set up velocity limits
        with open(settings_yaml_path, "r") as fd:
            setting_data = yaml.safe_load(fd)
            linear_range = setting_data["robot"]["continuous_actions"]["linear_range"]
            angular_range = setting_data["robot"]["continuous_actions"]["angular_range"]
            self._angular_low = angular_range[0]
            self._angular_high = angular_range[1]
            self._linear_low = linear_range[0]
            self._linear_high = linear_range[1]

    def _update_eval_statistics(
        self, obs_dict: dict, reward_info: dict, reward: float, action: int
    ):
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
                self._last_robot_pose, obs_dict["robot_pose"]
            )

        # collision detector
        if "crash" in reward_info:
            if reward_info["crash"] and not self._in_crash:
                self._collisions += 1
                # when crash occures, robot strikes obst for a few consecutive timesteps
                # we want to count it as only one collision
                self._in_crash = True
        else:
            self._in_crash = False

        # safe dist detector
        if "safe_dist" in reward_info:
            if reward_info["safe_dist"]:
                self._safe_dist_counter += 1

        self._last_robot_pose = obs_dict["robot_pose"]

        self._last_actions_switch[action] += 1
        min_obst_dist = np.min(obs_dict["laser_scan"])
        if min_obst_dist < 1.5:
            self._last_actions_switch_close_obst_dist[action] += 1
        elif min_obst_dist < 3:
            self._last_actions_switch_medium_obst_dist[action] += 1
        else:
            self._last_actions_switch_large_obst_dist[action] += 1

        self._episode_reward += reward * (0.99 ** self._steps_curr_episode)

    def _pub_action(self, action):
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        self.agent_action_pub.publish(action_msg)

    @staticmethod
    def get_distance(pose_1: Pose2D, pose_2: Pose2D):
        return math.hypot(pose_2.x - pose_1.x, pose_2.y - pose_1.y)


class AllInOneEnvDeploy(AllInOneEnv):
    def __init__(self, *args, **kwargs):
        if 'ns' not in kwargs:
            self.ns = args[0]
        else:
            self.ns = kwargs['ns']



    def send_cmd(self):
        merged_obs, _ = self.observation_collector.get_observations()
        action_model = self._models[0].get_next_action(self._last_obs_dict)
        # clip action
        action_model[0] = np.clip(
            a=action_model[0], a_min=self._linear_low, a_max=self._linear_high
        )
        action_model[1] = np.clip(
            a=action_model[1], a_min=self._angular_low, a_max=self._angular_high
        )
        self._pub_action(action_model)

    def _setup_robot_configuration(self, robot_yaml_path: str, settings_yaml_path: str):
        """get the configuration from the yaml file, including robot radius.

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

