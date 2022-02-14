import json
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rl_agent.utils.all_in_one_planner.local_planner_manager import LocalPlannerManager
from rl_agent.utils.all_in_one_planner.observation_collector import ObservationCollectorAllInOne
from rl_agent.utils.all_in_one_planner.reward_calculator import RewardCalculator


class StepProcessor:

    def __init__(self, is_train_mode: bool, ns_prefix: str, all_in_one_params_path: str, max_iterations: int,
                 local_planner_manager: LocalPlannerManager, reward_calculator: RewardCalculator, action_bounds: [int]):

        self._is_train_mode = is_train_mode
        self._ns_prefix = ns_prefix
        self._local_planner_manager = local_planner_manager
        self._observation_collector = None
        self._reward_calculator = reward_calculator
        self._action_bounds = action_bounds
        self._max_iterations = max_iterations

        # action agent publisher
        if self._is_train_mode:
            self.agent_action_pub = rospy.Publisher(f'{self._ns_prefix}cmd_vel', Twist, queue_size=1)
        else:
            self.agent_action_pub = rospy.Publisher(f'{self._ns_prefix}cmd_vel_pub', Twist, queue_size=1)

        # read in execution modes
        self._extract_step_parameters(all_in_one_params_path)

        self._last_actions = np.zeros(shape=(self._local_planner_manager.get_numb_models(), 2))
        self._last_obs_dict = dict()
        self._last_merged_obs = np.array([])
        self._current_iteration = 0  # dependant on controller frequency
        self._current_gym_step = 0  # dependant on all in one planner frequency

    def set_observation_collector(self, observation_collector: ObservationCollectorAllInOne):
        self._observation_collector = observation_collector
        self._last_merged_obs = np.zeros(shape=self._observation_collector.get_observation_space().shape)

    def get_step_parameters(self):
        return self._run_all_agents_each_iteration, \
               self._all_in_one_planner_frequency, \
               self._update_global_plan_frequency, \
               self._stabilize_with_hard_coding

    def process_step(self, action: int):
        # get action from chosen model
        if self._run_all_agents_each_iteration:
            action_model = self._last_actions[action, :]
        else:
            action_model = self._local_planner_manager.execute_local_planner(action, self._last_obs_dict,
                                                                             clip=self._action_bounds)

        # publish action
        self._pub_action(action_model)

        # extract new observation
        merged_obs, made_new_global_plan = self._get_new_obs()

        # check for final
        reward, reward_info = self._reward_calculator.check_done(self._last_obs_dict['laser_scan'],
                                                                 self._last_obs_dict['global_goal_robot_frame'])

        if self._reward_calculator.extended_eval:
            comp_time = np.zeros((self._all_in_one_planner_frequency - 1,))

        # repeat with selected local planner
        for i in range(self._all_in_one_planner_frequency - 1):
            if not reward_info['is_done']:
                # execute and publish action

                if self._reward_calculator.extended_eval:
                    start_time = time.time()

                action_model = self._local_planner_manager.execute_local_planner(action, self._last_obs_dict,
                                                                                 clip=self._action_bounds)

                if self._reward_calculator.extended_eval:
                    end_time = time.time()
                    comp_time[i] = (end_time - start_time) * 1000

                self._pub_action(action_model)

                # extract new observation
                merged_obs, made_new_global_plan_tmp = self._get_new_obs()
                made_new_global_plan = made_new_global_plan or made_new_global_plan_tmp

                # calculate reward
                reward, reward_info = self._reward_calculator.check_done(self._last_obs_dict['laser_scan'],
                                                                         self._last_obs_dict['global_goal_robot_frame'])

        # calculate detailed reward
        reward, reward_info = self._get_reward(action_model, made_new_global_plan)

        if self._run_all_agents_each_iteration:
            self._last_merged_obs[:-2 * self._local_planner_manager.get_numb_models()] = merged_obs
        else:
            self._last_merged_obs = merged_obs

        if self._run_all_agents_each_iteration:
            # Execute all local planners
            self._last_actions = self._local_planner_manager.execute_local_planners(self._last_obs_dict,
                                                                                    clip=self._action_bounds)
            # add last actions of local planners to observation
            self._last_obs_dict['last_actions'] = self._last_actions
            self._last_merged_obs[:2 * self._local_planner_manager.get_numb_models()] = self._last_actions.flatten()

        # Check if maximum number of iterations is reached
        if self._current_gym_step > self._max_iterations:
            reward_info['is_done'] = True
            reward_info['is_success'] = 0
            reward_info['done_reason'] = 0
            if self._reward_calculator.extended_eval:
                reward_info['global_path_reward'] = self._reward_calculator.global_plan_reward

        if self._reward_calculator.extended_eval:
            reward_info['local_planner_comp_time'] = np.mean(comp_time)

        self._current_gym_step += 1

        next_forced_action = -1
        if not reward_info['is_done'] and self._stabilize_with_hard_coding:
            dynamic_scan_min = np.min(self._last_obs_dict['scan_dynamic'])
            if dynamic_scan_min < self._lower_stabilize_th:
                next_forced_action = self._lower_stabilize_agent
            if dynamic_scan_min > self._upper_stabilize_th:
                next_forced_action = self._upper_stabilize_agent

        return action_model, self._last_obs_dict, self._last_merged_obs, reward, reward_info, next_forced_action

    def reset(self):
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self.agent_action_pub.publish(Twist())
        merged_obs, obs_dict = self._observation_collector.get_observations(make_new_global_plan=True)
        self._last_obs_dict = obs_dict

        if self._run_all_agents_each_iteration:
            self._last_merged_obs[:-2 * self._local_planner_manager.get_numb_models()] = merged_obs
        else:
            self._last_merged_obs = merged_obs

        if self._run_all_agents_each_iteration:
            # add last actions to observation
            self._last_obs_dict['last_actions'] = self._local_planner_manager.execute_local_planners(
                self._last_obs_dict,
                clip=self._action_bounds)
            self._last_merged_obs[:self._local_planner_manager.get_numb_models() * 2] = self._last_actions.flatten()

        self._current_iteration = 0
        self._current_gym_step = 0

    def _get_new_obs(self):
        make_new_global_plan = self._current_iteration % self._update_global_plan_frequency == 0
        merged_obs, self._last_obs_dict = self._observation_collector.get_observations(make_new_global_plan=make_new_global_plan)
        self._current_iteration += 1

        return merged_obs, make_new_global_plan

    def _pub_action(self, action):
        action_msg = Twist()
        action_msg.linear.x = action[0]
        if len(action) == 2:
            # non holonomic
            action_msg.angular.z = action[1]
        else:
            # holonomic
            action_msg.linear.y = action[1]
            action_msg.angular.z = action[2]
        self.agent_action_pub.publish(action_msg)

    def _extract_step_parameters(self, config_path: str):
        with open(config_path, 'r') as config_json:
            config_data = json.load(config_json)
        assert config_data is not None, "Error: All in one parameter file cannot be found!"

        # check if all agents should be run before model selection
        if 'run_all_agents_each_iteration' in config_data and config_data['run_all_agents_each_iteration']:
            self._run_all_agents_each_iteration = True
        elif 'run_all_agents_each_iteration' not in config_data:
            self._run_all_agents_each_iteration = False
        else:
            self._run_all_agents_each_iteration = False
        # extract update rate of global planner
        if 'update_global_plan_frequency' in config_data:
            self._update_global_plan_frequency = config_data['update_global_plan_frequency']
        else:
            rospy.logwarn(
                "Parameter \"update_global_plan_frequency\" not found in config file. Use default value of 5!")
            self._update_global_plan_frequency = 5
        # extract frequency of all in one planner
        if 'all_in_one_planner_frequency' in config_data:
            self._all_in_one_planner_frequency = config_data['all_in_one_planner_frequency']
        else:
            rospy.logwarn(
                'Parameter \"all_in_one_planner_frequency\" not found in config file. Us edefault value of 3!')
            self._all_in_one_planner_frequency = 3

        if "stabilize with hard coding" in config_data:
            stabilize_data = config_data["stabilize with hard coding"]
            self._stabilize_with_hard_coding = True
            self._lower_stabilize_th = stabilize_data['lower_bound']
            self._upper_stabilize_th = stabilize_data['upper_bound']
            model_names = self._local_planner_manager.get_model_names()
            if stabilize_data['lower_agent'] not in model_names:
                rospy.logerr("Can't find " + stabilize_data['lower_agent'] + " in agent list!")
            else:
                self._lower_stabilize_agent = model_names.index(stabilize_data['lower_agent'])
            if stabilize_data['lower_agent'] not in model_names:
                rospy.logerr("Can't find " + stabilize_data['upper_agent'] + " in agent list!")
            else:
                self._upper_stabilize_agent = model_names.index(stabilize_data['upper_agent'])
        else:
            self._stabilize_with_hard_coding = False

    def _get_reward(self, action_model, made_new_global_plan: bool):
        return self._reward_calculator.get_reward(
            self._last_obs_dict['laser_scan'], self._last_obs_dict['global_goal_robot_frame'],
            action=action_model, global_plan=self._last_obs_dict['global_plan'],
            robot_pose=self._last_obs_dict['robot_pose'], sub_goal=self._last_obs_dict['goal_reward_robot_frame'],
            sub_goal_old=self._last_obs_dict['goal_reward_robot_frame_old'],
            new_global_plan=made_new_global_plan, scan_dynamic=self._last_obs_dict['scan_dynamic'])
