import json
import os
from shutil import copyfile

import numpy as np
import rospkg
import rospy
from rl_agent.envs.all_in_one_models.base_local_planner.base_local_planner_all_in_one_interface import \
    BaseLocalPlannerAgent
from rl_agent.envs.all_in_one_models.drl.drl_agent import DrlAgentClient, DrlAgent
from rl_agent.envs.all_in_one_models.rlca.rlca_agent import RLCAAgent
from rl_agent.envs.all_in_one_models.teb.teb_all_in_one_interface_agent import TebAllinOneInterfaceAgent


class LocalPlannerManager:

    def __init__(self, drl_server: str, paths: dict, ns: str, is_train_mode: bool):
        # set up model parameters
        self._drl_server = drl_server
        self._ns = ns
        self._is_train_mode = is_train_mode

        self._setup_model_configuration(paths)

    def get_numb_models(self) -> int:
        return len(self._models)

    def get_model_names(self) -> [str]:
        return self._model_names

    def get_required_observations(self) -> dict:
        return self._required_observations

    def wait_for_agents(self, sim_step: callable):
        # wait till all agents are ready
        all_ready = False
        while not all_ready:
            all_ready = True
            for i in self._models:
                all_ready = all_ready and i.wait_for_agent()
            if not all_ready:
                if sim_step is not None:
                    sim_step()
                rospy.loginfo("Environment " + self._ns + ": Not all agents ready yet! Wait...")

    def execute_local_planner(self, model_numb: int, obs_dict: dict, clip: [int]) -> np.array:
        action_model = np.array(self._models[model_numb].get_next_action(obs_dict))
        action_model[0] = np.clip(a=action_model[0], a_min=clip[0],
                                  a_max=clip[1])
        action_model[1] = np.clip(a=action_model[1], a_min=clip[2], a_max=clip[3])
        return action_model

    def execute_local_planners(self, obs_dict: dict, clip: [int]) -> np.array:
        last_actions = np.zeros(shape=(self.get_numb_models(), 2))
        for i, agent in enumerate(self._models):
            current_action = agent.get_next_action(obs_dict)
            # clip action
            current_action[0] = np.clip(a=current_action[0], a_min=clip[0], a_max=clip[1])
            current_action[1] = np.clip(a=current_action[1], a_min=clip[2], a_max=clip[3])
            last_actions[i, :] = current_action
        return last_actions

    def reset_planners(self):
        for agent in self._models:
            agent.reset()

    def close_planners(self):
        for m in self._models:
            m.close()

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

        # set up model based agents based on the all_in_one_base_local_planner package
        base_dir_agent = ''
        if 'model' in paths:
            base_dir_agent = paths['model']

        if 'model_based' in config_model:
            model_based_names = config_model['model_based_names']

            base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
            for i, model_based_config_path in enumerate(config_model['model_based']):
                # case 1: Load config file from agent folder
                model_based_config_path_full = os.path.join(base_dir_agent, model_based_config_path)
                if not os.path.exists(model_based_config_path_full):
                    # case 2: Load config file from config folder
                    model_based_config_path_full = os.path.join(base_dir, 'configs', 'all_in_one_hyperparameters',
                                                                'base_local_planner_parameters', model_based_config_path)
                models.append(BaseLocalPlannerAgent(model_based_names[i], self._ns, model_based_config_path_full))

        if 'teb' in config_model:
            teb_names = config_model['teb_names']
            base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
            for i, teb_config_path in enumerate(config_model['teb']):
                # case 1: Load config file from agent folder
                teb_config_path_full = os.path.join(base_dir_agent, teb_config_path)
                if not os.path.exists(teb_config_path_full):
                    # case 2: Load config file from config folder
                    teb_config_path_full = os.path.join(base_dir, 'configs', 'all_in_one_hyperparameters',
                                                        'base_local_planner_parameters', teb_config_path)
                models.append(TebAllinOneInterfaceAgent(teb_names[i], self._ns, teb_config_path_full))

        self._models = models

        # copy hyperparameter file and model based planners config files
        if self._is_train_mode and 'model' in paths:
            doc_location = os.path.join(paths['model'], "all_in_one_parameters.json")
            if not os.path.exists(doc_location):
                with open(doc_location, "w", encoding='utf-8') as target:
                    json.dump(config_data, target, ensure_ascii=False, indent=4)
            if 'model_based' in config_model:
                for model_based_config_path in config_model['model_based']:
                    full_dest_path = os.path.join(paths['model'], model_based_config_path)
                    base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
                    if not os.path.exists(full_dest_path):
                        source_file_path = os.path.join(base_dir, 'configs', 'all_in_one_hyperparameters',
                                                        'base_local_planner_parameters', model_based_config_path)
                        copyfile(source_file_path, full_dest_path)
            if 'teb' in config_model:
                for teb_config_path in config_model['teb']:
                    full_dest_path = os.path.join(paths['model'], teb_config_path)
                    base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
                    if not os.path.exists(full_dest_path):
                        source_file_path = os.path.join(base_dir, 'configs', 'all_in_one_hyperparameters',
                                                        'base_local_planner_parameters', teb_config_path)
                        copyfile(source_file_path, full_dest_path)

        # collect necessary observations
        self._required_observations = dict()
        for m in self._models:
            self._required_observations.update(m.get_observation_info())

        # collect model names
        self._model_names = [model.get_name() for model in self._models]
