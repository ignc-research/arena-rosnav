import json
import os
import pickle
from multiprocessing import Process
from typing import Dict

import numpy as np
import zmq
from rl_agent.envs.all_in_one_models.model_base_class import ModelBase
from stable_baselines3 import PPO

# NOTE: Is is assumed that the input laser for the drl planner is:
# angle: {min: -1.5707963267948966, max: 4.694936014, increment:  0.017453292}


class DrlAgent(ModelBase):

    def __init__(self, save_path: str, name: str):
        observation_information = {'laser_scan': True,
                                   'goal_in_robot_frame': True}
        super(DrlAgent, self).__init__(observation_information, name)

        model_save_path = os.path.join(save_path, "best_model")
        vec_norm_save_path = os.path.join(save_path, "vec_normalize.pkl")

        if os.path.exists(vec_norm_save_path):
            with open(vec_norm_save_path, "rb") as file_handler:
                vec_normalize = pickle.load(file_handler)
            self._obs_norm_func = vec_normalize.normalize_obs
            self._obs_normalize = True
        else:
            self._obs_normalize = False

        # For python3.8+
        custom_objects = {
            "learning_rate": 0.0,
            "lr_schedule": lambda _: 0.0,
            "clip_range": lambda _: 0.0,
        }
        self._model = PPO.load(model_save_path, custom_objects=custom_objects).policy

    def get_next_action(self, observation_dict, observation_array=None) -> np.ndarray:
        # merge observation
        if observation_array is None:
            obs_twist = observation_dict['robot_twist']
            merged_obs = np.hstack([observation_dict['laser_scan'], np.array(observation_dict['goal_in_robot_frame']),
                                    np.array([obs_twist.linear.x, obs_twist.linear.y, obs_twist.angular.z])])
        else:
            merged_obs = np.copy(observation_array)

        # shift laser scan measurement
        # merged_obs[:360] = self._shift_scan(merged_obs[:360])

        # normalize
        if self._obs_normalize:
            merged_obs = self._obs_norm_func(merged_obs)

        return self._model.predict(merged_obs, deterministic=True)[0]

    def wait_for_agent(self):
        return True

    def reset(self):
        pass

    def _shift_scan(self, scan: np.ndarray):
        # old angle: {min: 0, max: 6.28318977356, increment: 0.0175019223243}
        # new angle: {min: -1.5707963267948966, max: 4.694936014, increment:  0.017453292}
        sub_array = np.hsplit(scan, 4)
        new_scan = np.concatenate((sub_array[3], sub_array[0], sub_array[1], sub_array[2]))
        return new_scan

class DrlAgentServer:

    def __init__(self, socket_name: str, drl_agents: Dict[str, DrlAgent]):
        context = zmq.Context()
        self._socket = context.socket(zmq.REP)
        self._socket.bind(socket_name)
        self._drl_agents = drl_agents

    def start(self):
        while True:
            md = self._socket.recv_json()
            msg = self._socket.recv()
            buf = memoryview(msg)
            obs = np.frombuffer(buf, dtype=md['dtype'])
            obs.reshape(md['shape'])

            drl_agent_name = md['drl_agent_name']
            action = self._drl_agents[drl_agent_name].get_next_action(dict(), obs)

            #  Send reply back to client
            self._socket.send(action.astype(np.float32))


class DrlAgentClient(ModelBase):

    def __init__(self, server_socket_name: str, name: str):
        observation_info = {'laser_scan': True,
                                   'goal_in_robot_frame': True}
        super().__init__(observation_info, name)

        context = zmq.Context()
        self._socket = context.socket(zmq.REQ)
        self._socket.connect(server_socket_name)

    def get_next_action(self, observation_dict) -> np.ndarray:
        obs_twist = observation_dict['robot_twist']
        merged_obs = np.hstack([observation_dict['laser_scan'], np.array(observation_dict['goal_in_robot_frame']),
                                np.array([obs_twist.linear.x, obs_twist.linear.y, obs_twist.angular.z])])

        md = dict(
            dtype=str(merged_obs.dtype),
            shape=merged_obs.shape,
            drl_agent_name=self.get_name()
        )

        self._socket.send_json(md, zmq.SNDMORE)
        self._socket.send(merged_obs)
        action = self._socket.recv()
        action_np = np.frombuffer(action, dtype=np.float32)
        return np.copy(action_np)

    def wait_for_agent(self):
        return True

    def reset(self):
        pass


def load_drl_models_from_config(paths: dict) -> Dict[str, DrlAgent]:
    drl_models = dict()
    with open(paths['all_in_one_parameters'], 'r') as model_json:
        config_data = json.load(model_json)

    assert config_data is not None, "Error: All in one parameter file cannot be found!"

    config_model = config_data['models']

    if 'drl' in config_model:
        agent_dir_names = config_model['drl']
        drl_agent_names = config_model['drl_names']

        for i, agent_dir, in enumerate(agent_dir_names):
            drl_model = DrlAgent(os.path.join(paths['drl_agents'], agent_dir), config_model['drl_names'][i])
            drl_models[drl_agent_names[i]] = drl_model

    return drl_models


def setup_and_start_drl_server(server_socket_name: str, paths: dict):
    drl_models = load_drl_models_from_config(paths)
    drl_server = DrlAgentServer(server_socket_name, drl_models)
    drl_server.start()
