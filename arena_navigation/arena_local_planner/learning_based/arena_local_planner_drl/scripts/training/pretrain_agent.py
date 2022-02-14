import json
import os
import sys
import time
from multiprocessing import Process, Manager

import numpy as np
import rospkg
import rospy
import torch as th
import yaml
from joblib._multiprocessing_helpers import mp
from rl_agent.envs.all_in_one_flatland_gym_env import AllInOneEnv
from stable_baselines3.common.vec_env import VecNormalize
from torch import nn, optim
from torch.optim.lr_scheduler import StepLR
from torch.utils.data import Dataset, random_split
from tqdm import tqdm

from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.rl_agent.envs.all_in_one_models.drl.drl_agent import \
    setup_and_start_drl_server

### NOTE: This pretraining procedures is based on https://github.com/Stable-Baselines-Team/rl-colab-notebooks/blob/sb3/pretraining.ipynb


# PARAMETERS
batch_size = 64
epochs = 8
scheduler_gamma = 0.6
learning_rate = 0.5
weight_decay = 5e-6
seed = 42
test_batch_size = 64


def pretrain_agent(policy, env_vec: VecNormalize, expert_dataset_file, test_portion: float = 0.05,
                   normalize_obs: bool = False):
    print("\n\n_________________________")
    print("\nStart pretraining...\n")
    print("\nLoad expert dataset...\n")

    # 1. Load dataset
    with np.load(expert_dataset_file) as expert_data:
        expert_actions = expert_data['expert_actions']
        expert_observations = expert_data['expert_observations']

    # 2.1 Train observation normalization
    print("Tune observation normalization...")

    print(expert_observations.size)

    if expert_observations.shape[0] > 150000:
        env_vec.obs_rms.update(expert_observations[:150000])
    else:
        env_vec.obs_rms.update(expert_observations)

    print("\nApply observation normalization...\n")

    # 2.2 Apply normalization to observation vector
    if normalize_obs:
        # split array to decrease memory overhead
        split_size = 4
        expert_observations = np.split(expert_observations, split_size)

        # apply obs normalization
        for j in range(split_size):
            expert_observations[j] = np.clip(
                (expert_observations[j] - env_vec.obs_rms.mean) / np.sqrt(env_vec.obs_rms.var + env_vec.epsilon),
                -env_vec.clip_obs, env_vec.clip_obs)
            print("Applied to " + str((j + 1) * (100.0 / split_size)) + "% of observations!")

        # save normalized obs dataset
        print("\nSave normalized dataset...\n")
        expert_dataset_file_new = ''.join(expert_dataset_file.split())[:-4] + "_normalized.npz"
        save_dataset(expert_dataset_file_new, expert_actions, np.concatenate(expert_observations))
        expert_dataset = ExpertDataSet(np.concatenate(expert_observations), expert_actions)
    else:
        expert_dataset = ExpertDataSet(expert_observations, expert_actions)

    print("Done!\n")

    # 3. Define training and test set
    train_size = int((1 - test_portion) * len(expert_dataset))
    test_size = len(expert_dataset) - train_size
    train_expert_dataset, test_expert_dataset = random_split(
        expert_dataset, [train_size, test_size]
    )
    print("test_expert_dataset: ", len(test_expert_dataset))
    print("train_expert_dataset: ", len(train_expert_dataset))
    print("\n")

    # 4. Train policy
    use_cuda = th.cuda.is_available()
    th.manual_seed(seed)
    device = th.device("cuda" if use_cuda else "cpu")
    kwargs = {"num_workers": 1, "pin_memory": True} if use_cuda else {}

    criterion = nn.CrossEntropyLoss()  # Discrete action space

    model = policy.to(device)
    # Here, we use PyTorch `DataLoader` to our load previously created `ExpertDataset` for training
    # and testing
    train_loader = th.utils.data.DataLoader(
        dataset=train_expert_dataset, batch_size=batch_size, shuffle=True, **kwargs
    )
    test_loader = th.utils.data.DataLoader(
        dataset=test_expert_dataset, batch_size=test_batch_size, shuffle=True, **kwargs,
    )

    # Define an Optimizer and a learning rate schedule.
    optimizer = optim.SGD(model.parameters(), lr=learning_rate, weight_decay=weight_decay)
    scheduler = StepLR(optimizer, step_size=1, gamma=scheduler_gamma)

    # Train
    train_loss = np.empty((epochs,))
    test_loss = np.empty((epochs + 1,))

    test_loss[0] = _test(model, device, test_loader, criterion)
    for epoch in range(1, epochs + 1):
        train_loss[epoch - 1] = _train(model, device, train_loader, optimizer, criterion, epoch)
        test_loss[epoch] = _test(model, device, test_loader, criterion)
        scheduler.step()

    return policy, test_loss, train_loss


def _train(model, device, train_loader, optimizer, criterion, epoch):
    model.train()

    train_loss = 0
    for batch_idx, (data, target) in enumerate(train_loader):
        data, target = data.to(device), target.to(device)
        optimizer.zero_grad()

        # Retrieve the logits for PPO when using discrete actions
        latent_pi, _, _ = model._get_latent(data)
        logits = model.action_net(latent_pi)
        action_prediction = logits
        target = target.long()

        loss = criterion(action_prediction, target)
        loss.backward()
        optimizer.step()

        train_loss += loss.item() * data.size(0)

    avg_loss = train_loss / len(train_loader.dataset)
    print("Train Epoch: {} \tLoss: {:.6f}".format(epoch, avg_loss, ))

    return avg_loss


def _test(model, device, test_loader, criterion):
    model.eval()
    test_loss = 0
    with th.no_grad():
        for data, target in test_loader:
            data, target = data.to(device), target.to(device)

            # Retrieve the logits for A2C/PPO when using discrete actions
            latent_pi, _, _ = model._get_latent(data)
            logits = model.action_net(latent_pi)
            action_prediction = logits
            target = target.long()
            loss = criterion(action_prediction, target)
            test_loss += loss.item() * data.size(0)

    test_loss /= len(test_loader.dataset)
    print(f"Test set: Average loss: {test_loss:.4f}")

    return test_loss


def _get_paths(all_in_one_config: str = "all_in_one_default.json") -> dict:
    """
    Function to generate agent specific paths

    :param agent_name: Precise agent name (as generated by get_agent_name())
    :param args (argparse.Namespace): Object containing the program arguments
    """
    dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    robot_model = rospy.get_param('/robot_model')
    paths = {
        'robot_setting': os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', robot_model +
                                          '.model.yaml'),
        'hyperparams':
            os.path.join(
                dir, 'configs', 'hyperparameters'),
        'robot_as':
            os.path.join(
                dir, 'configs', 'default_settings.yaml'),
        'all_in_one_parameters':
            os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'agent_parameters', 'pretrain_configs',
                         all_in_one_config),
        'drl_agents':
            os.path.join(
                dir, 'agents', 'rosnav-agents'),
        'map_folder': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters'),
        'map_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters',
                                       "map_curriculum_12envs.yaml")
    }
    return paths


def _make_env(paths: dict, rank: int):
    robot_model = rospy.get_param("/robot_model")

    params_path = os.path.join(paths['hyperparams'], robot_model + '_default.json')
    with open(params_path) as f:
        params = json.load(f)
    paths['map_parameters'] = os.path.join(paths['map_folder'], 'tmp', "map_" + str(rank) + ".json")
    return AllInOneEnv("sim_{}".format(rank + 1), paths['robot_setting'], paths['robot_as'],
                       params['reward_fnc'],
                       goal_radius=params['goal_radius'], paths=paths,
                       max_steps_per_episode=params['train_max_steps_per_episode'], drl_server="tcp://localhost:5555")


def unzip_map_parameters(paths: dict, numb_envs: int):
    if not os.path.exists(os.path.join(paths['map_folder'], 'tmp')):
        os.makedirs(os.path.join(paths['map_folder'], 'tmp'))
    with open(paths['map_parameters'], "r") as map_yaml:
        map_data = yaml.safe_load(map_yaml)
        for i in range(numb_envs):
            env_map_data = map_data[i + 1]
            map_env_path = os.path.join(paths['map_folder'], 'tmp', "map_" + str(i) + ".json")
            with open(map_env_path, "w") as map_json:
                json.dump(env_map_data, map_json)


class ExpertDataSet(Dataset):
    def __init__(self, expert_observations, expert_actions):
        self.observations = expert_observations
        self.actions = expert_actions

    def __getitem__(self, index):
        return self.observations[index], self.actions[index]

    def __len__(self):
        return len(self.observations)


def _create_expert_dataset(shared_array_actions,
                           shared_array_obs,
                           paths: dict,
                           iterations: int,
                           rank: int,
                           use_dynamic_scan: bool,
                           laser_stack_size: int,
                           scan_size: int,
                           robot_state_size: int):

    print("Start process " + str(rank))

    # 1. Create Gym Env
    env = _make_env(paths, rank)

    # 2. Define expert based on fully observable dynamic scan
    def simple_all_in_one(obs):
        _L = scan_size  # laser scan size
        switch_distance = 1.8
        last_laser_scan_dynamic = obs[_L:2 * _L]
        min_dist = np.min(last_laser_scan_dynamic)
        if min_dist <= switch_distance:
            return 0
        else:
            return 1

    # 3. Create expert trajectory
    if use_dynamic_scan:
        obs_space_size = 2 * laser_stack_size * scan_size + robot_state_size
    else:
        obs_space_size = laser_stack_size * scan_size + robot_state_size
    expert_observations = np.empty((iterations,) + (obs_space_size,))
    expert_actions = np.empty((iterations,), dtype=np.int)
    obs = env.reset()

    # do one empty iteration to load everything
    action = simple_all_in_one(obs)
    env.step(action)
    obs = env.reset()

    for i in tqdm(range(iterations)):
        action = simple_all_in_one(obs)

        if not use_dynamic_scan:
            # strap dynamic laser scan from observation vector
            _L = scan_size  # laser scan size
            dynamic_scan_indices = []
            for j in range(laser_stack_size):
                dynamic_scan_indices += list(range(_L * (2 * j + 1), _L * (2 * j + 2)))

            obs = np.delete(obs, dynamic_scan_indices)

        expert_observations[i] = obs
        expert_actions[i] = action
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()

    shared_array_actions[:] = expert_actions
    shared_array_obs[:] = expert_observations.flatten()


def save_dataset(save_path: str, expert_actions: np.ndarray, expert_observations: np.ndarray):
    # Save dataset
    np.savez_compressed(
        save_path,
        expert_actions=expert_actions,
        expert_observations=expert_observations,
    )

    # Create and return torch expert dataset
    expert_dataset = ExpertDataSet(expert_observations, expert_actions)
    return expert_dataset


def get_aio_parameters(all_in_one_config_path: str):
    with open(all_in_one_config_path, 'r') as config_json:
        all_in_one_config = json.load(config_json)
    assert all_in_one_config is not None, "All in one parameter file cannot be found!"
    obs_info = all_in_one_config['observation_space']
    if obs_info['laser_range'] == 'full':
        reduced_laser_size = -1
    else:
        assert 'size_laser_vector' in obs_info, "Please specify the size_laser_vector parameter in the " \
                                                "aoi config file or use laser_range=full setting otherwise!"
        reduced_laser_size = obs_info['size_laser_vector']

    assert 'laser_stack_size' in obs_info, "Please specify the laser_stack_size parameter in the aoi " \
                                           "config file!"

    laser_stack_size = obs_info['laser_stack_size']

    if 'add_robot_velocity' in obs_info and obs_info['add_robot_velocity']:
        add_robot_velocity = True
    else:
        add_robot_velocity = False

    return reduced_laser_size, laser_stack_size, add_robot_velocity


def get_numb_laser_beams(robot_yaml_path: str):
    with open(robot_yaml_path, "r") as fd:
        robot_data = yaml.safe_load(fd)
        for plugin in robot_data["plugins"]:
            if plugin["type"] == "Laser":
                laser_angle_min = plugin["angle"]["min"]
                laser_angle_max = plugin["angle"]["max"]
                laser_angle_increment = plugin["angle"]["increment"]
                laser_num_beams = int(
                    round(
                        (laser_angle_max - laser_angle_min)
                        / laser_angle_increment
                    )
                )
                return laser_num_beams

if __name__ == '__main__':
    args = sys.argv
    iterations = int(args[1])
    save_file_name = args[2]
    num_envs = int(args[3])
    aio_config = args[4]
    use_dynamic_scan = args[5].lower() == 'true'

    iterations_per_worker = int(iterations / num_envs)

    paths = _get_paths(aio_config)
    unzip_map_parameters(paths, num_envs)

    laser_num_beams = get_numb_laser_beams(paths['robot_setting'])

    # read parameters from aio config file
    all_in_one_config_path = paths['all_in_one_parameters']
    reduced_laser_size, laser_stack_size, add_robot_velocity = get_aio_parameters(all_in_one_config_path)
    if reduced_laser_size < 0:
        scan_size = laser_num_beams
    else:
        scan_size = reduced_laser_size

    if add_robot_velocity:
        robot_state_size = 6
    else:
        robot_state_size = 4

    # create drl server for env
    server_base_url = "tcp://*:5555"
    client_base_url = "tcp://localhost:5555"
    drl_server = Process(target=setup_and_start_drl_server, args=(server_base_url, paths))
    drl_server.start()

    processes = []

    if use_dynamic_scan:
        obs_space_size = 2 * laser_stack_size * scan_size + robot_state_size
    else:
        obs_space_size = laser_stack_size * scan_size + robot_state_size

    expert_obs = np.empty((num_envs, iterations_per_worker, obs_space_size))
    expert_actions = np.empty((num_envs, iterations_per_worker), dtype=np.int)

    for i in range(num_envs):
        manager = Manager()
        shared_array_actions = mp.RawArray('i', iterations_per_worker)
        shared_array_obs = mp.RawArray('f', iterations_per_worker * obs_space_size)
        p = Process(target=_create_expert_dataset,
                    args=(shared_array_actions, shared_array_obs, paths, iterations_per_worker, i, use_dynamic_scan,
                          laser_stack_size, scan_size, robot_state_size))
        p.start()
        time.sleep(12)
        processes.append((p, shared_array_actions, shared_array_obs))

    for i in range(num_envs):
        (p, shared_array_actions, shared_array_obs) = processes[i]
        p.join()
        shared_array_actions_np = np.reshape(np.frombuffer(shared_array_actions, dtype=np.uint32), -1)
        shared_array_obs_np = np.reshape(np.frombuffer(shared_array_obs, dtype=np.float32),
                                         (iterations_per_worker, obs_space_size))
        expert_obs[i, :, :] = shared_array_obs_np[:, :]
        expert_actions[i, :] = shared_array_actions_np[:]

    for i in range(num_envs):
        (p, _, _) = processes[i]
        if p.is_alive():
            p.kill()
    del processes

    base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    save_path = os.path.join(base_dir, "agents", 'aio-agents', "pretrained_aio_agents", save_file_name + ".npz")
    save_dataset(save_path, expert_actions.flatten(), expert_obs.reshape(-1, expert_obs.shape[-1]))

    print("Dataset saved!")

    del expert_actions, expert_obs, drl_server

    sys.exit()
