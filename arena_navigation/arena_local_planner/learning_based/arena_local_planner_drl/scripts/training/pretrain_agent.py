import json
import os
import sys

import numpy as np
import rospkg
import torch as th
from rl_agent.envs.all_in_one_flatland_gym_env import AllInOneEnv
from stable_baselines3.common.vec_env import VecNormalize
from torch import nn, optim
from torch.optim.lr_scheduler import StepLR
from torch.utils.data import Dataset, random_split
from tqdm import tqdm

### NOTE: This pretraining procedures is based on https://github.com/Stable-Baselines-Team/rl-colab-notebooks/blob/sb3/pretraining.ipynb

# PARAMETERS
batch_size = 64
epochs = 10
scheduler_gamma = 0.5
learning_rate = 0.4
weight_decay = 1e-3
seed = 1
test_batch_size = 64


def pretrain_agent(policy, env_vec: VecNormalize, iterations: int, test_portion: float = 0.05,
                   expert_dataset_file=None):
    # 1. Get expert dataset
    if expert_dataset_file is None:
        # 1.1 Create dataset
        base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
        save_path = os.path.join(base_dir, "agents", "pretrained_aio_agents", "expert_dataset_1.npz")
        expert_dataset = _create_expert_dataset(save_path, iterations)
        expert_observations = expert_dataset.observations
        expert_actions = expert_dataset.actions
    else:
        # 1.2 Load dataset
        with np.load(expert_dataset_file) as expert_data:
            expert_actions = expert_data['expert_actions']
            expert_observations = expert_data['expert_observations']

    print("\nStart pretraining...\n")

    # 2.1 Train observation normalization
    print("Tune observation normalization...")
    for obs in expert_observations:
        env_vec.obs_rms.update(obs)

    # 2.2 Apply normalization to observation vector
    expert_observations_normalized = np.array(list(map(env_vec.normalize_obs, expert_observations)))
    expert_dataset = ExpertDataSet(expert_observations_normalized, expert_actions)

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
    test_loss = np.empty((epochs+1,))

    test_loss[0] = _test(model, device, test_loader, criterion)
    for epoch in range(1, epochs + 1):
        train_loss[epoch-1] = _train(model, device, train_loader, optimizer, criterion, epoch)
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

    paths = {
        'robot_setting':
            os.path.join(
                rospkg.RosPack().get_path('simulator_setup'),
                'robot', 'myrobot' + '.model.yaml'),
        'hyperparams':
            os.path.join(
                dir, 'configs', 'hyperparameters'),
        'robot_as':
            os.path.join(
                dir, 'configs', 'default_settings.yaml'),
        'all_in_one_parameters':
            os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'agent_parameters', all_in_one_config),
        'drl_agents':
            os.path.join(
                dir, 'agents'),
        'map_folder': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters'),
        'map_parameters': os.path.join(dir, 'configs', 'all_in_one_hyperparameters', 'map_parameters',
                                       "indoor_obs14.json")
    }
    return paths


def _make_env(paths: dict):
    params_path = os.path.join(paths['hyperparams'], 'all_in_one_default.json')
    with open(params_path) as f:
        params = json.load(f)

    return AllInOneEnv("sim_1", paths['robot_setting'], paths['robot_as'],
                       params['reward_fnc'],
                       goal_radius=params['goal_radius'], paths=paths,
                       max_steps_per_episode=params['train_max_steps_per_episode'])


class ExpertDataSet(Dataset):
    def __init__(self, expert_observations, expert_actions):
        self.observations = expert_observations
        self.actions = expert_actions

    def __getitem__(self, index):
        return self.observations[index], self.actions[index]

    def __len__(self):
        return len(self.observations)


def _create_expert_dataset(save_path: str, iterations: int):
    print("Create expert dataset for {} iterations.".format(iterations))

    # 1. Create Gym Env
    paths = _get_paths("all_in_one_pretrain.json")
    env = _make_env(paths)

    # 2. Define expert based on fully observable dynamic scan
    def simple_all_in_one(obs):
        _L = 360  # laser scan size
        switch_distance = 1.2
        last_laser_scan_dynamic = obs[_L:2 * _L]
        min_dist = np.min(last_laser_scan_dynamic)
        if min_dist <= switch_distance:
            return 0
        else:
            return 1

    # 3. Create expert trajectory
    expert_observations = np.empty((iterations,) + (env.observation_space.shape[0] - 3 * 360,))
    expert_actions = np.empty((iterations,))
    obs = env.reset()

    for i in tqdm(range(iterations)):
        action = simple_all_in_one(obs)

        # strap dynamic laser scan from observation vector
        _L = 360  # laser scan size
        dynamic_scan_indices = list(range(_L, 2 * _L)) + list(range(3 * _L, 4 * _L)) + list(range(5 * _L, 6 * _L))
        obs = np.delete(obs, dynamic_scan_indices)

        expert_observations[i] = obs
        expert_actions[i] = action
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()

    print("Save expert dataset!")

    # 4. Save dataset
    np.savez_compressed(
        save_path,
        expert_actions=expert_actions,
        expert_observations=expert_observations,
    )

    # 5. Create and return torch expert dataset
    expert_dataset = ExpertDataSet(expert_observations, expert_actions)
    return expert_dataset


if __name__ == '__main__':
    args = sys.argv
    iterations = int(args[1])
    name = args[2]
    base_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    save_path = os.path.join(base_dir, "agents", "pretrained_aio_agents", name + ".npz")
    _create_expert_dataset(save_path, iterations)
