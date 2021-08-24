import csv
import time
from shutil import copyfile

import numpy as np
from stable_baselines3.common.vec_env import VecNormalize


def evaluate_policy_manually(policy: callable, env: VecNormalize, episodes: int, log_folder: str, gamma: float,
                             all_in_config_file: str):
    gamma = 1
    rewards = np.zeros((episodes,))
    global_path_rewards = np.zeros((episodes,))
    collisions = np.zeros((episodes,))
    distance_travelled = []
    travel_time = []
    is_success = np.zeros((episodes,))
    computation_times = []

    model_distribution = np.zeros((episodes, env.env_method("get_number_models")[0]))
    model_distribution_close_obst_dist = np.zeros((episodes, env.env_method("get_number_models")[0]))
    model_distribution_medium_obst_dist = np.zeros((episodes, env.env_method("get_number_models")[0]))
    model_distribution_large_obst_dist = np.zeros((episodes, env.env_method("get_number_models")[0]))

    # run evaluation
    for i in range(episodes):
        print('Episode {:d} / {:d}'.format(i, episodes))
        obs = env.reset()[0]
        done = False
        current_reward = 0
        current_iteration = 0
        while not done:
            action = np.array([policy(obs)])
            start_time = time.time()
            obs, reward, done, info = env.step(action)
            end_time = time.time()
            comp_time_in_ms = (end_time - start_time) * 1000.0
            computation_times.append(comp_time_in_ms)
            info = info[0]
            done = done[0]
            reward = reward[0]
            obs = obs[0]
            current_reward += reward * (gamma ** current_iteration)
            current_iteration += 1
            if done:
                rewards[i] = current_reward
                global_path_rewards[i] = info['global_path_reward']
                collisions[i] = info['collisions']
                is_success[i] = info['is_success']
                if is_success[i] == 1:
                    distance_travelled.append(info['distance_travelled'])
                    travel_time.append(info['time'])

                model_distribution[i, :] = info['model_distribution']
                model_distribution_close_obst_dist[i, :] = info['model_distribution_close_obst_dist']
                model_distribution_medium_obst_dist[i, :] = info['model_distribution_medium_obst_dist']
                model_distribution_large_obst_dist[i, :] = info['model_distribution_large_obst_dist']

    # remove empty entries
    model_distribution_close_obst_dist = [i for i in model_distribution_close_obst_dist if np.sum(i) != 0]
    model_distribution_medium_obst_dist = [i for i in model_distribution_medium_obst_dist if np.sum(i) != 0]
    model_distribution_large_obst_dist = [i for i in model_distribution_large_obst_dist if np.sum(i) != 0]

    comp_times_mean_per_second = np.mean(computation_times) * (
            10.0 / env.env_method("get_all_in_one_planner_frequency")[0])
    # save results
    with open(log_folder + '/evaluation_full.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["rewards", "collisions", "is_success"])
        for i in range(episodes - 1):
            writer.writerow(["{:.2f}".format(rewards[i]), collisions[i], is_success[i]])

    with open(log_folder + "/evaluation_summary.txt", 'w') as file:
        file.write("Mean reward: " + str(np.mean(rewards)) + "\n")
        file.write("Mean global path reward: " + str(np.mean(global_path_rewards)) + "\n")
        file.write("Mean collisions: " + str(np.mean(collisions)) + "\n")
        file.write("Mean distance travelled: " + str(np.mean(distance_travelled)) + "\n")
        file.write("Mean time: " + str(np.mean(travel_time)) + "\n")
        file.write("Mean success rate: " + str(np.mean(is_success)) + "\n")
        file.write("Mean computation time per second simulation time " + str(comp_times_mean_per_second) + "\n")
        file.write("Mean model distribution: " + str(np.mean(model_distribution, axis=0)) + "\n")
        file.write("Mean model distribution close obstacle distance: " + str(
            np.mean(model_distribution_close_obst_dist, axis=0)) + "\n")
        file.write("Mean model distribution medium obstacle distance: " + str(
            np.mean(model_distribution_medium_obst_dist, axis=0)) + "\n")
        file.write("Mean model distribution large obstacle distance: " + str(
            np.mean(model_distribution_large_obst_dist, axis=0)) + "\n")
        file.write("With models: " + str(env.env_method("get_model_names")[0]))

    # copy config file
    copyfile(all_in_config_file, log_folder + '/all_in_one_parameters.json')
