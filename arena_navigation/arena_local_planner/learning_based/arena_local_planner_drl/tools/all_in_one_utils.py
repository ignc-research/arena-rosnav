import csv
import math
import time
from shutil import copyfile

import numpy as np
import rospy
import torch
import visualization_msgs
from stable_baselines3.common.vec_env import VecNormalize
from visualization_msgs.msg import Marker


class Evaluator:
    def __init__(self):
        self.colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 0.0], [1.0, 0.0, 1.0],
                       [0.0, 1.0, 1.0]]  # TODO This is not optimal as only 6 models can be visualized
        self.agent_visualization = rospy.Publisher('/eval_sim/all_in_one_action_prob_vis', Marker, queue_size=1)

    def evaluate_policy_manually(self, policy: callable, action_probs_func: callable, env: VecNormalize, episodes: int,
                                 log_folder: str, gamma: float,
                                 all_in_config_file: str, log_statistics=True):
        gamma = 0.995
        rewards = np.zeros((episodes,))
        global_path_rewards = np.zeros((episodes,))
        collisions = np.zeros((episodes,))
        distance_travelled = []
        travel_time = []
        is_success = np.zeros((episodes,))
        computation_times = []
        computation_times_local_planner = []

        model_distribution = np.zeros((episodes, env.env_method("get_number_models")[0]))
        model_distribution_close_obst_dist = np.zeros((episodes, env.env_method("get_number_models")[0]))
        model_distribution_medium_obst_dist = np.zeros((episodes, env.env_method("get_number_models")[0]))
        model_distribution_large_obst_dist = np.zeros((episodes, env.env_method("get_number_models")[0]))

        model_names = env.env_method('get_model_names')[0]

        # run evaluation
        for i in range(episodes):
            print('Episode {:d} / {:d}'.format(i, episodes))
            obs = env.reset()[0]
            done = False
            current_reward = 0
            current_iteration = 0
            while not done:
                action = np.array([policy(obs)])
                obs_forward = torch.as_tensor([obs]).to("cuda")
                possible_actions = [i for i in range(env.env_method("get_number_models")[0])]
                possible_actions_tensor = torch.as_tensor(possible_actions).to("cuda")
                with torch.no_grad():
                    action_probs_tensor = action_probs_func(obs_forward, possible_actions_tensor)
                if torch.is_tensor(action_probs_tensor):
                    action_probs = action_probs_tensor.cpu().numpy()
                else:
                    action_probs = np.array(action_probs_tensor)
                self.visualize_action_probs(action_probs, model_names)

                if log_statistics:
                    start_time = time.time()

                time.sleep(0.05)
                obs, reward, done, info = env.step(action)

                if log_statistics:
                    end_time = time.time()
                    comp_time_in_ms = (end_time - start_time) * 1000.0
                    computation_times.append(comp_time_in_ms)

                info = info[0]
                done = done[0]
                reward = env.get_original_reward()[0]
                obs = obs[0]

                if log_statistics:
                    computation_times_local_planner.append(info['local_planner_comp_time'])

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

        if log_statistics:
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
                file.write("Mean computation per local planner iteration " + str(np.mean(computation_times_local_planner)) + "\n")
                file.write("Mean model distribution: " + str(np.mean(model_distribution, axis=0)) + "\n")
                file.write("Mean model distribution close obstacle distance: " + str(
                    np.mean(model_distribution_close_obst_dist, axis=0)) + "\n")
                file.write("Mean model distribution medium obstacle distance: " + str(
                    np.mean(model_distribution_medium_obst_dist, axis=0)) + "\n")
                file.write("Mean model distribution large obstacle distance: " + str(
                    np.mean(model_distribution_large_obst_dist, axis=0)) + "\n")
                file.write("With models: " + str(env.env_method("get_model_names")[0]))

            with open(log_folder + '/evaluation_summary.csv', 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["Mean success rate", str(np.mean(is_success))])
                writer.writerow(["Mean collisions", str(np.mean(collisions))])
                writer.writerow(["Mean time", str(np.mean(travel_time))])
                writer.writerow(["Mean distance travelled", str(np.mean(distance_travelled))])
                writer.writerow(["Mean reward",  str(np.mean(rewards))])
                writer.writerow(["Mean computation time per second simulation time", str(comp_times_mean_per_second)])
                writer.writerow(["Mean computation per local planner iteration", str(
                    np.mean(computation_times_local_planner))])
                writer.writerow(["Mean model distribution", str(np.mean(model_distribution, axis=0))])

            # copy config file
            copyfile(all_in_config_file, log_folder + '/all_in_one_parameters.json')

    def visualize_action_probs(self, action_probs: [float], names: [str]):
        for i in range(action_probs.size):
            marker = Marker()
            marker.header.stamp = rospy.get_rostime()
            marker.header.frame_id = 'map'
            marker.id = 200 + i

            marker.type = visualization_msgs.msg.Marker.TEXT_VIEW_FACING
            marker.action = visualization_msgs.msg.Marker.ADD

            marker.color.r = self.colors[i][0]
            marker.color.g = self.colors[i][1]
            marker.color.b = self.colors[i][2]
            marker.color.a = 1

            marker.pose.position.x = i + 1
            marker.pose.position.y = -3

            marker.scale.z = 1

            marker.text = names[i] + ": " + "{:.2f}".format((math.e ** action_probs[i]) * 100) + "%"

            self.agent_visualization.publish(marker)
