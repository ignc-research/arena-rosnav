import csv
import json
import math
import random
from pathlib import Path

import numpy as np
import rospy
from nav_msgs.srv import GetMap
from stable_baselines3.common.vec_env import VecNormalize
from task_generator.task_generator.obstacles_manager import ObstaclesManager


def evaluate_policy_manually(policy: callable, env: VecNormalize, episodes: int, log_folder: str, gamma: float):
    rewards = np.zeros((episodes,))
    collisions = np.zeros((episodes,))
    distance_travelled = np.zeros((episodes,))
    time = np.zeros((episodes,))
    is_success = np.zeros((episodes,))
    model_distribution = np.zeros((episodes, env.env_method("get_number_models")[0]))

    # run evaluation
    for i in range(episodes):
        obs = env.reset()[0]
        done = False
        current_reward = 0
        current_iteration = 0
        while not done:
            action = np.array([policy(obs)])
            obs, reward, done, info = env.step(action)
            info = info[0]
            done = done[0]
            reward = reward[0]
            obs = obs[0]
            current_reward += reward * (gamma ** current_iteration)
            current_iteration += 1
            if done:
                rewards[i] = current_reward
                collisions[i] = info['collisions']
                distance_travelled[i] = info['distance_travelled']
                time[i] = info['time']
                is_success[i] = info['is_success']
                model_distribution[i, :] = info['model_distribution']

    # omit first run as it is often falsy (for move-base based models)
    rewards = rewards[1:]
    collisions = collisions[1:]
    distance_travelled = distance_travelled[1:]
    time = time[1:]
    is_success = is_success[1:]
    model_distribution = model_distribution[1:, :]

    # save results
    with open(log_folder + '/evaluation_full.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["rewards", "collisions", "distance_travelled", "time", "is_success"])
        for i in range(episodes - 1):
            writer.writerow(["{:.2f}".format(rewards[i]), collisions[i], "{:.2f}".format(distance_travelled[i]),
                             "{:.2f}".format(time[i]), is_success[i]])

    with open(log_folder + "/evaluation_summary.txt", 'w') as file:
        file.write("Mean reward: " + str(np.mean(rewards)) + "\n")
        file.write("Mean collisions: " + str(np.mean(collisions)) + "\n")
        file.write("Mean distance travelled: " + str(np.mean(distance_travelled)) + "\n")
        file.write("Mean time: " + str(np.mean(time)) + "\n")
        file.write("Mean success rate: " + str(np.mean(is_success)) + "\n")
        file.write("Mean model distribution: " + str(np.mean(model_distribution, axis=0)) + "\n")
        file.write("With models: " + str(env.env_method("get_model_names")[0]))


# def generate_evaluation_scenario(ns, dst_json_path: str, length: int = 100, repeats: int = 2, numb_static_obst: int = 6,
#                                  numb_dyn_obst: int = 15):
#     service_client_get_map = rospy.ServiceProxy('/static_map', GetMap)
#     map_response = service_client_get_map()
#
#     dst_json_path_ = Path(dst_json_path)
#     dst_json_path_.parent.mkdir(parents=True, exist_ok=True)
#     json_data = {}
#     obstacles_manager = ObstaclesManager(ns, map_response.map)
#     scenarios = []
#     for i in range(length):
#         scenario_i = {}
#         scenario_i['scene_name'] = 'scene_' + str(i)
#         scenario_i['repeats'] = repeats
#
#         scenario_i_dyn_obst = [generate_random_obstacle(is_dynamic=True,
#                                                         linear_velocity=0.3,
#                                                         angular_velocity_max=math.pi / 6,
#                                                         min_obstacle_radius=0.1,
#                                                         max_obstacle_radius=0.4) for _ in range(numb_dyn_obst)]
#
#         def num_static_obst_vertices(): return random.randint(3, 5)
#
#         scenario_i_static_obs = [generate_random_obstacle(is_dynamic=False,
#                                                           num_vertices=num_static_obst_vertices(),
#                                                           min_obstacle_radius=0.5,
#                                                           max_obstacle_radius=2
#                                                           ) for _ in range(numb_dyn_obst)]
#
#         scene1 = {}
#         scene2 = {}
#         scene1['scene_name'] = 'scene_1'
#         scene1['repeats'] = 2
#         scene1_dynamic_obstacles = {}
#         scene1_static_obstacles = {}
#         scene1_robot = {'start_pos': [
#             0.0, 0.0, 0.2], 'goal_pos': [4, 8, 0]}
#         # trigger is optional, if it is not given, it will be trigged immediately
#         scene1_dynamic_obstacles['dynamic_obs_0'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
#             0, 3, 0], 'waypoints': [[0, 7, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo', 'triggers': ['watcher_1']}
#         scene1_dynamic_obstacles['dynamic_obs_1'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
#             0, 4, 0], 'waypoints': [[8, 0, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo', 'triggers': ['watcher_2']}
#         # shape can be polygon or circle
#         scene1_static_obstacles['static_obs_1'] = {
#             'shape': 'polygon', 'vertices': [[2, 0.4], [2, 0.5], [5, 0.5], [5, 0.4]]}
#         scene1_static_obstacles['static_obs_2'] = {
#             'shape': 'polygon', 'vertices': [[3, 0.4], [3, 0.5], [7, 0.5]]}
#         scene1_static_obstacles['static_obs_2'] = {
#             'shape': 'circle', 'x': 4, 'y': 5, 'radius': 0.2}
#         scene1['dynamic_obstacles'] = scene1_dynamic_obstacles
#         scene1['static_obstacles'] = scene1_static_obstacles
#         scene1['robot'] = scene1_robot
#         scene1['watchers'] = {'watcher_1': {
#             'pos': [1, 1], 'range': 1}, 'watcher_2': {'pos': [5, 5], 'range': 2}}
#
#         scene2['scene_name'] = 'scene_2'
#         scene2['repeats'] = 1
#         scene2_dynamic_obstacles = {}
#         scene2_static_obstacles = {}
#         scene2_robot = {'start_pos': [
#             0.0, 0.1, 0.2], 'goal_pos': [1.5, 1.5, 0]}
#         scene2_dynamic_obstacles['dynamic_obs_0'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
#             7, 7, 0], 'waypoints': [[-4, 0, 0], [-4, -4, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo'}
#         scene2_dynamic_obstacles['dynamic_obs_1'] = {'obstacle_radius': 0.3, 'linear_velocity': 0.2, 'start_pos': [
#             10, 3, 0], 'waypoints': [[0, 4, 0], [-5, 0, 0]], 'is_waypoint_relative': True, 'mode': 'yoyo'}
#         scene2_static_obstacles['static_obs_1'] = {'shape': 'polygon', 'vertices': [
#             [1.2, 0.4], [1.2, 0.5], [0.75, 0.5], [0.75, 0.4]]}
#         scene2['dynamic_obstacles'] = scene2_dynamic_obstacles
#         scene2['static_obstacles'] = scene2_static_obstacles
#         scene2['robot'] = scene2_robot
#         scene2['watchers'] = {'watcher_1': {
#             'pos': [1, 1], 'range': 4}, 'watcher_2': {'pos': [1, 1], 'range': 4}}
#     json_data['scenerios'] = [scene1, scene2]
#     json.dump(json_data, dst_json_path_.open('w'), indent=4)
#
#
# def generate_random_obstacle(is_dynamic=False,
#                              linear_velocity=0.3,
#                              angular_velocity_max=math.pi / 4,
#                              num_vertices=3,
#                              min_obstacle_radius=0.5,
#                              max_obstacle_radius=1.5):
#     """
#     Taken and adapted from task_generator.obstacle_manager.py
#     """
#     # define body
#     body = {"name": "random", "pose": [0, 0, 0]}
#     if is_dynamic:
#         body["type"] = "dynamic"
#     else:
#         body["type"] = "static"
#     body["color"] = [1, 0.2, 0.1, 1.0]  # [0.2, 0.8, 0.2, 0.75]
#     body["footprints"] = []
#
#     # define footprint
#     f = {}
#     f["density"] = 1
#     f['restitution'] = 1
#     f["layers"] = ["all"]
#     f["collision"] = 'true'
#     f["sensor"] = "false"
#     # dynamic obstacles have the shape of circle
#     if is_dynamic:
#         f["type"] = "circle"
#         f["radius"] = random.uniform(
#             min_obstacle_radius, max_obstacle_radius)
#     else:
#         f["type"] = "polygon"
#         f["points"] = []
#         # random_num_vert = random.randint(
#         #     min_obstacle_vert, max_obstacle_vert)
#         radius = random.uniform(
#             min_obstacle_radius, max_obstacle_radius)
#         # When we send the request to ask flatland server to respawn the object with polygon, it will do some checks
#         # one important assert is that the minimum distance should be above this value
#         # https://github.com/erincatto/box2d/blob/75496a0a1649f8ee6d2de6a6ab82ee2b2a909f42/include/box2d/b2_common.h#L65
#         POINTS_MIN_DIST = 0.005 * 1.1
#
#         def min_dist_check_passed(points):
#             points_1_x_2 = points[None, ...]
#             points_x_1_2 = points[:, None, :]
#             points_dist = ((points_1_x_2 - points_x_1_2)
#                            ** 2).sum(axis=2).squeeze()
#             np.fill_diagonal(points_dist, 1)
#             min_dist = points_dist.min()
#             return min_dist > POINTS_MIN_DIST
#
#         points = None
#         while points is None:
#             angles = 2 * np.pi * np.random.random(num_vertices)
#             points = np.array([np.cos(angles), np.sin(angles)]).T
#             if not min_dist_check_passed(points):
#                 points = None
#         f['points'] = points.tolist()
#
#     body["footprints"].append(f)
#     # define dict_file
#     dict_file = {'bodies': [body], "plugins": []}
#     if is_dynamic:
#         # We added new plugin called RandomMove in the flatland repo
#         random_move = {}
#         random_move['type'] = 'RandomMove'
#         random_move['name'] = 'RandomMove Plugin'
#         random_move['linear_velocity'] = linear_velocity
#         random_move['angular_velocity_max'] = angular_velocity_max
#         random_move['body'] = 'random'
#         dict_file['plugins'].append(random_move)
#
#     return dict_file
