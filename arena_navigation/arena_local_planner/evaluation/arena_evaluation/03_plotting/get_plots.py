from email.policy import default
import numpy as np
import glob # usefull for listing all files of a type in a directory
import os
import time
import yaml
import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import image
import scipy.ndimage as ndimage
import seaborn as sns
import pandas as pd
import argparse

def parsing():
    parser = argparse.ArgumentParser(description='Create quantitative and qualitative plots for user.') # create parser object
    parser.add_argument('--config', type=str, action='store', help='config yaml', default='get_plots_config.yaml') # store userID
    args = parser.parse_args()
    return args

class plotter():
    def __init__(self):
        self.args = parsing()
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.data_dir = os.path.dirname(self.dir_path) + "/02_evaluation" # parent_directory_path + directory name where csv files are located
        self.now = time.strftime("%y-%m-%d_%H:%M:%S")
        try:
            os.mkdir(self.dir_path + "/data")
        except:
            pass
        self.read_config()
        self.grab_data()
        self.load_data()
        self.plot_dir = self.dir_path + "/plots_{}".format(self.now)
        os.mkdir(self.plot_dir)

### load data ###
    def read_config(self):
        with open(self.dir_path+"/"+self.args.config) as file:
            self.config = yaml.safe_load(file)

    def grab_data(self): # move data.json from 02_evaluation into 03_plotting/data
        files = glob.glob("{0}/*.json".format(self.data_dir))
        try:
            self.most_recent_grab = sorted(files)[-1].split("/")[-1] # get last in order JSON data file from /02_evaluation
        except:
            self.most_recent_grab = None
        for file in files:
            file_name = file.split("/")[-1]
            os.rename(self.data_dir+"/"+file_name, self.dir_path+"/data/"+file_name) # move file from dir_path to data folder

    def load_data(self):
        if self.most_recent_grab != None: # if there were no data files found in 02_evaluation get most recent file from config.yaml
            self.config["most_recent_file"] = self.most_recent_grab
        if self.config["specify_data"] == True:
            self.data_file_name = self.config["specified_data_filename"]
            if self.data_file_name == None: # FALLBACK: if no data file specified
                self.data_file_name = self.config["most_recent_file"]
                print("INFO: No data file was specified. Using most recent data file from data directory by default.")
        else:
            self.data_file_name = self.config["most_recent_file"]
            print("INFO: No data file was specified. Using most recent data file from data directory by default.")
        self.data_file_name = self.data_file_name.split("/")[-1]
        try:
            with open(self.dir_path+'/data/{}'.format(self.data_file_name)) as json_file:
                self.data = json.load(json_file)
            print("INFO: Dataset {} loaded.".format(self.data_file_name))
        except:
            self.data_file_name = self.config["most_recent_file"] # FALLBACK: if specified data file is invalid
            with open(self.dir_path+'/data/{}'.format(self.data_file_name)) as json_file:
                self.data = json.load(json_file)
            print("INFO: Specified data file could not be found. Using most recent data file from data directory by default.")
            print("INFO: Dataset {} loaded.".format(self.data_file_name))
        self.config["most_recent_file"] = self.data_file_name
        with open(self.dir_path+"/get_plots_config.yaml", 'w') as file: # update most_recent_file in config
            yaml.dump(self.config, file)
            self.keys = list(self.data.keys())

        ### get scenario properties ###
        self.get_map() # self.maps
        self.get_obstacle_number() # self.obstacle_numbers
        self.get_velocity() # self.velocities
        self.get_planner() # self.planners

    def get_map(self): # get map from df file name
        self.map_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))) + "/simulator_setup/maps"
        self.maps_dict = {x.split("/")[-2]:x for x in sorted(glob.glob("{0}/*/".format(self.map_dir)))}
        for key in self.keys:
            for map in sorted(self.maps_dict.keys()): # check if a map in /simulator_setup/maps fits scenario name
                if map in key:
                    self.data[key]["map"] = map
        self.maps = np.unique([self.data[key]["map"] for key in self.keys])

    def get_obstacle_number(self):
        obstacle_numbers = ["obs0"+str(x) for x in range(10)] + ["obs"+str(x)for x in range(10,100)]
        obstacle_number_found = False
        for key in self.keys:
            for obstacle_number in obstacle_numbers: # check if a map in /simulator_setup/maps fits scenario name
                if obstacle_number in key:
                    self.data[key]["obstacle_number"] = obstacle_number
                    obstacle_number_found = True
        if obstacle_number_found == False:
            for key in self.keys:
                self.data[key]["obstacle_number"] = "base_obstacle_number"
        self.obstacle_numbers = np.unique([self.data[key]["obstacle_number"] for key in self.keys])

    def get_velocity(self):
        velocities = ["vel0"+str(x) for x in range(10)] + ["vel"+str(x)for x in range(10,100)]
        velocity_found = False
        for key in self.keys:
            for velocity in velocities: # check if a map in /simulator_setup/maps fits scenario name
                if velocity in key:
                    self.data[key]["velocity"] = velocity
                    velocity_found = True
        if velocity_found == False:
            for key in self.keys:
                self.data[key]["velocity"] = "base_velocity"
        self.velocities = np.unique([self.data[key]["velocity"] for key in self.keys])

    def get_planner(self):
        planners = list(self.config["labels"].keys())
        planner_found = False
        for key in self.keys:
            for planner in planners:
                if planner in key:
                    self.data[key]["planner"] = planner
                    planner_found = True
        if planner_found == False:
            for key in self.keys:
                self.data[key]["planner"] = "base_planner"
        self.planners = np.unique([self.data[key]["planner"] for key in self.keys])
### end of block load data ###

### qualitative plots ###
    def get_qualitative_plots(self):
        os.mkdir(self.plot_dir + "/qualitative_plots")
        ### iteration part ###
        for map in self.maps:
            map_keys = [] # list of keys with current map
            map_path = self.maps_dict[map]
            with open(map_path+"/map.yaml") as file: # get map file from map.yaml in map directory
                map_yaml = yaml.safe_load(file)
                map_file = map_yaml["image"]
                map_resolution = map_yaml["resolution"]
                map_origin = map_yaml["origin"]
            for key in self.keys:
                if self.data[key]["map"] == map:
                    map_keys.append(key) # append key if map matches current map
            for velocity in self.velocities:
                vel_keys = [] # list of keys with current velocity
                for key in map_keys:
                    if self.data[key]["velocity"] == velocity:
                        vel_keys.append(key) # append key if velocity matches current velocity
                for obstacle_number in self.obstacle_numbers:
                    obs_keys = [] # list of keys with the current obstacle number
                    for key in vel_keys:
                        if self.data[key]["obstacle_number"] == obstacle_number:
                            obs_keys.append(key) # append key if obstacle_number matches current obstacle_number

                    if len(obs_keys) == 0:
                        continue

                    ### plotting part ###
                    fig, ax = plt.subplots()

                    # plot map image
                    img= image.imread("{0}/{1}".format(map_path,map_file))
                    img_rotated = ndimage.rotate(img, 90, reshape=True) # rotate by 90 degree to get rviz konvention
                    plt.imshow(img_rotated, cmap="gray")

                    # plot each planners path and if flag given collisions and zones
                    for key in sorted(obs_keys):
                        # plot paths for every episode
                        planner = self.data[key]["planner"]
                        if planner in self.config["leave_out_planner"]:
                            continue
                        paths = self.data[key]["paths_travelled"]
                        episodes = paths.keys()
                        for i,episode in enumerate(episodes):
                            if i == 0:
                                plt.plot([],[], # plot legend only with empty lines
                                "-",
                                label = self.config["labels"][planner],
                                color = self.config["color_scheme"][planner],
                                linewidth = 2)

                            x,y = to_ros_coords(paths[episode], img, map_resolution, map_origin)
                            x = x[1:-1] # NOTE: sometimes episode is wrongly assigned _> skip first and last coordinates
                            y = y[1:-1]
                            plt.plot(x,y,
                            "-",
                            color = self.config["color_scheme"][planner],
                            alpha = self.config["path_alpha"],
                            linewidth = self.config["path_size"]/map_resolution,zorder=1)

                            if self.config["plot_progression"]:
                                x_progression = x[0::self.config["progression_steps"]]
                                y_progression = y[0::self.config["progression_steps"]]
                                plt.scatter(x_progression,y_progression,
                                color = self.config["color_scheme"][planner],
                                alpha = self.config["path_alpha"],
                                s = self.config["progression_size"]/map_resolution,zorder=1)

                        # plot collisions
                        if self.config["plot_collisions"]:
                            collisions = self.data[key]["collision_zones"]["collisions"]
                            if len(collisions) != 0:
                                x,y = to_ros_coords(collisions, img, map_resolution, map_origin)
                                plt.scatter(x,y,
                                    color = self.config["color_scheme"][planner],
                                    alpha = self.config["collision_alpha"],
                                    s = self.config["collision_size"]/map_resolution,zorder=2)
                        # plot collision zones and centroids
                        if self.config["plot_collision_zones"]:
                            centroids = self.data[key]["collision_zones"]["centroids"]
                            if len(centroids) != 0:
                                counts = self.data[key]["collision_zones"]["counts"]
                                x,y = to_ros_coords(centroids, img, map_resolution, map_origin)
                                plt.scatter(x,y,
                                    color = self.config["color_scheme"][planner],
                                    alpha = self.config["collision_alpha"],
                                    s = self.config["collision_size"]/map_resolution,zorder=2)
                                for i,centroid in enumerate(centroids):
                                    # plot circle for collision zone
                                    ax.add_patch(plt.Circle(tuple(to_ros_coords(centroid, img, map_resolution, map_origin)),
                                    radius = self.config["collision_zone_base_diameter"]*counts[i]/map_resolution,
                                    color=self.config["color_scheme"][planner],
                                    fill=False,zorder=2))
                                    # plot transparent circle as background of zone
                                    ax.add_patch(plt.Circle(tuple(to_ros_coords(centroid, img, map_resolution, map_origin)),
                                    radius = self.config["collision_zone_base_diameter"]*counts[i]/map_resolution,
                                    color=self.config["color_scheme"][planner],
                                    fill=True,
                                    alpha = self.config["collision_zone_alpha"],zorder=2))

                    # plot scenario properties (start, goal, dynamic obstacles)
                    self.plot_scenario(obs_keys, img,  map_resolution, map_origin)

                    # plot legend, title, axes labels
                    if self.config["plot_qualitative_legend"]:
                        plt.legend(loc=self.config["plot_qualitative_legend_location"], fontsize=self.config["plot_qualitative_legend_fontsize"])
                    else:
                        plt.legend(loc=self.config["plot_qualitative_legend_location"], fontsize=self.config["plot_qualitative_legend_fontsize"])
                        ax.get_legend().remove()
                    if self.config["plot_qualitative_title"]:
                        if obstacle_number == "base_obstacle_number" and velocity == "base_velocity":
                            plt.title("Map: {0}".format(map), fontsize = self.config["plot_qualitative_title_size"], fontweight = "bold")
                        elif obstacle_number == "base_obstacle_number":
                            plt.title("Map: {0} Velocity: {1}.{2}".format(map, velocity.replace("vel","")[0], velocity.replace("vel","")[1]), fontsize = self.config["plot_qualitative_title_size"], fontweight = "bold")
                        elif velocity == "base_velocity":
                            plt.title("Map: {0}\n Obstacles: {1}".format(map, int(obstacle_number.replace("obs",""))), fontsize = self.config["plot_qualitative_title_size"], fontweight = "bold")
                        else:
                            plt.title("Map: {0}\n Obstacles: {1} Velocity: {1}.{2} ".format(map, int(obstacle_number.replace("obs","")), velocity.replace("vel","")[0], velocity.replace("vel","")[1]), fontsize = self.config["plot_qualitative_title_size"], fontweight = "bold")
                    if self.config["plot_qualitative_axes"]:
                        plt.xlabel("x in [m]", fontsize=self.config["plot_qualitative_axes_size"])
                        plt.ylabel("y in [m]", fontsize=self.config["plot_qualitative_axes_size"])
                        x_locs = ax.get_xticks()[1:-1]
                        y_locs = ax.get_yticks()[1:-1]
                        ax.set_xticks(x_locs)
                        ax.set_xticklabels([int(x*map_resolution) for x in x_locs])
                        ax.set_yticks(y_locs)
                        ax.set_yticklabels([int(y*map_resolution) for y in y_locs])
                        ax.tick_params(axis='both', which='major', labelsize=self.config["plot_qualitative_axes_ticks_size"])
                    else:
                        x_locs = ax.get_xticks()[1:-1]
                        y_locs = ax.get_yticks()[1:-1]
                        ax.set_xticks(x_locs)
                        ax.set_yticks(y_locs)
                        ax.set_xticklabels([])
                        ax.set_yticks(y_locs)
                        ax.set_yticklabels([])
                    plt.savefig(self.plot_dir + "/qualitative_plots/qualitative_plot_{0}_{1}_{2}_{3}".format(map,obstacle_number,velocity,self.now), bbox_inches='tight')
                    plt.close()

    def plot_scenario(self, keys, img,  map_resolution, map_origin):
        scenario_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))) + "/simulator_setup/scenarios"
        scenario_dict = {x.split("/")[-1].replace(".json",""):x for x in sorted(glob.glob("{0}/*.json".format(scenario_dir)))}
        for scene in sorted(scenario_dict.keys()): # check if a map in /simulator_setup/maps fits scenario name
            if scene in keys[0]:
                scenario_path = scenario_dict[scene]
        with open(scenario_path) as file:
            scenario = json.load(file)
        if "pedsim_agents" in scenario.keys():
            dynamic_obstacles = scenario["pedsim_agents"]
            obstacle_paths = [[obstacle["pos"]]+ obstacle["waypoints"] for obstacle in dynamic_obstacles]
            start_x, start_y = to_ros_coords(scenario["robot_position"], img, map_resolution, map_origin)
            goal_x, goal_y = to_ros_coords(scenario["robot_goal"], img, map_resolution, map_origin)
            obstacle_paths = transform_waypoints(obstacle_paths, img, map_resolution, map_origin, ped_sim=True)
        else:
            start = scenario["scenarios"][0]["robot"]["start_pos"]
            goal = scenario["scenarios"][0]["robot"]["goal_pos"]
            dynamic_obstacles = scenario["scenarios"][0]["dynamic_obstacles"]
            obstacle_paths = [[dynamic_obstacles[obstacle]["start_pos"][:-1]]+[waypoint[:-1] for waypoint in dynamic_obstacles[obstacle]["waypoints"]] for obstacle in dynamic_obstacles]
            start_x, start_y = to_ros_coords(start, img, map_resolution, map_origin)
            goal_x, goal_y = to_ros_coords(goal, img, map_resolution, map_origin)
            obstacle_paths = transform_waypoints(obstacle_paths, img, map_resolution, map_origin)

        # plot start point and goal
        # labels for legend only
        plt.scatter([],[], marker = self.config["start_marker"], label = "Start", color = self.config["start_point_color"])
        plt.scatter([],[], marker = self.config["goal_marker"], label = "Goal", color = self.config["goal_point_color"])
        # start and goal point
        plt.scatter(start_x,start_y, marker = self.config["start_marker"], s = self.config["start_size"]/map_resolution, color = self.config["start_point_color"],zorder=5)
        plt.scatter(goal_x,goal_y, marker = self.config["goal_marker"], s = self.config["goal_size"]/map_resolution, color = self.config["goal_point_color"],zorder=5)

        # plot dynamic obstacle path
        for path in obstacle_paths:
            # plot waypoints as circles
            for i,waypoint in enumerate(path):
                plt.gca().add_patch(plt.Circle(waypoint,
                    radius = self.config["obstacle_radius"]/map_resolution,
                    color=self.config["obstacle_color"],
                    fill=False,zorder=5))
                if i == len(path)-1:
                    continue
                plt.gca().add_patch(patches.FancyArrowPatch(waypoint, path[i+1], arrowstyle='<->', mutation_scale = self.config["path_arrow_size"], color = self.config["path_arrow_color"],zorder=5))
### end of block qualitative plots ###

### quantitative plots ###
    def get_quantitative_plots(self):
        os.mkdir(self.plot_dir + "/quantitative_plots")
        metrics = list(self.data[self.keys[0]]["summary_df"].keys())
        ### iteration part ###
        for map in self.maps:
            map_keys = [] # list of keys with current map
            for key in self.keys:
                if self.data[key]["map"] == map:
                    map_keys.append(key) # append key if map matches current map
            for velocity in self.velocities:
                vel_keys = [] # list of keys with current velocity
                for key in map_keys:
                    if self.data[key]["velocity"] == velocity:
                        vel_keys.append(key) # append key if velocity matches current velocity
                for obstacle_number in self.obstacle_numbers:
                    obs_keys = [] # list of keys with the current obstacle number
                    data = pd.DataFrame() # concat all summary_df of the planners into one and save planner in column
                    for key in sorted(vel_keys):
                        if self.data[key]["obstacle_number"] == obstacle_number:
                            obs_keys.append(key) # append key if obstacle_number matches current obstacle_number
                            dat = pd.DataFrame(self.data[key]["summary_df"]) # concat the summary_df of that key (planner)
                            dat["planner"] = self.data[key]["planner"]
                            data = pd.concat([data,dat])

                    if len(obs_keys) == 0:
                        continue

                    # plotting part
                    for metric in metrics:
                        if metric in self.config["leave_out_metric"]:
                            continue
                        if metric in ["done_reason", "curvature"]:
                            continue
                        fig, ax = plt.subplots()
                        if metric == "success": # bar plots for success metric
                            planner_list = []
                            success_list = []
                            collision_list = []
                            timeout_list = [1.0]*len(obs_keys)
                            for obs_key in sorted(obs_keys):
                                len_df = len(self.data[obs_key]["summary_df"]["done_reason"])
                                planner_list.append(self.data[obs_key]["planner"])
                                success_list.append(self.data[obs_key]["summary_df"]["done_reason"].count("goal_reached")/len_df)
                                collision_list.append((self.data[obs_key]["summary_df"]["done_reason"].count("goal_reached")+self.data[obs_key]["summary_df"]["done_reason"].count("collision"))/len_df)
                            planner_list = [self.config["labels"][x] for x in planner_list]
                            data_success = pd.DataFrame(list(zip(planner_list,timeout_list,success_list,collision_list)), columns=["Planners","Timeout","Success","Collision"])
                            ax = sns.barplot(x = "Planners", y = "Timeout", color = self.config["plot_success_time_out_color"], label = "Timeout", saturation = self.config["plot_success_alpha"], data=data_success)
                            ax = sns.barplot(x = "Planners", y = "Collision", color = self.config["plot_success_collision_color"], label = "Collision", saturation = self.config["plot_success_alpha"], data=data_success)
                            ax = sns.barplot(x = "Planners", y = "Success", color = self.config["plot_success_success_color"], label = "Success", saturation = self.config["plot_success_alpha"], data=data_success)
                            if self.config["plot_quantitative_legend"]:
                                ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])
                            else:
                                ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])
                                ax.get_legend().remove()
                            ax.set_xlabel(self.config["plot_quantitative_labels"]["planner"], fontsize = self.config["plot_quantitative_axes_label_size"])
                            ax.set_ylabel(self.config["plot_quantitative_labels"][metric], fontsize = self.config["plot_quantitative_axes_label_size"])
                            ax.tick_params(axis='both', which='major', labelsize=self.config["plot_quantitative_axes_tick_size"])
                        else:
                            if self.config["plot_quantitative_violin"]:
                                ax.zorder = 5
                                ax = sns.violinplot(x="planner", y=metric, data = data, inner = self.config["plot_quantitative_violin_inner"], palette = self.config["color_scheme"])
                                ax.set_xlabel(self.config["plot_quantitative_labels"]["planner"], fontsize = self.config["plot_quantitative_axes_label_size"])
                                ax.set_ylabel(self.config["plot_quantitative_labels"][metric], fontsize = self.config["plot_quantitative_axes_label_size"])
                                ax.tick_params(axis='both', which='major', labelsize=self.config["plot_quantitative_axes_tick_size"])
                            else:
                                if not self.config["plot_barplot_errorbars"]:
                                    ci = None
                                else:
                                    ci = "sd"
                                ax.zorder = 5
                                ax = sns.barplot(x="planner", y=metric, palette = self.config["color_scheme"], data=data, ci=ci, saturation=self.config["plot_barplot_alpha"],capsize=self.config["plot_barplot_capsize"],errcolor=self.config["plot_barplot_errorcolor"])
                                ax.set_xlabel(self.config["plot_quantitative_labels"]["planner"], fontsize = self.config["plot_quantitative_axes_label_size"])
                                ax.set_ylabel(self.config["plot_quantitative_labels"][metric], fontsize = self.config["plot_quantitative_axes_label_size"])
                                ax.set_xticklabels([self.config["labels"][x.get_text()] for x in ax.get_xticklabels()], fontsize = self.config["plot_quantitative_axes_tick_size"])                            
                                ax.tick_params(axis='both', which='major', labelsize=self.config["plot_quantitative_axes_tick_size"])
                        # title 
                        if self.config["plot_quantitative_suptitle"]:
                            plt.suptitle("{0}".format(self.config["plot_quantitative_labels"][metric]), fontsize = self.config["plot_quantitative_suptitle_size"], fontweight = "bold")
                        # subtitle
                        if self.config["plot_quantitative_title"]:
                            if obstacle_number == "base_obstacle_number" and velocity == "base_velocity":
                                plt.title("Map: {0}".format(map), fontsize = self.config["plot_quantitative_title_size"])
                            elif obstacle_number == "base_obstacle_number":
                                plt.title("Map: {0} Velocity: {1}.{2}".format(map, velocity.replace("vel","")[0], velocity.replace("vel","")[1]), fontsize = self.config["plot_quantitative_title_size"])
                            elif velocity == "base_velocity":
                                plt.title("Map: {0} Obstacles: {1}".format(map, int(obstacle_number.replace("obs",""))), fontsize = self.config["plot_quantitative_title_size"])
                            else:
                                plt.title("Map: {0} Obstacles: {1} Velocity: {1}.{2} ".format(map, int(obstacle_number.replace("obs","")), velocity.replace("vel","")[0], velocity.replace("vel","")[1]), fontsize = self.config["plot_quantitative_title_size"])

                        # grid
                        if self.config["plot_quantitative_ygrid"]:
                            sns.set_style("whitegrid")

                        plt.savefig(self.plot_dir + "/quantitative_plots/{0}_{1}_{2}_{3}".format(metric,map,obstacle_number,velocity), bbox_inches='tight')
                        plt.close()
### end of block quantitative plots ###

### obs in one plots ###
    def get_obs_in_one_plots(self):
        os.mkdir(self.plot_dir + "/obs_in_one_plots")
        metrics = list(self.data[self.keys[0]]["summary_df"].keys())
        ### iteration part ###
        for map in self.maps:
            map_keys = [] # list of keys with current map
            for key in self.keys:
                if self.data[key]["map"] == map:
                    map_keys.append(key) # append key if map matches current map
            for velocity in self.velocities:
                vel_keys = [] # list of keys with current velocity
                for key in map_keys:
                    if self.data[key]["velocity"] == velocity:
                        vel_keys.append(key) # append key if velocity matches current velocity

                if len(vel_keys) == 0:
                    continue

                # plotting part
                obs_in_one_color_scheme = {self.config["labels"][k]:v for k,v in self.config["color_scheme"].items()}               
                for metric in metrics:
                    if metric in self.config["leave_out_metric"]:
                        continue
                    if metric in ["done_reason", "curvature"]:
                        continue
                    fig, ax = plt.subplots()
                    data = pd.DataFrame() # concat all summary_df of the planners into one and save planner in column
                    for obstacle_number in self.obstacle_numbers:
                        for key in sorted(vel_keys):
                            if self.data[key]["obstacle_number"] == obstacle_number:
                                dat = pd.DataFrame(self.data[key]["summary_df"]) # concat the summary_df of that key (planner)
                                dat["Planner"] = self.config["labels"][self.data[key]["planner"]]
                                dat["obstacle_number"] = self.data[key]["obstacle_number"].replace("obs0","").replace("obs","")
                                len_dat = len(dat.index)
                                dat["Success"] = [list(dat["done_reason"]).count("goal_reached")/len_dat]*len_dat
                                data = pd.concat([data,dat])
                    if metric == "success": # bar plots for success metric
                        ax.zorder = 5
                        ax = sns.barplot(x="obstacle_number", y="Success", hue="Planner", palette = obs_in_one_color_scheme, data=data, ci=None, alpha=self.config["plot_obs_in_one_alpha"])
                        ax.set_xlabel(self.config["plot_quantitative_labels"]["obstacle_number"], fontsize = self.config["plot_quantitative_axes_label_size"])
                        ax.set_ylabel(self.config["plot_quantitative_labels"][metric], fontsize = self.config["plot_quantitative_axes_label_size"])
                        ax.tick_params(axis='both', which='major', labelsize=self.config["plot_quantitative_axes_tick_size"])
                        if self.config["plot_quantitative_legend"]:
                            ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])
                        else:
                            ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])
                            ax.get_legend().remove()
                    else:
                        if self.config["plot_quantitative_violin"]:
                            ax.zorder = 5
                            ax = sns.violinplot(x="obstacle_number", y=metric, hue="Planner", palette = obs_in_one_color_scheme, data=data, inner = self.config["plot_quantitative_violin_inner"])
                            ax.set_xlabel(self.config["plot_quantitative_labels"]["obstacle_number"], fontsize = self.config["plot_quantitative_axes_label_size"])
                            ax.set_ylabel(self.config["plot_quantitative_labels"][metric], fontsize = self.config["plot_quantitative_axes_label_size"])
                            ax.tick_params(axis='both', which='major', labelsize=self.config["plot_quantitative_axes_tick_size"])
                            if self.config["plot_quantitative_legend"]:
                                ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])    
                            else:
                                ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])
                                ax.get_legend().remove()                          
                        else:
                            if not self.config["plot_obs_in_one_errorbars"]:
                                ci = None
                            else:
                                ci = "sd"
                            ax.zorder = 5
                            ax = sns.barplot(x="obstacle_number", y=metric, hue="Planner", palette = obs_in_one_color_scheme, data=data, ci=ci, saturation=self.config["plot_obs_in_one_alpha"],capsize=self.config["plot_obs_in_one_capsize"],errcolor=self.config["plot_obs_in_one_errorcolor"])
                            ax.set_xlabel(self.config["plot_quantitative_labels"]["obstacle_number"], fontsize = self.config["plot_quantitative_axes_label_size"])
                            ax.set_ylabel(self.config["plot_quantitative_labels"][metric], fontsize = self.config["plot_quantitative_axes_label_size"])
                            ax.tick_params(axis='both', which='major', labelsize=self.config["plot_quantitative_axes_tick_size"])
                            if self.config["plot_quantitative_legend"]:
                                ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])
                            else:
                                ax.legend(loc=self.config["plot_quantitative_legend_location"], fontsize=self.config["plot_quantitative_legend_fontsize"])
                                ax.get_legend().remove()                        

                    # title 
                    if self.config["plot_quantitative_suptitle"]:
                        plt.suptitle("{0}".format(self.config["plot_quantitative_labels"][metric]), fontsize = self.config["plot_quantitative_suptitle_size"], fontweight = "bold")
                    # subtitle
                    if self.config["plot_quantitative_title"]:
                        if velocity == "base_velocity" and obstacle_number == "base_obstacle_number":
                            plt.title("Map: {0}".format(map), fontsize = self.config["plot_quantitative_title_size"])
                        elif obstacle_number == "base_obstacle_number":
                            plt.title("Map: {0} Velocity: {1}.{2}".format(map, velocity.replace("vel","")[0], velocity.replace("vel","")[1]), fontsize = self.config["plot_quantitative_title_size"])
                        else:
                            plt.title("Map: {0}".format(map), fontsize = self.config["plot_quantitative_title_size"])
                    # grid
                    if self.config["plot_quantitative_ygrid"]:
                        sns.set_style("whitegrid")

                    plt.savefig(self.plot_dir + "/obs_in_one_plots/{0}_{1}_obs_in_one_{2}".format(metric,map,velocity), bbox_inches='tight')
                    plt.close()
### end of block obs in one plots ###

### latex table ###
    def get_latex_table(self):
        ### iteration part ###
        data = pd.DataFrame() # concat all summary_df of the planners into one and save planner in column
        for map in self.maps:
            map_keys = [] # list of keys with current map
            for key in self.keys:
                if self.data[key]["map"] == map:
                    map_keys.append(key) # append key if map matches current map
            for velocity in self.velocities:
                vel_keys = [] # list of keys with current velocity
                for key in map_keys:
                    if self.data[key]["velocity"] == velocity:
                        vel_keys.append(key) # append key if velocity matches current velocity
                for obstacle_number in self.obstacle_numbers:
                    obs_keys = [] # list of keys with the current obstacle number
                    for key in sorted(vel_keys):
                        if self.data[key]["obstacle_number"] == obstacle_number:
                            obs_keys.append(key) # append key if obstacle_number matches current obstacle_number
                            dat = pd.DataFrame(self.data[key]["summary_df"]) # concat the summary_df of that key (planner)
                            len_dat = len(dat.index)
                            dat["success"] = [list(dat["done_reason"]).count("goal_reached")/len_dat]*len_dat
                            dat["planner"] = self.config["labels"][self.data[key]["planner"]]
                            dat["obs"] = self.data[key]["obstacle_number"].replace("obs","")
                            dat["map"] = map
                            data = pd.concat([data,dat[["time","path_length","success","collision","jerk","map","planner","obs"]]])
        data = data.groupby(["map","planner","obs"]).mean()
        data_map_average = data.groupby(level=["planner","obs"]).mean()
        data_map_average_obs_overage = data.groupby(level=["planner"]).mean()
        data_obs_average = data.groupby(level=["map","planner"]).mean()
        maps_list = self.maps
        planner_list = [self.config["labels"][x] for x in self.planners]
        obs_list = ["05","10","20","avg"]
        for map in maps_list:
            for planner in planner_list:
                data.loc[map,planner,"avg"] = data_obs_average.loc[map,planner]
        for planner in planner_list:
            data_map_average.loc[(planner,"avg"),:] = data_map_average_obs_overage.loc[planner]
        data = data.sort_index()
        data = data.rename({"time":"Time","path_length":"Path Length","success":"Success Rate","collision":"Collisions","jerk":"Movement Jerk"}, axis=1)
        data_map_average = data_map_average.sort_index()
        data_map_average = data_map_average.rename({"time":"Time","path_length":"Path Length","success":"Success Rate","collision":"Collisions","jerk":"Movement Jerk"}, axis=1)
        cols = ["Time","Path Length","Success Rate","Collisions","Movement Jerk"]
        data_minimal = pd.DataFrame(index=data_obs_average.index, columns=cols,)
        for map in maps_list:
            for planner in planner_list:
                for col in cols:
                    col_data = str()
                    for obs in obs_list:
                        if obs == "05":
                            col_data = str(np.round(data.loc[(map,planner,obs),col],2))
                        else:
                            col_data = col_data + "|" + str(np.round(data.loc[(map,planner,obs),col],2))
                    data_minimal.loc[(map,planner),col] = col_data
        data_map_average_minimal = pd.DataFrame(index=data_map_average_obs_overage.index, columns=cols,)
        for planner in planner_list:
            for col in cols:
                col_data = str()
                for obs in obs_list:
                    if obs == "05":
                        col_data = str(np.round(data_map_average.loc[(planner,obs),col],2))
                    else:
                        col_data = col_data + " | " + str(np.round(data_map_average.loc[(planner,obs),col],2))
                data_map_average_minimal.loc[(planner),col] = col_data

        table = {"map_data":data_minimal,"avg_data":data_map_average_minimal}
        text_file = open(self.plot_dir+"/latex_table.txt", "w")
        for t in table:
            text_file.write("\nTable for: "+t+"\n")
            text_file.write(table[t].to_latex())
        text_file.close()
### end of block latex table ###

# additional functions
# x,y coordinate transformations needed for qualitative plots
def to_ros_coords(coords, img, map_resolution, map_origin):
    if type(coords) == tuple:
        return  [img.shape[0] - (y-map_origin[1])/map_resolution for x,y in coords],[img.shape[1] - (x-map_origin[0])/map_resolution for x,y in coords]
    elif type(coords) == list and (type(coords[0]) == float or type(coords[0]) == int):
        return  img.shape[0] - (coords[1]-map_origin[1])/map_resolution,img.shape[1] - (coords[0]-map_origin[0])/map_resolution
    elif type(coords) == list:
        return  [img.shape[0] - (point[1]-map_origin[1]) / map_resolution for point in coords],[img.shape[1] - (point[0]-map_origin[0]) / map_resolution for point in coords]

def transform_waypoints(obstacle_paths, img, map_resolution, map_origin, ped_sim = False): # only necessary for old scenario.json
    transformed_paths = []
    if ped_sim:
        for path in obstacle_paths:
            transformed_paths.append([list(to_ros_coords(p,img, map_resolution, map_origin)) for p in path])
    else:
        for path in obstacle_paths:
            start = list(to_ros_coords(path[0],img, map_resolution, map_origin))
            waypoint = list(to_ros_coords((np.array(path[0]) + np.array(path[1])).tolist(),img, map_resolution, map_origin))
            transformed_paths.append([start,waypoint])
    return transformed_paths


if __name__=="__main__":
    Plotter = plotter()
    if Plotter.config["plot_qualitative"]:
        Plotter.get_qualitative_plots()
    if Plotter.config["plot_quantitative"]:
        Plotter.get_quantitative_plots()
    if Plotter.config["plot_obs_in_one"]:
        Plotter.get_obs_in_one_plots()
    if Plotter.config["latex"]:
        Plotter.get_latex_table()