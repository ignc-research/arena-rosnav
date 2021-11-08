import numpy as np
import glob # usefull for listing all files of a type in a directory
import os
import time
import yaml
import json
import matplotlib.pyplot as plt
from matplotlib import image
import scipy.ndimage as ndimage
import sys

class plotter():
    def __init__(self):
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.data_dir = os.path.dirname(self.dir_path) + "/02_evaluation" # parent_directory_path + directory name where csv files are located
        self.now = time.strftime("%y-%m-%d_%H:%M:%S")
        self.read_config()
        self.grab_data()
        self.load_data()
        # os.mkdir(self.dir_path + "/plots_{}".format(self.now))

    def read_config(self):
        with open(self.dir_path+"/get_plots_config.yaml") as file:
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
        self.keys = self.data.keys()

    def get_qualitative_plots(self):
        ### get scenario properties ###
        self.get_map() # self.maps
        self.get_obstacle_number() # self.obstacle_numbers
        self.get_velocity() # self.velocities
        self.get_planner() # self.planners

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
                    
                    ### plotting part ###
                    plt.figure()

                    # plot image
                    img= image.imread("{0}/{1}".format(map_path,map_file))
                    img_rotated = ndimage.rotate(img, 90, reshape=True) # rotate by 90 degree to get rviz konvention
                    plt.imshow(img_rotated)

                    # plot each planners path and if flag given collisions and zones
                    for key in obs_keys:
                        # plot paths for every episode
                        planner = self.data[key]["planner"]
                        paths = self.data[key]["paths_travelled"]
                        episodes = paths.keys()
                        for i,episode in enumerate(episodes):
                            if i == 0:
                                x,y = to_ros_coords(paths[episode], img, map_resolution, map_origin)
                                plt.scatter(x,y, # coordinates need to transformed to ros format
                                label = self.config["labels"][planner],
                                color = self.config["color_scheme"][planner],
                                alpha = self.config["path_alpha"],
                                s = self.config["path_size"])
                            else:
                                x,y = to_ros_coords(paths[episode], img, map_resolution, map_origin)
                                plt.scatter(x,y, # coordinates need to transformed to ros format
                                color = self.config["color_scheme"][planner],
                                alpha = self.config["path_alpha"],
                                s = self.config["path_size"])
                    plt.show()
                    sys.exit()

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
        planners = list(self.config["color_scheme"].keys())
        planner_found = False
        for key in self.keys:
            for planner in planners: # check if a map in /simulator_setup/maps fits scenario name
                if planner in key:
                    self.data[key]["planner"] = planner
                    planner_found = True
        if planner_found == False:
            for key in self.keys:
                self.data[key]["planner"] = "base_planner"
        self.planners = np.unique([self.data[key]["planner"] for key in self.keys])

def to_ros_coords(coords, img, map_resolution, map_origin):
    # return [[img.shape[1] + (x-map_origin[0])/map_resolution, img.shape[0] - (y-map_origin[1])/map_resolution] for x,y in coords]
    return [x/map_resolution for x,y in coords], [y/map_resolution for x,y in coords]

if __name__=="__main__":
    Plotter = plotter()
    if Plotter.config["plot_qualitative"]:
        Plotter.get_qualitative_plots()