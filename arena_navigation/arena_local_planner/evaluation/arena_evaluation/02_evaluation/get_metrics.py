import numpy as np
import pandas as pd
import glob # usefull for listing all files of a type in a directory
import os
import time
import yaml
import json
import warnings
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
import sys

class get_metrics():
    def __init__(self):
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.data_dir = os.path.dirname(self.dir_path) + "/01_recording/" # parent_directory_path + directory name where csv files are located
        self.now = time.strftime("%y-%m-%d_%H:%M:%S")
        self.read_config()

    def read_config(self):
        with open(self.dir_path+"/get_metrics_config.yaml") as file:
            self.config = yaml.safe_load(file)

    def evaluate_data(self): # read in all csv files and compute metrics
        print("INFO: Start data transformation and evaluation: {}".format(time.strftime("%H:%M:%S")))
        data = {}
        files = glob.glob("{0}/*.csv".format(self.data_dir)) # get all the csv files paths in the directory where this script is located
        planners = []
        if len(files) == 0:
            print("ERROR: No files to evaluate were found the requestID directory. Terminating script.")
            sys.exit()
        for file in files: # summarize all the csv files and add to dictionary
            file_name = file.split("/")[-1].split("--")[:-1] # cut off date and time and .csv ending
            if len(file_name) == 0:
                print("-------------------------------------------------------------------------------------------------")
                print("ERROR: " + file + " does not match the naming convention. Skipping this file")
                files.remove(file)
                continue
            planners.append(file_name[0])
            file_name = "--".join(file_name) # join together to only include local planner, map and obstacle number
            print("-------------------------------------------------------------------------------------------------")
            print("INFO: Beginning data tranformation and evaluation for: {}".format(file_name))
            df = self.extend_df(pd.read_csv(file, converters = {"laser_scan":self.string_to_float_list, "action": self.string_to_float_list}))
            df = self.drop_last_episode(df)
            data[file_name] = {
                "summary_df": self.get_summary_df(df).to_dict(orient = "list"),
                "paths_travelled": self.get_paths_travelled(df),
                "collision_zones": self.get_collision_zones(df)
            }
            print("INFO: Data tranformation and evaluation finished for: {}".format(file_name))
        self.grab_data(files)
        with open(self.dir_path+"/data_{}.json".format(self.now), "w") as outfile:
            json.dump(data, outfile)
        with open(self.dir_path+"/data_{}.txt".format(self.now), "w") as outfile:
            outfile.write("\n".join(list(np.unique(planners))))
        print("-------------------------------------------------------------------------------------------------")
        print("INFO: End data transformation and evaluation: {}".format(time.strftime("%y-%m-%d_%H:%M:%S")))
        return data
 
    def grab_data(self,files): # move data from 01_recording into 02_evaluattion into a data folder with timestamp
        os.mkdir(self.dir_path+"/data_{}".format(self.now))
        for file in files:
            file_name = file.split("/")[-1]
            os.rename(self.data_dir+"/"+file_name, self.dir_path+"/data_{}/".format(self.now)+file_name) # move file from dir_path to data folder
    def string_to_float_list(self,df_column): # convert list from csv saved as string to list of floats
        return list(np.array((df_column.replace("[","").replace("]","").split(", "))).astype(float))

    def extend_df(self,df):
        model = np.unique(df["model"])[0]
        with warnings.catch_warnings():
            warnings.simplefilter('ignore') 
            df["collision"] = self.get_collision(df,model)
            df["action_type"] = self.get_action_type(df)
            # df["computation_time"] = self.get_computation_time(df)
            df["max_clearing_distance"] = [np.nanmax(np.where(np.isfinite(x), x, 0)) for x in df["laser_scan"]]
            df["min_clearing_distance"] = [np.nanmin(x) for x in df["laser_scan"]]
            df["mean_clearing_distance"] = [np.nanmean(np.where(np.isfinite(x), x, 0)) for x in df["laser_scan"]]
            df["median_clearing_distance"] = [np.nanmedian(np.where(np.isfinite(x), x, 0)) for x in df["laser_scan"]]
            df["curvature"],df["normalized_curvature"] = self.get_curvature(df)
            df["roughness"] = self.get_roughness(df)
            df["jerk"] = self.get_jerk(df)
        return df

    def get_collision(self,df,model):
        raw_collision = list(np.any(np.less_equal(x,self.config["robot_radius"][model])) for x in df["laser_scan"])
        helper_collision = [False] + raw_collision[:-1]
        return [x > 0 for x in [r - h for r,h in zip(list(map(int, raw_collision)),list(map(int, helper_collision)))]]

    def get_action_type(self,df):
        action_type_column = []
        for x in df["action"]:
            if x[0] == 0.0 and x[1] == 0.0 and x[2] == 0.0:
                action_type_column.append("stop")
            elif x[0] == 0.0 and x[1] == 0.0 and x[2] != 0.0:
                action_type_column.append("rotate")
            else:
                action_type_column.append("move")
        return action_type_column

    def get_computation_time(self,df):
        computation_time_column = []
        for i,x in enumerate(df["time"]):
            if i == 0:
                computation_time_column.append(np.nan)
            else:
                computation_time_column.append(x-df["time"][i-1])
        return computation_time_column
    
    def get_curvature(self,df):
        curvature_list = []
        normalized_curvature_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            points = [list(x) for x in zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"])]
            for i,point in enumerate(points):
                try:
                    x = np.array(point)
                    y = np.array(points[i+1])
                    z = np.array(points[i+2])
                    curvature_list.append(self.calc_curvature(x,y,z)[0])
                    normalized_curvature_list.append(self.calc_curvature(x,y,z)[1])
                    continue
                except:
                    curvature_list.append(np.nan)
                    normalized_curvature_list.append(np.nan)
                    continue
        return curvature_list, normalized_curvature_list

    def calc_curvature(self,x,y,z): # Menger curvature of 3 points
        triangle_area = 0.5 * np.abs(x[0]*(y[1]-z[1]) + y[0]*(z[1]-x[1]) + z[0]*(x[1]-y[1]))
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')        
            curvature = 4*triangle_area / (np.abs(np.linalg.norm(x-y)) * np.abs(np.linalg.norm(y-z)) * np.abs(np.linalg.norm(z-x)))
            normalized_curvature = curvature* (np.abs(np.linalg.norm(x-y)) + np.abs(np.linalg.norm(y-z)))
        return [curvature, normalized_curvature]

    def get_roughness(self,df):
        roughness_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            points = [list(x) for x in zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"])]
            for i,point in enumerate(points):
                try:
                    x = np.array(point)
                    y = np.array(points[i+1])
                    z = np.array(points[i+2])
                    roughness_list.append(self.calc_roughness(x,y,z))
                    continue
                except:
                    roughness_list.append(np.nan)
                    continue
        return roughness_list

    def calc_roughness(self,x,y,z):
        with warnings.catch_warnings():
            warnings.simplefilter('ignore') 
            triangle_area = 0.5 * np.abs(x[0]*(y[1]-z[1]) + y[0]*(z[1]-x[1]) + z[0]*(x[1]-y[1]))
            roughness = 2 * triangle_area / np.abs(np.linalg.norm(z-x))**2 # basically height / base (relative height)
        return roughness

    def get_jerk(self,df):
        jerk_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            velocities = [list(x) for x in zip(df.loc[df["episode"]==episode,"robot_lin_vel_x"],df.loc[df["episode"]==episode,"robot_lin_vel_y"])]
            for i,vel in enumerate(velocities):
                try:
                    v1 = np.array(vel)
                    v2 = np.array(velocities[i+1])
                    v3 = np.array(velocities[i+2])
                    jerk_list.append(self.calc_jerk(v1,v2,v3))
                    continue
                except:
                    jerk_list.append(np.nan)
                    continue
        return jerk_list

    def calc_jerk(self,v1,v2,v3):
        v1 = (v1[0]**2 + v1[1]**2)**0.5 # total velocity
        v2 = (v2[0]**2 + v2[1]**2)**0.5
        v3 = (v3[0]**2 + v3[1]**2)**0.5            
        a1 = v2-v1 # acceleration
        a2 = v3-v2
        jerk = np.abs(a2-a1)
        return jerk

    def drop_last_episode(self,df):
        episodes = np.unique(df["episode"])
        df = df.drop(df[df["episode"] == episodes[-1]].index)
        return df

    def get_summary_df(self,df): # NOTE: column specification hardcoded !
        sum_df = df.groupby(["episode"]).sum()
        mean_df = df.groupby(["episode"]).mean()
        summary_df = mean_df
        summary_df["time"] = self.get_time(df)
        summary_df["collision"] = sum_df["collision"]
        summary_df["path_length"] = self.get_path_length(df)
        summary_df["success"],summary_df["done_reason"]  = self.get_success(summary_df)
        summary_df["max_curvature"] = self.get_max_curvature(df)
        # summary_df["angle_over_length"] , summary_df["cusps"] = self.get_AOL(df,summary_df) # NOTE: in simulation cusps will never occur, unnessary
        summary_df["angle_over_length"] , _ = self.get_AOL(df,summary_df) # NOTE: in simulation cusps will never occur, unnessary
        summary_df = summary_df.drop(columns = ['robot_lin_vel_x', 'robot_lin_vel_y', 'robot_ang_vel', 'robot_orientation', 'robot_pos_x', 'robot_pos_y'])
        return summary_df
    
    def get_time(self,df):
        time_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            times = list(df.loc[df["episode"]==episode,"time"])
            time_list.append((times[-1]-times[0])*0.3) # sim to real time factor, currently hardcoded TODO: implement realtime factor
        return time_list

    def get_path_length(self,df):
        path_length_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            path_length = 0
            points = list(zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"]))
            for i,point in enumerate(points):
                if i == 0:
                    continue
                else:
                    path_length = path_length + np.linalg.norm(np.array(point)-np.array(points[i-1]))
            path_length_list.append(path_length)
        return path_length_list

    def get_success(self,summary_df):
        success_list = []
        done_reason_list = []
        for episode in summary_df.index:
            if summary_df.loc[episode,"collision"] > self.config["collision_treshold"]:
                success_list.append(False)
                done_reason_list.append("collision")
            elif summary_df.loc[episode,"time"] > self.config["time_out_treshold"]:
                success_list.append(False)
                done_reason_list.append("time_out")
            else:
                success_list.append(True)
                done_reason_list.append("goal_reached")
        return success_list, done_reason_list

    def get_max_curvature(self,df):
        max_curvature_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            max_curvature_list.append(np.max(df.loc[df["episode"]==episode,"curvature"]))
        return max_curvature_list

    def get_AOL(self,df,summary_df):
        AOL_list = []
        cusps_list = []
        episodes = np.unique(df["episode"])
        for episode in episodes:
            path_length = summary_df.loc[episode,"path_length"]
            total_yaw = 0
            cusps = 0
            yaws = list(df.loc[df["episode"]==episode,"robot_orientation"])
            for i,yaw in enumerate(yaws):
                if i == 0:
                    continue
                else:
                    yaw_diff = np.abs(yaw-yaws[i-1])
                    if yaw_diff == np.pi:
                        cusps += 1
                    total_yaw += yaw_diff
            AOL_list.append(total_yaw/path_length)
            cusps_list.append(cusps)
        return AOL_list, cusps_list

    def get_paths_travelled(self,df):
        paths_travelled = {}
        episodes = np.unique(df["episode"])
        for episode in episodes:
            paths_travelled[str(episode)] = list(zip(df.loc[df["episode"]==episode,"robot_pos_x"],df.loc[df["episode"]==episode,"robot_pos_y"]))
        return paths_travelled

    def get_collision_zones(self,df):
        collisions = df.loc[df["collision"]==True,["robot_pos_x","robot_pos_y","collision"]]
        points = [list(x) for x in list(zip(collisions["robot_pos_x"],collisions["robot_pos_y"]))]

        silhouette_score_list = []
        with warnings.catch_warnings():
            warnings.simplefilter('ignore') 
            kmax = len(points)-1
            if len(points) <= 3:
                return {"centroids": [], "counts": [], "collisions": []}
            for k in range(2, kmax+1):
                kmeans = KMeans(n_clusters = k).fit(points)
                labels = kmeans.labels_
                silhouette_score_list.append(silhouette_score(points, labels, metric = 'euclidean'))
            best_k = np.argmax(silhouette_score_list) + 2 # kmeans here starts at 2 centroids so argmax 0 equals 2 centroids
            kmeans = KMeans(n_clusters = best_k).fit(points)
            centroids = kmeans.cluster_centers_
            _ , counts = np.unique(kmeans.labels_, return_counts=True)
        return {"centroids": centroids.tolist(), "counts": counts.tolist(), "collisions": collisions.values.tolist()}

if __name__=="__main__":
    metrics = get_metrics()
    metrics_data = metrics.evaluate_data()