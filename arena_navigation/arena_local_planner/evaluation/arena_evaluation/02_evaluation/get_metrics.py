import numpy as np
import pandas as pd
import glob # usefull for listing all files of a type in a directory
import os
import time
import yaml

class get_metrics():
    def __init__(self) -> None:
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.data_dir = os.path.dirname(self.dir_path) + "/01_recording" # parent_directory_path + directory name where csv files are located
        self.read_config()

    def read_config(self):
        with open(self.dir_path+"/get_metrics_config.yaml") as file:
            self.config = yaml.safe_load(file)

    def grab_data(self):
        files = glob.glob("{0}/*.csv".format(self.data_dir)) # get all the csv files paths in the directory /01_recording
        for file in files:
            file_name = file.split("/")[-1]
            os.rename(self.data_dir+"/"+file_name, self.dir_path+"/"+file_name) # move file from data_dir to dir_path (where this script is located)

    def evaluate_data(self): # read in all csv files and compute metrics
        data = {}
        files = glob.glob("{0}/*.csv".format(self.dir_path)) # get all the csv files paths in the directory where this script is located
        for file in files: # summarize all the csv files and add to dictionary
            df = pd.read_csv(file, converters = {"laser_scan":self.string_to_float_list, "action": self.string_to_float_list})
            file_name = file.split("/")[-1].split("_")[:-2] # cut off date and time and .csv ending
            file_name = "_".join(file_name) # join together to only include local planner, map and obstacle number
            data[file_name] = {
                "df": self.extend_df(df),
                "summary_df": [],
                "paths_travelled": []
            }
            return data

    def string_to_float_list(self,df_column): # convert list from csv saved as string to list of floats
        return np.array((df_column.replace("[","").replace("]","").split(", "))).astype(float)

    def extend_df(self,df):
        df["collision"] = [np.any(np.less_equal(x,self.config["robot_radius"])) for x in df["laser_scan"]]
        df["action_type"] = self.get_action_type(df)
        df["computation_time"] = self.get_computation_time(df)
        df["max_clearing_distance"] = [np.nanmax(x) for x in df["laser_scan"]]
        df["min_clearing_distance"] = [np.nanmin(x) for x in df["laser_scan"]]
        df["mean_clearing_distance"] = [np.nanmean(x) for x in df["laser_scan"]]
        df["median_clearing_distance"] = [np.nanmedian(x) for x in df["laser_scan"]]

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

if __name__=="__main__":
    metrics = get_metrics()
    metrics.grab_data()
    metrics_data = metrics.evaluate_data()