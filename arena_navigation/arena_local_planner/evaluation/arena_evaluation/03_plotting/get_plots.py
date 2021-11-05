import numpy as np
import glob # usefull for listing all files of a type in a directory
import os
import time
import yaml
import json
import matplotlib.pyplot as plt

class plotter():
    def __init__(self):
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.data_dir = os.path.dirname(self.dir_path) + "/02_evaluation" # parent_directory_path + directory name where csv files are located
        self.now = time.strftime("%y-%m-%d_%H:%M:%S")
        self.grab_data()
        self.read_config()
        self.load_data()

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

    def get_qualitative_plots(self):
        return



if __name__=="__main__":
    Plotter = plotter()
    if Plotter.config["plot_qualitative"]:
        Plotter.get_qualitative_plots()