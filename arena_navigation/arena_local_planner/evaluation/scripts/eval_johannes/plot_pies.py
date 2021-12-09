from matplotlib import colors
import matplotlib.pyplot as plt
import numpy as np
import yaml
import pandas as pd
import argparse # get user input from terminal as arguments
import glob # usefull for listing all files of a type in a directory
import os

def parsing(): # define what flags user can/must give as terminal input
    parser = argparse.ArgumentParser(description='Evaluate ods files.') # create parser object
    parser.add_argument('path_to_parent_directory', type=str, action='store',
                        help='path to parent directory of directory with the .ods files')                        
    args = parser.parse_args()
    return args

def import_ods_file(path_to_parent_directory):
    return pd.read_excel(path_to_parent_directory, engine="odf",index_col=0)

def load_config_yaml():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    with open(dir_path+"/eval_config.yaml", "r") as stream:
        config = yaml.safe_load(stream)
        return config["color_mapping"],config["pie_labels"],config["subdirectory_names"]

def filter_data(data): # get only those observations with drl-teb distribution
    filtered_data = {}
    for key in data:
        # if key == list(data.keys())[0]:
        #     filtered_data = data[key].loc[data[key]["distribution_drl_teb"]!=["[1.]"]*len(data[key]["distribution_drl_teb"])]
        # else:
        #     filtered_data = filtered_data.append(data[key].loc[data[key]["distribution_drl_teb"]!=["[1.]"]*len(data[key]["distribution_drl_teb"])])
        filtered_data[key] = (data[key].loc[data[key]["distribution_drl_teb"]!=["[1.]"]*len(data[key]["distribution_drl_teb"])]["distribution_drl_teb"][0]).replace("[","").replace("]","").split(" ")
    return filtered_data

def get_data(path_to_parent_directory,subdirectory_names):
    files = []
    data = {}
    for subdirectory in subdirectory_names:
        files = glob.glob("{0}/{1}/*.ods".format(path_to_parent_directory,subdirectory)) # get all the .ods files paths in the directory provided from user
        for file in files:
            data[file.split(path_to_parent_directory+"/"+subdirectory+"/")[-1].split(".ods")[0]] = import_ods_file(file)
    return filter_data(data)

def plot_pies(data,color_mapping,pie_labels,subdirectory_names):
    nrow = len(subdirectory_names)
    ncol = int(len(data)/len(subdirectory_names))
    fig, axs = plt.subplots(nrow,ncol)
    for i,key in enumerate(sorted(data.keys())):
        axs[i//ncol,i%ncol].pie(data[key], labels = ["DRL","TEB"], startangle = 90, colors = ["tab:green","tab:blue"],autopct='%1.0f%%', pctdistance=0.5, labeldistance=1.2)
        # axs[i//ncol,i%ncol].title.set_text(pie_labels[key])
    plt.legend(loc="lower right")
    fig.suptitle("DRL-TEB distribution across maps and obstacles numbers", fontsize=16)
    plt.tight_layout()
    plt.savefig("pie.png")
    plt.show()

if __name__ == "__main__": # execute code
    args = parsing()
    path_to_parent_directory = args.path_to_parent_directory
    color_mapping, pie_labels, subdirectory_names = load_config_yaml()
    data = get_data(path_to_parent_directory,subdirectory_names)

    plot_pies(data,color_mapping,pie_labels,subdirectory_names)