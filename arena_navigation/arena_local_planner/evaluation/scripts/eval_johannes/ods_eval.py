import matplotlib.pyplot as plt
import numpy as np
import yaml
import pandas as pd
import argparse # get user input from terminal as arguments
import glob # usefull for listing all files of a type in a directory
import os
import datetime

def parsing(): # define what flags user can/must give as terminal input
    parser = argparse.ArgumentParser(description='Evaluate ods files.') # create parser object
    parser.add_argument('path_to_ods_files', type=str, action='store',
                        help='path to directories with the .ods files')                             
    args = parser.parse_args()
    return args

def import_ods_file(path_to_ods_file):
    return pd.read_excel(path_to_ods_file, engine="odf",index_col=0)

def get_data(path_to_ods_files):
    files = glob.glob("{0}/*.ods".format(path_to_ods_files)) # get all the .ods files paths in the directory provided from user
    data = {}
    for file in files:
        data[file.split(path_to_ods_files+"/")[-1].split(".ods")[0]] = import_ods_file(file)
    return data

def get_planners(data):
    return list(data[list(data.keys())[0]].index)

def get_metrics(data):
    return list(data[list(data.keys())[0]].keys())

def load_config_yaml():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    with open(dir_path+"/eval_config.yaml", "r") as stream:
        config = yaml.safe_load(stream)
        return config["color_mapping"],config["label_mapping"],config["leave_out_planner"]

def plot_bars(data):
    print("----------")
    print("Begin plotting.")
    color_mapping, label_mapping, leave_out_planner = load_config_yaml()
    df_names = list(data.keys())
    planners = get_planners(data)
    try:
        for leave_out in leave_out_planner:
            planners.remove(leave_out)
            print(leave_out + " was left out.")
    except TypeError:
        print("No planners were left out.")
    except ValueError:
        planners = get_planners(data)
        print("One or more planner to leave out not found. Check spelling. All planners plotted.")
    metrics = get_metrics(data)

    bar_num = len(planners)
    bar_width = 0.1
    x_ticks_base = np.repeat(np.ceil(bar_num*bar_width)+bar_width,len(df_names))*np.arange(len(df_names)) # base for x position of bars

    dir_path = os.path.dirname(os.path.realpath(__file__))
    time_now = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    path = os.path.join(dir_path,"plots_"+str(time_now))
    os.mkdir(path)

    for metric in metrics:
        fig,ax = plt.subplots()
        for i,df_name in enumerate(df_names):
            for j,planner in enumerate(planners):
                if i == 0:
                    ax.bar(x_ticks_base[i]+j*bar_width, data[df_name].loc[planner,metric], width=bar_width, label=label_mapping[planner],  color=color_mapping[planner])
                else:
                    ax.bar(x_ticks_base[i]+j*bar_width, data[df_name].loc[planner,metric], width=bar_width,  color=color_mapping[planner])
        plt.xlabel('No. Obstacles', fontsize=15)
        plt.xticks([r + ((len(planners))/2-0.5)*bar_width for r in x_ticks_base], [label_mapping[df_name] for df_name in df_names]) # NOTE: hardcoded for obs quantity
        plt.ylabel('{}'.format(label_mapping[metric]), fontsize=15)
        plt.title("{} over No. Obstacles".format(label_mapping[metric]), fontweight='bold', fontsize=16)
        ax.grid('on')
        plt.legend(loc='upper left')
        plt.savefig(path+'/{0}.png'.format(metric))
    print("Finish plotting.")
    print("----------")


if __name__ == "__main__": # execute code
    args = parsing()
    path_to_ods_files = args.path_to_ods_files

    data = get_data(path_to_ods_files)

    plot_bars(data)