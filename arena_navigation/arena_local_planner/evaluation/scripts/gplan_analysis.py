import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import re

def gplan_to_df(gplan_csv, reset_csv): # takes csv file names as arguments
    gplan = pd.read_csv(gplan_csv) # read csv file

    global_plan = {} # initialize dict for storing lists
    global_plan["pos"] = []
    global_plan["time"] = []

    ### cleaning and structuring of poses column content ###
    poses = gplan["poses"]
    for t,time in enumerate(poses):
        strings = time.split("position: \n    ") # split by "position: \n    "", now every string start with x coordinate
        strings = strings[1:] # drop first element (unrelevant), no x,y coordinates here
        strings = [re.sub("\n", "", i) for i in strings] # remove "\n"
        strings = [i[0:i.find("z: 0.0  orientation:")-1] for i in strings] # cut off everything after the x,y coordinates
        strings = [re.sub(" ", "", i) for i in strings] # remove whitespaces
        strings = [i.split("y:") for i in strings] # split the string in x and y coordinate and remove "y:"
        strings = [[re.sub("x:", "", i[0]),i[1]] for i in strings] #  remove "x:" and return [x,y]
        strings = [[np.float(coord) for coord in i] for i in strings] # convert from string to float
        global_plan["pos"].append([[coords[1]*(-1),coords[0]] for coords in strings]) # append the position list to dictionary, x and y inverted to match ros
        global_plan["time"].append(gplan.loc[t,"Time"]) # append the time to the dictionary

    global_plan_df = pd.DataFrame.from_dict(global_plan) # create pandas dataframe from dict
    reset = pd.read_csv(reset_csv)

    global_plan_df["run"] = [is_run(t,reset) for t in global_plan_df["time"]] # create run column in global_plan_df

    return global_plan_df # return cleaned and structured pandas dataframe

def plot_run(global_plan_df,run = 1, color = "blue"):
    replannings = len(global_plan_df.loc[lambda global_plan_df: global_plan_df["run"] == run, "pos"]) # number of replannings
    for x in range(len(global_plan_df.loc[lambda global_plan_df: global_plan_df["run"] == run, "pos"])): # select global plans of the run
        plt.plot(*zip(*global_plan_df.loc[x, "pos"]), alpha = 1/(replannings-x+1), c = color, lw = 3) # need to zip those x,y pairs
    # plt.show()
    print("Number of replannings during this run: " + str(replannings)) # print number of replannings

def is_run(time,resets): # function for assigning the run/episode
    for j in range(len(resets)):
        if time >= resets.loc[len(resets)-1,"Time"] :
            return resets.loc[len(resets)-1,"data"]
            break
        if time >= resets.loc[j,"Time"]:
            continue
        else:
            return resets.loc[j-1,"data"]
            break

