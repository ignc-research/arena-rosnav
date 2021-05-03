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
        global_plan["time"].append(np.float(gplan.loc[t,"Time"])) # append the time to the dictionary

    global_plan_df = pd.DataFrame.from_dict(global_plan) # create pandas dataframe from dict
    reset = pd.read_csv(reset_csv)

    global_plan_df["run"] = [is_run(t,reset) for t in global_plan_df["time"]] # create run column in global_plan_df

    return global_plan_df # return cleaned and structured pandas dataframe

def plot_run(global_plan_df,run = 1, color = "tab:cyan", pwp="True"):
    run_df = global_plan_df.loc[lambda global_plan_df: global_plan_df["run"] == run,:]
    replannings = len(run_df) # number of replannings
    for x in range(len(run_df)): # select global plans of the run
        plt.plot(*zip(*run_df.loc[run_df.index[x],"pos"]), alpha = np.exp(-x), c = color, lw = 3, ls = "dashed") # need to zip those x,y pairs
        # plt.scatter(*zip(*run_df.loc[run_df.index[x],"pos"][0::100]), alpha = x/len(run_df), c = color, marker = "x", s = 100) # plot marks on global path for every gp
    if pwp:
        plt.scatter(*zip(*run_df.loc[run_df.index[0],"pos"][0::100]), alpha = 1, c = color, marker = "o", s = 50) # plot marks for only last global plan
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
            return resets.loc[j,"data"]
            break

    

# example
#esdf = gplan_to_df("sensorsim-police-gplan.csv","scenario_reset.csv")
#plot_run(esdf, 10)