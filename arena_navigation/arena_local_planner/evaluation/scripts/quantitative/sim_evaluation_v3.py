# example 
# python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --quantity vel --base cadrl --metrics time path collision success


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd 
import argparse # get user input from terminal as arguments
import glob # usefull for listing all files of a type in a directory

"""
NOTE: evaluation file names must have this format wpgen_planner_map_obs05_vel01.json (order is not relevant)
wpgen: name of waypoint generator
planner: name of planner used
map: map on which simulation was run
obs01: number of obstacles is 5
vel01: dyn. obstacle velocity is 0.1

"""

def parsing(): # define what flags user can/must give as terminal input
    parser = argparse.ArgumentParser(description='Evaluate simulation runs.') # create parser object
    parser.add_argument('path', type=str, action='store',
                        help='path to directory where JSON files to be evaluated are') # store path to directory where JSON file lie, mandatory
    parser.add_argument('--metrics', action='store', nargs='+', choices=["time","path","collision","success"], default="none",
                        help='list of metrics to be evaluated') # store list of metrics to be evaluated, mandatory at least 1
    parser.add_argument('--quantity', action='store', nargs='?', default='obs', choices=['obs','vel'],
                        help='plot evaluation graphs over obstacle number (obs) or velocity (vel)') # store over which quantity metrics should be evaluated, default = obs                        
    parser.add_argument('--allplot_quantity', action='store', nargs='?', default='none', choices=['obs','vel'],
                        help='plot all in one plot over obstacle number (obs) or velocity (vel)') # store over which quantity metrics should be evaluated, default = none                        
    parser.add_argument('--latex', action='store_true',
                        help='flag: write dataframe as latex table into a txt file') # store TRUE if flag given, else FALSE per default, optional                        
    parser.add_argument('--legendsoff', action='store_false',
                        help='flag: print graphs without legends') # store False if flag given, else True per default, optional
    parser.add_argument('--show', action='store_true',
                        help='flag:show plots via plt.show()') # store TRUE if flag given, else FALSE per default, optional
    parser.add_argument('--csv', action='store_true',
                        help='flag:export dfs to csv') # store TRUE if flag given, else FALSE per default, optional                        
    parser.add_argument('--withclassic', action='store_true',
                        help='flag:plot classic planners into metrics plots') # store TRUE if flag given, else FALSE per default, optional     
    parser.add_argument('--byplanner', action='store_true',
                        help='flag:plot metrics divided into waypoint generator for every local planner') # store TRUE if flag given, else FALSE per default, optional                               
    parser.add_argument('--nosubtitle', action='store_false',
                        help='flag:plot metrics without subtitle specifying planner/wpgen') # store TRUE if flag given, else FALSE per default, optional                               
    args = parser.parse_args()
    return args

def summary(df,cols,f): # summary of the json files and returning as single row as pandas df with file name as index
    l =[]
    l.append(np.mean(df["time"])) # calculate mean time
    l.append(np.mean(df["path"])) # calculate mean path length
    l.append(np.float(np.sum(df["collision"]))) # calculate total number of collisions
    l.append(np.sum(df["collision"]<3)/len(df["collision"])*100) # calculate fraction of successes
    index_name = [f.split("/")[-1].replace(".json","")] # get the file name with out .json ending
    return pd.DataFrame([l], columns=cols, index=index_name)

def get_data(path,cols): # create dataframe containing ALL data from the json files
    data = pd.DataFrame([],columns=cols) # initialize the pandas df
    files = glob.glob("{0}/*.json".format(path)) # get all the JSON files paths in the directory provided from user
    for f in files: # summarize all the json files and append to dataframe
        df = pd.read_json(f)
        pdf = summary(df,cols,f)
        data = data.append(pdf)
    return data

def order_data(data,param_list,wpgen,planner,maps,classic,quantity): # create wpgen, planner, map, parameter columns for future computations
    other_quantity = np.array([*param_list.keys()])[[x != quantity for x in [*param_list.keys()]]][0] # get the other quantity
    indices = list(data.index)
    params = [quantity,other_quantity]
    wpgen_col = ["none"]*len(indices)
    planner_col = ["none"]*len(indices)
    map_col = ["none"]*len(indices)
    parameter_col = {}
    for param in params:
        parameter_col[param] = ["none"]*len(indices)

    for i,index in enumerate(indices): # create new columns from index
        for wp in wpgen:
            if wp in index:
                wpgen_col[i] = wp
                break
        for p in planner:
            if p in index:
                planner_col[i] = p
                if p in classic:
                    wpgen_col[i] = "classic"
                break
        for map in maps:
            if map in index:
                map_col[i] = map
                break
        for param in params:
            for p in param_list[param]:
                if p in index:
                    parameter_col[param][i] = p
                    break
    data["wpgen"] = wpgen_col
    data["planner"] = planner_col
    data["map"] = map_col
    for param in params:
        data[param] = parameter_col[param]
    data.index = ["{0}_{1}".format(data["wpgen"][i],data["planner"][i]) for i in range(len(data))] # set new indices with {wpgen}_{planner}

    # dropping data with "none" in columns
    data = data.loc[data["map"]!="none"]
    data = data.loc[data["wpgen"]!="none"]
    data = data.loc[data["planner"]!="none"]
    for param in params:
        data = data.loc[data[param]!="none"]

    return data.sort_index()

def get_table(data,wpgen,planner,maps,param_list,cols):
    params = [*param_list.keys()]
    data_grouped = data.groupby(["map"]+params+["wpgen","planner"]).mean() 
    data_overall = data.groupby(params+["wpgen","planner"]).mean()
    table = {}
    table["overall"] = {}
    for map in maps:
        table[map] = {}
        for p in param_list[params[0]]: # first hierarchy 1st param (obs), 2nd hierarchy 2nd param (vel), NOTE: change hierarchy by changing param_list order
            table[map][p] = pd.concat([data_grouped.loc[map].loc[p].loc[p2] for p2 in param_list[params[1]]],axis=1)
        table[map] = pd.concat([table[map][p] for p in param_list[params[0]]],axis=1).round(2)
    for p in param_list[params[0]]: # first hierarchy 1st param (obs), 2nd hierarchy 2nd param (vel), NOTE: change hierarchy by changing param_list order
        table["overall"][p] = pd.concat([data_overall.loc[p].loc[p2] for p2 in param_list[params[1]]],axis=1)
    table["overall"] = pd.concat([table["overall"][p] for p in param_list[params[0]]],axis=1).round(2)
    return table
    """
    table structure for each map respectively:
                obs05               obs10                       obs20
            vel02 vel03    vel02   vel03   vel02   vel03
            time|path|collision|succces (for every vel)   
    planner
    """

def clear_missings(data,maps,wpgen,planner,param_list,quantity):
    # check which bags are missing and create zero array for missing bags
    other_quantity = np.array([*param_list.keys()])[[x != quantity for x in [*param_list.keys()]]][0] # get the other quantity
    new_index = pd.MultiIndex.from_arrays([data["map"],data["wpgen"],data["planner"],data[other_quantity],data[quantity]], names=("map",'wpgen','planner',other_quantity,quantity))
    data.index = new_index
    data = data.sort_index()
    result = pd.DataFrame(columns=data.columns)
    
    for map in maps:
        for wp in wpgen:
            for plan in planner:
                if wp != "classic" and plan in classic:
                    continue
                if wp == "classic" and plan not in classic:
                    continue 
                for q2 in param_list[other_quantity]:
                    for q1 in param_list[quantity]:
                        try:
                            data.loc[map].loc[wp].loc[plan].loc[q2].loc[q1,"time"]
                        except:
                            l = [0,0,0,0,wp,plan,map,q1,q2]
                            print("Following data were missing and filled with zero vectors: ",map,wp,plan,q1,q2)
                            result = result.append(pd.DataFrame([l], columns=data.columns))
    new_index = pd.MultiIndex.from_arrays([result["map"],result["wpgen"],result["planner"],result[other_quantity],result[quantity]], names=("map",'wpgen','planner',other_quantity,quantity))
    result.index = new_index                        
    result = result.sort_index()
    data = data.append(result).sort_index()
    return data

def plot_metrics(data,labels,colors,wpgen,planner,maps,param_list,quantity,metrics,legendsoff,show,classic,withclassic,byplanner,nosubtitle):
    barwidth = 0.1 # NOTE: tuned manually
    fontsize_ticks = 13
    fontsize_title = 19
    fontsize_subtitle = 14
    fontsize_axlabel = 19
    
    other_quantity = np.array([*param_list.keys()])[[x != quantity for x in [*param_list.keys()]]][0] # get the other quantity
    new_index = pd.MultiIndex.from_arrays([data["map"],data["wpgen"],data["planner"],data[other_quantity],data[quantity]], names=("map",'wpgen','planner',other_quantity,quantity))
    if byplanner:
        new_index = pd.MultiIndex.from_arrays([data["map"],data["planner"],data["wpgen"],data[other_quantity],data[quantity]], names=("map",'planner','wpgen',other_quantity,quantity))
    data.index = new_index
    data = data.sort_index() # prepare new indice and sorting for better accessability of data
    r = {} # x positions for the bar lots

    for map in maps: # NOTE: replace ["map1"] by maps
        map_data = data.loc[map] # filter by map
        xticks = len(param_list[quantity]) # number of x ticks (quantity values)
    
        ########################## plot by planner ###########################################
        if byplanner:
            for plan in planner:
                planner_data = map_data.loc[plan]
                if plan in classic: # ignore classic planners in planners
                    continue
                for metric in metrics:
                    fig, ax = plt.subplots()
                    iterate = wpgen
                    if withclassic:
                        iterate = wpgen + classic
                    for i,iter in enumerate(iterate):
                        if iter == "classic": # ignore classic in wpgen
                            continue
                        else:
                            if i == 0:
                                r[iter] = np.arange(xticks) # number of x ticks
                            else:
                                if iterate[i-1] == "classic":
                                    r[iter] = [x + barwidth for x in r[iterate[i-2]]]
                                else:
                                    r[iter] = [x + barwidth for x in r[iterate[i-1]]]
                        if iter in classic:
                            wp_data = data.loc[map].loc[iter].loc["classic"]
                        else:
                            wp_data = planner_data.loc[iter]
                        for a,oq in enumerate(param_list[other_quantity]):
                            oq_data = wp_data.loc[oq]
                            l = len(param_list[other_quantity]) # number of other quantity to map with alpha
                            if metric == "success": # success needs inverse alphas because its descending
                                if a == l-1: # label only for the series most in front which in success is the one with highest quantity value
                                    ax.bar(r[iter], oq_data[metric], width=barwidth, label=labels[iter],  color=colors[iter], alpha=(a+1)/l)
                                else:
                                    ax.bar(r[iter], oq_data[metric], width=barwidth,  color=colors[iter], alpha=(a+1)/l)
                            else:
                                if a == 0:
                                    ax.bar(r[iter], oq_data[metric], width=barwidth, label=labels[iter],  color=colors[iter], alpha=1-a/l)
                                else:
                                    ax.bar(r[iter], oq_data[metric], width=barwidth,  color=colors[iter], alpha=1-a/l)
                    #captions
                    caption = ""
                    if metric == "path":
                        caption = "avg. Path Length [m]"
                        title = "Path Length"
                    if metric == "time":
                        caption = "avg. Time t.g. [s]"
                        title = "Time t.g."
                    if metric == "success":
                        caption = "Success"
                        title = caption
                    if metric == "collision":
                        caption = "Collisions"
                        title = caption
                    # plot labeling and title
                    if quantity == "obs":
                        # Add xticks on the middle of the group bars
                        plt.xlabel('No. Obstacles', fontsize=fontsize_axlabel)
                        plt.xticks([r + ((len(iterate)-1)/2-0.5)*barwidth for r in range(xticks)], [int(obs.replace("obs","")) for obs in param_list["obs"]], fontsize=fontsize_ticks) # NOTE: hardcoded for obs quantity
                        plt.yticks(fontsize=fontsize_ticks)
                        plt.ylabel('{}'.format(caption), fontsize=fontsize_axlabel)
                        plt.suptitle("{} over No. Obstacles".format(title), fontweight='bold', fontsize=fontsize_title)
                        if nosubtitle:
                            plt.title("Local Planner: {0}, Map: {1}".format(labels[plan],labels[map]), fontsize=fontsize_subtitle)
                        else:
                            plt.title("Map: {1}".format(labels[map]), fontsize=fontsize_subtitle)
                        ax.grid('on')
                        # Create legend & Show graphic
                        if legendsoff:
                            plt.legend(loc='upper left')
                        if withclassic:
                            plt.savefig('{0}_obs_{1}_byplanner_{2}_withclassic.png'.format(metric,map,plan))
                        else:
                            plt.savefig('{0}_obs_{1}_byplanner_{2}.png'.format(metric,map,plan))
                        if show:
                            plt.show()
                        else:
                            plt.close()                            
                    if quantity == "vel":
                        # Add xticks on the middle of the group bars
                        plt.xlabel('Obstacle Velocity', fontsize=fontsize_axlabel)
                        plt.xticks([r + ((len(iterate)-1)/2-0.5)*barwidth for r in range(xticks)], ["0.{0}".format(int(vel.replace("vel",""))) for vel in param_list["vel"]], fontsize=fontsize_ticks) # NOTE: hardcoded for vel quantity
                        plt.yticks(fontsize=fontsize_ticks)
                        plt.ylabel('{}'.format(caption), fontsize=fontsize_axlabel)
                        plt.suptitle("{} over Obstacle Velocity".format(title), fontweight='bold', fontsize=fontsize_title)
                        if nosubtitle:
                            plt.title("Local Planner: {0}, Map: {1}".format(labels[plan],labels[map]), fontsize=fontsize_subtitle)
                        else:
                            plt.title("Map: {1}".format(labels[map]), fontsize=fontsize_subtitle)                        
                        ax.grid('on')
                        # Create legend & Show graphic
                        if legendsoff:
                            plt.legend(loc='upper left')
                        if withclassic:
                            plt.savefig('{0}_vel_{1}_byplanner_{2}_withclassic.png'.format(metric,map,plan))
                        else:
                            plt.savefig('{0}_vel_{1}_byplanner_{2}.png'.format(metric,map,plan))
                        if show:
                            plt.show()
                        else:
                            plt.close()

        ########################## plot by waypoint generator ###########################################
        else:
            for wp in wpgen:
                # skip plotting only classic planners, NOTE: can be commented out if necessary
                if wp == "classic":
                    continue
                wp_data = map_data.loc[wp]
                for metric in metrics:
                    fig, ax = plt.subplots()
                    for i,plan in enumerate(planner):
                        if wp == "classic": # for plotting only classic planners r values have to be set differently and only for classic planners
                            if plan in classic:
                                if plan == classic[0]:
                                    r[plan] = np.arange(xticks) # number of x ticks
                                else:
                                    r[plan] = [x + barwidth for x in r[planner[i-1]]]
                        else:
                            if i == 0:
                                r[plan] = np.arange(xticks) # number of x ticks
                            else:
                                r[plan] = [x + barwidth for x in r[planner[i-1]]]
                        if plan in classic and wp != "classic":
                            if withclassic:
                                planner_data = data.loc[map].loc["classic"].loc[plan]
                            else:
                                continue
                        elif plan not in classic and wp == "classic":
                            continue
                        else:
                            planner_data = wp_data.loc[plan]
                        for a,oq in enumerate(param_list[other_quantity]):
                            oq_data = planner_data.loc[oq]
                            l = len(param_list[other_quantity]) # number of other quantity to map with alpha
                            if metric == "success":
                                if a == l-1: # label only for the series most in front which in success is the one with highest quantity value
                                    ax.bar(r[plan], oq_data[metric], width=barwidth, label=labels[plan],  color=colors[plan], alpha=(a+1)/l)
                                else:
                                    ax.bar(r[plan], oq_data[metric], width=barwidth,  color=colors[plan], alpha=(a+1)/l)
                            else:
                                if a == 0:
                                    ax.bar(r[plan], oq_data[metric], width=barwidth, label=labels[plan],  color=colors[plan], alpha=1-a/l)
                                else:
                                    ax.bar(r[plan], oq_data[metric], width=barwidth,  color=colors[plan], alpha=1-a/l)
                  #captions
                    caption = ""
                    if metric == "path":
                        caption = "avg. Path Length [m]"
                        title = "Path Length"
                    if metric == "time":
                        caption = "avg. Time t.g. [s]"
                        title = "Time t.g."
                    if metric == "success":
                        caption = "Success"
                        title = caption
                    if metric == "collision":
                        caption = "Collisions"
                        title = caption
                    # plot labels and title
                    if quantity == "obs":
                        # Add xticks on the middle of the group bars
                        plt.xlabel('No. Obstacles', fontsize=fontsize_axlabel)
                        planners_plotted = len(planner)
                        if wp == "classic":
                            planners_plotted = len(classic)
                        if wp != "classic" and not withclassic:
                            planners_plotted = len(planner)-len(classic)
                        plt.xticks([r + (planners_plotted/2-0.5)*barwidth for r in range(xticks)], [int(obs.replace("obs","")) for obs in param_list["obs"]], fontsize=fontsize_ticks) # NOTE: hardcoded for obs quantity
                        plt.yticks(fontsize=fontsize_ticks)
                        plt.ylabel('{}'.format(caption), fontsize=fontsize_axlabel)
                        plt.suptitle("{} over No. Obstacles".format(title), fontweight='bold', fontsize=fontsize_title)
                        if nosubtitle:
                            if wp == "classic": # NOTE: if you dont want a subtitle with the wpgenerator comment the following lines out
                                plt.title("Classic Navigation Systems, Map: {0}".format(labels[map]), fontsize=fontsize_subtitle)
                            else:
                                plt.title("Waypoint generator: {0}, Map: {1}".format(labels[wp],labels[map]), fontsize=fontsize_subtitle)                            
                        else:
                            plt.title("Map: {1}".format(labels[map]), fontsize=fontsize_subtitle)                         
                        ax.grid('on')
                        # Create legend & Show graphic
                        if legendsoff:
                            plt.legend(loc='upper left')
                        if withclassic:
                            plt.savefig('{0}_obs_{1}_{2}_withclassic.png'.format(metric,map,wp))
                        else:
                            plt.savefig('{0}_obs_{1}_{2}.png'.format(metric,map,wp))
                        if show:
                            plt.show()
                        else:
                            plt.close()
                    if quantity == "vel":
                        # Add xticks on the middle of the group bars
                        plt.xlabel('Obstacle Velocity', fontsize=fontsize_axlabel)
                        planners_plotted = len(planner)
                        if wp == "classic":
                            planners_plotted = len(classic)
                        if wp != "classic" and not withclassic:
                            planners_plotted = len(planner)-len(classic)
                        plt.xticks([r + (planners_plotted/2-0.5)*barwidth for r in range(xticks)], ["0.{0}".format(int(vel.replace("vel",""))) for vel in param_list["vel"]], fontsize=fontsize_ticks) # NOTE: hardcoded for vel quantity
                        plt.yticks(fontsize=fontsize_ticks)
                        plt.ylabel('{}'.format(caption), fontsize=fontsize_axlabel)
                        plt.suptitle("{} over Obstacle Velocity".format(title), fontweight='bold', fontsize=fontsize_title)
                        if nosubtitle:
                            if wp == "classic": # NOTE: if you dont want a subtitle with the wpgenerator comment the following lines out
                                plt.title("Classic Navigation Systems, Map: {0}".format(labels[map]), fontsize=fontsize_subtitle)
                            else:
                                plt.title("Waypoint generator: {0}, Map: {1}".format(labels[wp],labels[map]), fontsize=fontsize_subtitle)                            
                        else:
                            plt.title("Map: {1}".format(labels[map]), fontsize=fontsize_subtitle) 
                        ax.grid('on')
                        # Create legend & Show graphic
                        if legendsoff:
                            plt.legend(loc='upper left')
                        if withclassic:                            
                            plt.savefig('{0}_vel_{1}_{2}_withclassic.png'.format(metric,map,wp))
                        else:
                            plt.savefig('{0}_vel_{1}_{2}.png'.format(metric,map,wp))
                        if show:
                            plt.show()
                        else:
                            plt.close()

def get_all_plot(data,labels,colors,wpgen,planner,param_list,quantity,metrics,show,classic):
    barwidth = 0.1
    fontsize_ticks = 16
    fontsize_title = 24
    fontsize_subtitle = 24
    fontsize_axlabel = 24

    other_quantity = np.array([*param_list.keys()])[[x != quantity for x in [*param_list.keys()]]][0] # get the other quantity
    bar_number = (len(planner)-len(classic))*(len(wpgen)-1)+len(classic) # nonclassic planners*waypoints without classic + classic planners
    xticks = len(wpgen)-1
    l = len(param_list[other_quantity]) # number of values of the other quantity depicted in alpha

    new_index = pd.MultiIndex.from_arrays([data["map"],data[quantity],data[other_quantity],data["planner"],data["wpgen"]], names=("map",quantity,other_quantity,'planner','wpgen'))
    data.index = new_index
    r = {} # x positions for the bar lots

    for metric in metrics:
        fig, axs = plt.subplots(len(maps),len(param_list[quantity]), sharey=True, sharex=True, figsize = [len(param_list[quantity])*5,len(maps)*5]) # rows: number of maps, columns: number of quantity values
        for map in maps:
            for q in param_list[quantity]:
                for a,oq in enumerate(param_list[other_quantity]):
                    plot_data = data.loc[map].loc[q].loc[oq]
                    for i,plan in enumerate(planner):
                        if plan in classic:
                            continue
                        planner_data = plot_data.loc[plan]
                        if i == 0:
                            r[plan] = np.arange(xticks) # number of x ticks
                        else:
                            r[plan] = [x + barwidth for x in r[planner[i-1]]]
                        if metric == "success":
                            if a == l-1: # label only for the series most in front which in success is the one with highest quantity value
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth, label=labels[plan],  color=colors[plan], alpha=(a+1)/l)
                            else:
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth,  color=colors[plan], alpha=(a+1)/l)
                        else:
                            if a == 0:
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth, label=labels[plan],  color=colors[plan], alpha=1-a/l)
                            else:
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth,  color=colors[plan], alpha=1-a/l)
                    for i,plan in enumerate(classic):
                        planner_data = plot_data.loc[plan]
                        if i == 0:
                            r[plan] = wpgen.index("classic")
                        else:
                            r[plan] = r[classic[i-1]]+barwidth
                        if metric == "success":
                            if a == l-1: # label only for the series most in front which in success is the one with highest quantity value
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth, label=labels[plan],  color=colors[plan], alpha=(a+1)/l)
                            else:
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth,  color=colors[plan], alpha=(a+1)/l)
                        else:
                            if a == 0:
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth, label=labels[plan],  color=colors[plan], alpha=1-a/l)
                            else:
                                axs[maps.index(map),param_list[quantity].index(q)].bar(r[plan], planner_data[metric], width=barwidth,  color=colors[plan], alpha=1-a/l)
                # captions
                caption = ""
                if metric == "path":
                    caption = "avg. Path Length [m]"
                    title = "Path Length"
                if metric == "time":
                    caption = "avg. Time t.g. [s]"
                    title = "Time t.g."
                if metric == "success":
                    caption = "Success"
                    title = caption
                if metric == "collision":
                    caption = "Collisions"
                    title = caption
                # labels and titles for each individual plot
                non_classic_planners_plotted = len(planner)-len(classic)
                classic_planners_plotted = len(classic)
                xpos = [r + (non_classic_planners_plotted/2-0.5)*barwidth for r in range(xticks)] # these are the xticks for the non-classical planner
                xpos.append(xticks+(classic_planners_plotted/2-0.5)*barwidth) # the last tick for the classic planners
                axs[maps.index(map),param_list[quantity].index(q)].set_xticks(xpos)
                axs[maps.index(map),param_list[quantity].index(q)].set_xticklabels([labels[wp] for wp in wpgen], fontsize=fontsize_ticks)
                axs[maps.index(map),param_list[quantity].index(q)].set(ylabel="{0}".format(caption), fontsize=fontsize_axlabel)
                if quantity == "vel":
                    axs[maps.index(map),param_list[quantity].index(q)].set_title("Map: {0}, {1}: 0.{2}".format(labels[map],labels[quantity],int(q.replace(quantity,""))), fontsize=fontsize_title)
                else:
                    axs[maps.index(map),param_list[quantity].index(q)].set_title("Map: {0}, {1}: {2}".format(labels[map],labels[quantity],int(q.replace(quantity,""))), fontsize=fontsize_title)
                axs[0,0].legend(loc="upper left")
        for ax in axs.flat:
            ax.label_outer()
            ax.grid("on")
        fig.suptitle("{0} over Navigation Systems for all scenarios".format(title), fontweight='bold', fontsize=fontsize_title)
        plt.savefig('all_in_one_plot_{0}_{1}.png'.format(metric,quantity))  
        if show:
            plt.show()
        else:
            plt.close()      





if __name__ == "__main__": # execute code

    ### this block reads in terminal input of user and stores the flags for later ###
    args = parsing()
    path = args.path
    latex = args.latex
    metrics = args.metrics
    quantity = args.quantity
    legendsoff = args.legendsoff
    show = args.show
    csv = args.csv
    withclassic = args.withclassic
    byplanner = args.byplanner
    nosubtitle = args.nosubtitle
    allplot_quantity = args.allplot_quantity
    ######


    ### this code block creates dataframes containing all evaluation file's summarized data ###
    # hyperparameters specifying the contents of files and the wanted structure for the datasets
    cols = ["time","path","collision","success"]    # define the quantities to measure
    obs = ["obs10","obs20", "obs30"] # define different obstacles numbers, names must match file names
    vel = ["vel03"] # define different velocities, names must match file names
    maps = ["empty","map0","open"] # define the maps trained on, names must match file names
    wpgen = ["spatialhorizon", "classic"] # NOTE: classic MUST be in the back
    planner = ["R0","R1","R2","R4","RLCA","MPC", "TEB"] # all planners, NOTE: classic planners MUST be in the back!!!
    classic = ["MPC","TEB"] # classic planners

    # different kinds of datasets for visualization and latex table formatting
    param_list = {"obs":obs,"vel":vel} # just a dict with the parameters to measure
    raw_data = get_data(path,cols)
    data = order_data(raw_data,param_list,wpgen,planner,maps,classic,quantity)
    ############

    ### this code block creates a df in table format needed for latex and may also create csv files of the df ###
    table = get_table(data,wpgen,planner,maps,param_list,cols)
    if latex:
        text_file = open("latex_table.txt", "w")
        for t in table:
            text_file.write("\nTable for: "+t+"\n")
            text_file.write(table[t].to_latex())
        text_file.close()

    if csv:
        data.to_csv("evaluation_all_data.csv")
        for t in table:
            table[t].to_csv("evaluation_{}_table.csv".format(t))
    ###########


    ### this code block plots graphs of the metrics provided by user over the provided quantity ###
    labels = { # labels which will be used in the plots e.g. xticks, x/ylabels, titles, legend
        "cadrl":"CADRL", 
        "esdf": "LM-WP",
        "MPC": "MPC",
        "subsampling": "SUB-WP",
        "TEB": "TEB",
        "drl1": "DRL1",
        "drl2": "DRL2",
        "drl3": "DRL3",
        "RLCA":"RLCA",
        "ego": "EGO",
        "R0": "R0",
        "R1": "R1",
        "R2": "R2",
        "R4": "R4",
        "ego": "EGO",
        "spatialhorizon": "STH-WP",
        "timespace": "TS-WP",
        "rlca": "COL-RL",
        "map1": "Map 1",
        "empty": "Empty Map",
        "map0": "Map 0",
        "open": "Open Field",
        #-------------------------------------
        "classic": "Classic",
        "obs" : "Obstacle Number",
        "vel" : "Velocity"
        }
    colors = {  # color scheme for local planners plus classic planners
        "MPC":"tab:red",
        "TEB": "tab:blue",
        "RLCA": "tab:green",
        "R0": "tab:pink",
        "R1": "tab:purple",
        "R2": "tab:orange",
        "R4": "tab:cyan"
        }
 
    colors_wp = { # color scheme for waypoint generator plus classic planners
        "subsampling":"tab:red",
        "timespace": "tab:blue",
        "spatialhorizon": "tab:green",
        "teb": "tab:orange",
        "mpc": "tab:cyan",
        "ego": "tab:grey",
        }

    data = clear_missings(data,maps,wpgen,planner,param_list,quantity)

    # plotting metrics
    if metrics != "none":
        if byplanner:
            plot_metrics(data,labels,colors_wp,wpgen,planner,maps,param_list,quantity,metrics,legendsoff,show,classic,withclassic,byplanner,nosubtitle)     
        else:
            plot_metrics(data,labels,colors,wpgen,planner,maps,param_list,quantity,metrics,legendsoff,show,classic,withclassic,byplanner,nosubtitle)
        if allplot_quantity != "none":
            get_all_plot(data,labels,colors,wpgen,planner,param_list,allplot_quantity,metrics,show,classic)
    #########

    # use for finding errors in the data/json files
    # therefore command out the plotting metrics commands
    # pd.set_option('display.max_rows', None) # commands to print everything in panda DataFrame
    # pd.set_option('display.max_columns', None)
    # pd.set_option('display.width', None)
    # pd.set_option('display.max_colwidth', None)
    # print(data)