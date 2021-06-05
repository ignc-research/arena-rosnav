# for data
import sys
import copy
import pprint as pp
import bagpy
from bagpy import bagreader
import pandas as pd
import json
import rospkg
import yaml
# for plots
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.pyplot import figure
from matplotlib.patches import Polygon
from matplotlib._png import read_png
import matplotlib.cm as cm
# calc
import numpy as np
import math
import seaborn as sb
import rospy 
from visualization_msgs.msg import Marker, MarkerArray
import pathlib
import os
from sklearn.cluster import AgglomerativeClustering
# gplan
import gplan_analysis as gplan
matplotlib.rcParams.update({'font.size': 15})
# 
from termcolor import colored, cprint
class newBag():
    def __init__(self, planner, file_name, bag_name):
        # planner
        self.planner         = planner.split("wpg")[0]
        self.wpg             = planner.split("wpg")[1]
        self.file_name       = file_name
        # csv dir
        self.csv_dir         = bag_name.replace(".bag","")
        # bag topics
        self.odom_topic      = "/sensorsim/police/odom"
        self.collision_topic = "/sensorsim/police/collision"
        self.subgoal_topic   = "/sensorsim/police/subgoal"
        self.gp_topic        = "/sensorsim/police/gplan"
        self.wpg_topic       = "/sensorsim/police/subgoal_wpg"
        # global apth
        self.plot_gp = True

        self.nc_total = 0
        # eval bags
        self.bag = bagreader(bag_name)
        eps = self.split_runs()
        if len(eps) != 0:
            self.evalPath(self.planner, file_name, eps)
        else:
            print("no resets for: " + bag_name)


    def make_json(self, data):
        fn = self.file_name
        fa = fn.split("_")
        fn = fa[0] +"_"+ fa[1] + "_" + fa[2] + "_" + self.wpg

        jfile = "quantitative/" + self.planner + "_" + fn + ".json"
        if not os.path.isfile(jfile): 
            with open(jfile, 'w') as outfile:
                json.dump(data, outfile, indent=2)

    def make_txt(self,file,msg,ron="a"):
        file = file.replace("/","_") + ".txt"
        # write all modes in same scenario file
        file = file.replace("arena","")
        file = file.replace("cadrl","")
        file = file.replace("dwa","")
        file = file.replace("teb","")
        file = file.replace("mpc","")
        file = file.replace("esdf","")
        file = file.replace("subsample","")
        file = "quantitative/" + file

        f = open(file, ron)
        f.write(msg)
        f.close()
        # return

    def split_runs(self):
        # get odometry
        
        odom_csv = self.bag.message_by_topic(self.odom_topic)
        df_odom  = pd.read_csv(odom_csv, error_bad_lines=False)

        df_collision = []
        df_subg      = []
        df_gp        = []
        df_wpg       = []

        # get topics
        try:
            # check if collision was published
            collision_csv = self.bag.message_by_topic(self.collision_topic)
            df_collision  = pd.read_csv(collision_csv, error_bad_lines=False)

            #check if subgoals in bag
            subg_csv = self.bag.message_by_topic(self.subgoal_topic)
            df_subg  = pd.read_csv(subg_csv, error_bad_lines=False)

            gp       = self.bag.message_by_topic(self.gp_topic)
            # df_gp    = pd.read_csv(gp, error_bad_lines=False)

            wpg      = self.bag.message_by_topic(self.wpg_topic)
            df_wpg   = pd.read_csv(wpg, error_bad_lines=False)

            # print(df_goal)


        except Exception as e:
            # otherwise run had zero collisions
            print(e)


        t_col = []
   
        try:
            for i in range(len(df_collision)): 
                t_col.append(df_collision.loc[i, "Time"])   
                
            self.nc_total = len(t_col)
            # get reset time
            reset_csv = self.bag.message_by_topic("/scenario_reset")
            df_reset  = pd.read_csv(reset_csv, error_bad_lines=False)
            t_reset   = []
            for i in range(len(df_reset)): 
                t_reset.append(df_reset.loc[i, "Time"])

            # subgoals
            sg_n = 0
            subgoal_x = []
            subgoal_y = []

            # wpg
            wpg_n = 0
            wpg_x = []
            wpg_y = []

            pose_x = []
            pose_y = []
            t = []

            bags = {}
            # run idx
            n = 0
            # collsion pos
            col_xy = []
            nc = 0
            
            old_x = None
            old_y = None
            dist2_oldp = 2500


            global start, select_run

            for i in range(len(df_odom)): 
                current_time = df_odom.loc[i, "Time"]
                x = df_odom.loc[i, "pose.pose.position.x"]
                x = round(x,2)
                y = df_odom.loc[i, "pose.pose.position.y"]
                y = round(y,2)
                reset = t_reset[n]

                # print(reset)

                # check if respawned
                
                start_x = start[0] + 0.5

                dist2_oldp = 0
                if old_x != None:
                    dist2_oldp = math.sqrt((x-old_x)**2+(y-old_y)**2)

                # if current_time > reset-6 and n < len(t_reset)-1 and x < start_x:
                # if current_time > reset and n < len(t_reset)-1:
                if dist2_oldp > 1 and n < len(t_reset)-1:
                    n += 1
                    # store the run
                    if n in select_run or len(select_run) == 0:
                        bags["run_"+str(n)] = [pose_x, pose_y, t, col_xy, subgoal_x, subgoal_y, wpg_x, wpg_y]

                    # reset 
                    wpg_x     = []
                    wpg_y     = []

                    subgoal_x = []
                    subgoal_y = []

                    pose_x    = []
                    pose_y    = []
                    t         = []

                    col_xy    = []

                    old_x = None
                    old_y = None

                if n+1 in select_run or len(select_run) == 0 and dist2_oldp < 1:

                    # append pos if pose is empty
                    if len(pose_x) == 0:
                        pose_x.append(x)
                        pose_y.append(y)
                  
                    pose_x.append(x)
                    pose_y.append(y)

                    t.append(current_time)
                    # get trajectory

                    # check for col
                    if len(t_col) > nc:
                        if current_time >= t_col[nc]:
                            col_xy.append([x,y])
                            nc += 1

                    # check for goals
                    if len(df_subg) > 0:
                        sg_t = round(df_subg.loc[sg_n, "Time"],3)
                        sg_x = round(df_subg.loc[sg_n, "pose.position.x"],3)
                        sg_y = round(df_subg.loc[sg_n, "pose.position.y"],3)

                        if current_time > sg_t and sg_n < len(df_subg) - 1:

                            subgoal_x.append(sg_x)
                            subgoal_y.append(sg_y)

                            sg_n += 1

                    if len(df_wpg) > 0:
                        wp_t = round(df_wpg.loc[wpg_n, "Time"],3)
                        wp_x = round(df_wpg.loc[wpg_n, "pose.position.x"],3)
                        wp_y = round(df_wpg.loc[wpg_n, "pose.position.y"],3)

                        if current_time > wp_t and wpg_n < len(df_wpg) - 1:

                            wpg_x.append(wp_x)
                            wpg_y.append(wp_y)

                            wpg_n += 1

                    old_x = x
                    old_y = y


            # remove first 
            if "run_1" in bags:    
                bags.pop("run_1")

            df = pd.DataFrame(data=bags)
            run_csv = self.csv_dir + "/" + self.csv_dir.rsplit('/', 1)[-1] + ".csv"
            df.to_csv(run_csv,index=False)   
            print("csv created in: " + self.csv_dir)  
        except Exception as e:
            # otherwise run had zero collisions
            print(e)
            bags = {}
        return bags
    
    def average(self,lst): 
        if len(lst)>0:
            return sum(lst) / len(lst) 
        else:
             return 0

    def plot_global_plan(self,run_n,pwp):
        global plt_cfg

        if plt_cfg["plot_gp"] and self.plot_gp:
            csv_dir = self.csv_dir 
            # print(csv_dir+"/scenario_reset.csv")
            if os.path.isfile(csv_dir+"/sensorsim-police-gplan.csv") and os.path.isfile(csv_dir+"/scenario_reset.csv"): 
                esdf = gplan.gplan_to_df(csv_dir+"/sensorsim-police-gplan.csv", csv_dir+"/scenario_reset.csv")
                gplan.plot_run(esdf, run_n, "tab:cyan",pwp)
                self.plot_gp = False
            else:
                if "empty" in csv_dir:
                    csv_dir = "../bags/scenarios/run_2/subsample/cadrl_vel_03_empty_obs20"
                    esdf = gplan.gplan_to_df(csv_dir+"/sensorsim-police-gplan.csv", csv_dir+"/scenario_reset.csv")
                else:
                    csv_dir = "../bags/scenarios/run_2/subsample/cadrl_map1_ob20_vel_03_subsampling"
                    esdf = gplan.gplan_to_df(csv_dir+"/sensorsim-police-gplan.csv", csv_dir+"/scenario_reset.csv")
                gplan.plot_run(esdf, run_n, "tab:cyan")
                self.plot_gp = False

    def plot_collisions(self, xya, clr):
        global ax, plt_cfg, lgnd
        all_cols_x = []
        all_cols_y = []
        col_exists = False

        for run_a in xya:
            for col_xy in run_a:
                all_cols_x.append(-col_xy[1])
                all_cols_y.append(col_xy[0])

                if plt_cfg["plot_collisions"]:
                    circle = plt.Circle((-col_xy[1], col_xy[0]), 0.3, color=clr, fill = True, alpha = 0.6)
                    ax.add_patch(circle)
                    
                col_exists = True
        

        if col_exists:
            self.make_grid([all_cols_x, all_cols_y], clr)

    def evalPath(self, planner, file_name, bags):
        col_xy = []
        global ax, axlim, plt_cfg, line_clr, line_stl

        durations = [] 
        trajs = []
        vels  = []

        # self.make_txt(file_name, "\n"+"Evaluation of "+planner+":") --txt
        axlim = {}
        axlim["x_min"] =  100
        axlim["x_max"] = -100
        axlim["y_min"] =  100
        axlim["y_max"] = -100

        json_data              = {}
        json_data['run']       = []
        json_data['time']      = []
        json_data['path']      = []
        json_data['velocity']  = []
        json_data['collision'] = []

        for run in bags:
            if run != "nrun_2/":
                

                pose_x = bags[run][0]
                pose_y = bags[run][1]
                sg_x   = bags[run][4]
                sg_y   = bags[run][5]
                wp_x   = bags[run][6]
                wp_y   = bags[run][7]

                x    =  np.array(pose_x)
                y    = -np.array(pose_y)
                sg_x =  np.array(sg_x)
                sg_y = -np.array(sg_y)
                wp_x =  np.array(wp_x)
                wp_y = -np.array(wp_y)

                # print(wp_x)
                # print(wp_y)

                # x
                if min(pose_x) < axlim["x_min"]:
                    axlim["x_min"] = min(pose_x)
                if max(pose_x) > axlim["x_max"]:
                    axlim["x_max"] = max(pose_x)
                # y
                if min(pose_y) < axlim["y_min"]:
                    axlim["y_min"] = min(pose_y)
                if max(pose_y) > axlim["y_max"]:
                    axlim["y_max"] = max(pose_y)
                
                t = bags[run][2]

                dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
                path_length = np.sum(np.sqrt(dist_array)) 
                # for av
                trajs.append(path_length)
                if path_length > 0 and plt_cfg["plot_trj"]:
                    # print(lgnd)
                    ax.plot(y, x, line_clr, linestyle = line_stl, alpha=0.8)
                    ax.set_xlabel("x in [m]")
                    ax.set_ylabel("y in [m]")

                pwp = True
                if plt_cfg["plot_subgoals"]:
                    if len(wp_y) > 0 and len(wp_x) > 0:
                        pwp = False
                        ax.plot(wp_y, wp_x, "s", color='g', alpha=0.2)
                    elif len(sg_y) > 0 and len(sg_x) > 0:
                        pwp = False
                        ax.plot(sg_y, sg_x, "^", color='k', alpha=0.2)
                    

                duration = t[len(t)-1] - t[0]
                # for av
                durations.append(duration)
                av_vel = path_length/duration
                # for av
                vels.append(av_vel)

                n_col = len(bags[run][3])

                duration    = round(duration,3)
                path_length = round(path_length,3)
                av_vel      = round(av_vel,3)

                cr = run+": "+str([duration, path_length, av_vel, n_col])
                # plot global plan
                n_run = run.replace("run_","")
                n_run = int(n_run)
                if plt_cfg["plot_gp"]:
                    self.plot_global_plan(n_run,pwp)

                # append current run to txt
                # self.make_txt(file_name, "\n"+cr) -- txt

                col_xy.append(bags[run][3])

                # prepare dict for json
                json_data['run'].append(run)
                json_data['time'].append(duration)
                json_data['path'].append(path_length)
                json_data['velocity'].append(av_vel)
                json_data['collision'].append(n_col)




        msg_planner = "\n----------------------   "    + planner                               + " summary: ----------------------"
        msg_at      = "\naverage time:        "        + str(round(self.average(durations),3)) + " s"
        msg_ap      = "\naverage path length: "        + str(round(self.average(trajs),3))     + " m"
        msg_av      = "\naverage velocity:    "        + str(round(self.average(vels),3))      + "  m/s"
        msg_col     = "\ntotal number of collisions: " + str(self.nc_total)+"\n"

        print("----------------------   "+planner+"   ----------------------")
        print("average time:        ", round(self.average(durations),3), "s")
        print("average path length: ", round(self.average(trajs),3), "m")
        print("average velocity:    ", round(self.average(vels),3), " m/s")
        print("total collisions:    ",   str(self.nc_total))
        
        # average to txt (summary)
        # self.make_txt(file_name,msg_planner) -- txt
        # self.make_txt(file_name,msg_at)
        # self.make_txt(file_name,msg_ap)
        # self.make_txt(file_name,msg_av)
        # self.make_txt(file_name,msg_col)

        self.make_json(json_data)
        if plt_cfg["plot_collisions"]:
            self.plot_collisions(col_xy,line_clr)

    def fit_cluster(self,ca):

        plt.subplots(figsize=(6, 7))

        all_col_pts = []
        for arr in ca:
            for point in arr:
                all_col_pts.append(point)
        
        if len(all_col_pts) > 0:
            X = np.array(all_col_pts)
            
            cluster = AgglomerativeClustering(n_clusters=5, affinity='euclidean', linkage='ward')
            cluster.fit_predict(X)
            plt.scatter(-X[:,1],X[:,0], c=cluster.labels_, cmap='rainbow')

        plt.xlim(-16,3)
        plt.ylim(-4,24)

    def make_grid(self, acxy, clr):
        global ax, plt_cfg, grid_step

        # max grid size
        cx_min = min(acxy[0]) 
        cx_max = max(acxy[0]) 
        cy_min = min(acxy[1]) 
        cy_max = max(acxy[1]) 

        # rect p1 
        rcta_p1_x = cx_min - 0.5
        rcta_p1_y = cy_min - 0.5
        # rect p2
        rcta_p2_x = int(cx_max - rcta_p1_x) + 2
        rcta_p2_y = int(cy_max - rcta_p1_y) + 1
        # grid step even
        grid_even = False
        # dist in m
        # grid_step = 2
        # make grid step even
        while(not grid_even):
            if rcta_p2_x % grid_step > 0:
                rcta_p2_x += 1
            if rcta_p2_y % grid_step > 0:
                rcta_p2_y += 1
            if (rcta_p2_x % grid_step + rcta_p2_y % grid_step) == 0:
                grid_even = True


        n_grid_cells = (int(rcta_p2_x/grid_step), int(rcta_p2_y/grid_step))
        # grid  = np.zeros(n_grid_cells, dtype=np.ndarray)
        cells = np.zeros(n_grid_cells, dtype=np.ndarray)
        
        rows = np.shape(cells)[0]
        cols = np.shape(cells)[1]  
        
        # array iteration 
        j = 0
        i = 0
        # total cells
        n = 0
        n_cell = 0
        cells_filled = False
        # for i in range(rows):
        while not cells_filled: 

            # corner pts of each cell
            x1 = round(grid_step*i+rcta_p1_x,2)
            y1 = round(grid_step*j+rcta_p1_y,2)

            x2 = round(grid_step*i+rcta_p1_x,2)
            y2 = round(grid_step*(j+1)+rcta_p1_y,2)

            x3 = round(grid_step*(i+1)+rcta_p1_x,2)
            y3 = round(grid_step*(j+1)+rcta_p1_y,2)

            x4 = round(grid_step*(i+1)+rcta_p1_x,2)
            y4 = round(grid_step*j+rcta_p1_y,2)
            
            # add coordinates 
            n_cell += 1
            cells[i][j] = [n_cell ,x1, y1, x2, y2, x3, y3, x4, y4]


            if plt_cfg["plot_grid"]:
                # plot coordinates 
                circle = plt.Circle((x1, y1), 0.1, color=clr, fill = True, alpha = 1)
                ax.add_patch(circle)
                circle = plt.Circle((x2, y2), 0.1, color=clr, fill = True, alpha = 1)
                ax.add_patch(circle)
                circle = plt.Circle((x3, y3), 0.1, color=clr, fill = True, alpha = 1)
                ax.add_patch(circle)
                circle = plt.Circle((x4, y4), 0.1, color=clr, fill = True, alpha = 1)
                ax.add_patch(circle)

                rcta = plt.Rectangle((rcta_p1_x, rcta_p1_y), rcta_p2_x, rcta_p2_y, linewidth=2, edgecolor=clr, facecolor='none')
                ax.add_patch(rcta)
            
            i += 1
            # row completed
            if i == rows:
                n += rows
                i = 0
                j += 1
                # all cells added
                if n == rows*cols:
                    cells_filled = True

        self.find_zones(cells,acxy,clr)

    def find_zones(self, cells, acxy, clr):
        global ax, plt_cfg, grid_step
        zones = {}

        for i in range(len(acxy[0])):
            
            # collision coords
            x = acxy[0][i]
            y = acxy[1][i]

            for arr in cells:
                for cell in arr:

                    # construct cell coords
                    cell_nr = cell[0]
                    p_x1 = cell[1]
                    p_y1 = cell[2]

                    p_x2 = cell[3]
                    p_y2 = cell[4]

                    p_x3 = cell[5]
                    p_y3 = cell[6]

                    p_x4 = cell[7]
                    p_y4 = cell[8]
                    
                    # check if collision in cell
                    if p_x1 <= x and p_y1 <= y and  p_x2 <= x and p_y2 >= y and p_x3 >= x and p_y3 >= y and p_x4 >= x and p_y4 <= y:
                        if cell_nr in zones:
                            zones[cell_nr].append([x,y])
                            # average center
                            zones[str(cell_nr)+"_c"][0] += x
                            zones[str(cell_nr)+"_c"][0] /= 2

                            zones[str(cell_nr)+"_c"][1] += y
                            zones[str(cell_nr)+"_c"][1] /= 2
                        else:
                            zones[cell_nr]           = [[x,y]]
                            zones[str(cell_nr)+"_c"] = [x,y]
                        break



                    # print(i, nof_cols)
        

        print("---------------------")

        # for key in filtered_zones:
        #     print(key)

        rows = np.shape(cells)[0]

        self.merge_zones(zones, clr, rows)

    def merge_zones(self, zones, clr, rows):

        col_tol = 5
        filtered_zones = {}

        if plt_cfg["plot_zones"]:
            for i in zones:
                nof_cols = len(zones[i])
                
                if nof_cols > 0 and isinstance(i, str):
                    filtered_zones[i] = zones[i]
                    filtered_zones[i+"n"] = nof_cols

                if nof_cols >= col_tol:
                    center = zones[str(i)+"_c"]
                    radius = 0.4 + 0.1*nof_cols
                    circle = plt.Circle((center[0], center[1]), radius, color=clr, fill = False, alpha = 1, lw = 2)
                    ax.add_patch(circle)

        # print(filtered_zones)
        merged = []
        while True:
            for i in filtered_zones:

                if "c" in i and not "n" in i: 
                    center = filtered_zones[i]

                    # construct adjacent cells
                    k = int(i.replace("_c",""))

                    # adjacent cells
                    # left right
                    left_cell  = k - 1
                    right_cell = k + 1
                    # top bot
                    top_cell   = k + rows
                    bot_cell   = k - rows
                    # diagonal
                    top_right  = k + rows + 1
                    top_left   = k + rows - 1
                    bot_right  = k - rows + 1
                    bot_left   = k - rows - 1

                    # print(i, center)
                    adj_cells = [left_cell, right_cell, top_cell, bot_cell, top_right, top_left, bot_right, bot_left]

                    for ad in adj_cells:
                        key  = str(ad)+"_c"
                        # check if 
                        if key in filtered_zones:
                            ad_c = filtered_zones[key]

                            dist = math.sqrt((ad_c[0] - center[0])**2 + (ad_c[1] - center[1])**2)
                            if dist < grid_step:
                                # print(i,key)
                                # print(dist)
                                cm_x = (ad_c[0] + center[0])/2
                                cm_y = (ad_c[1] + center[1])/2

                                # print(cm_x,ad_c[0],center[0])
                                # print(cm_y,ad_c[1],center[1])
                                merged.append([cm_x, cm_y])

            break

        # if plt_cfg[plot_zones]:
        #     for i in merged:
        #         radius = 1
        #         circle = plt.Circle((i[0], i[1]), radius, color=clr, fill = False, alpha = 1, lw = 2)
        #         ax.add_patch(circle)

def fancy_print(msg,success):
    if success:
        cprint(msg + " "u'\N{check mark}', "green")
    else:
        cprint(msg, "yellow")

def plot_arrow(start,end):
    global ax
    # ax.arrow(-start[1], start[0], -end[1], end[0], head_width=0.05, head_length=0.1, fc='k', ec='k')
    plt.arrow(-start[1], start[0], -end[1], end[0],  
        head_width = 0.2, 
        width = 0, 
        ec = "black",
        fc = "black",
        ls ="-")

def plot_dyn_obst(ob_xy):
    global ax

    circle = plt.Circle((-ob_xy[1], ob_xy[0]), 0.3, color="black", fill = False, alpha = 1)
    ax.add_patch(circle)

def read_scn_file(map, ob):
    # gets start / goal of each scenario as global param
    global start, goal, plt_cfg
    # find json path
    rospack = rospkg.RosPack()
    json_path = rospack.get_path('simulator_setup')+'/scenarios/eval/'

    for file in os.listdir(json_path):
        if file.endswith(".json") and map in file and ob in file:
            jf = file
    # read file
    with open(json_path+"/"+jf, 'r') as myfile:
        data=myfile.read()
    obj = json.loads(data)

    # json to dict
    for i in obj:
        for l in obj[i]:
            data = l
    
    # get json data
    for do in data["dynamic_obstacles"]:
        sp   = data["dynamic_obstacles"][do]["start_pos"]
        sp_x = sp[0]
        sp_y = sp[1]

        wp   = data["dynamic_obstacles"][do]["waypoints"][0]
        wp_x = wp[0]
        wp_y = wp[1]
        
        ep_x = sp_x + wp_x
        ep_y = sp_y + wp_y
        ep   = [ep_x, ep_y]

        if plt_cfg["plot_obst"]:
            plot_dyn_obst(sp)
            plot_dyn_obst(ep)
            plot_arrow(sp,wp)

        
    start = data["robot"]["start_pos"]
    goal  = data["robot"]["goal_pos"]

def eval_cfg(cfg_file, filetype):
    global ax, sm, start, goal, axlim, plt_cfg, line_clr, line_stl

    cur_path    = str(pathlib.Path().absolute()) 
    parent_path = str(os.path.abspath(os.path.join(cur_path, os.pardir)))
    
    fancy_print("loading config: " + cfg_file, 0)
    # load default config
    with open(cfg_file, "r") as ymlfile:
        cfg = yaml.safe_load(ymlfile)
    default_cfg = cfg["default_cfg"]
    fancy_print("loading config: " + cfg_file, 1)



    plt_cfg  = copy.deepcopy(default_cfg)
  
    for curr_figure in cfg:
        # plot file name
        cfg_folder = cfg_file
        # cfg_folder = cfg_folder.replace(".yaml","")
        if not os.path.exists('../plots/' + cfg_folder):
            os.mkdir('../plots/' + cfg_folder)

        plot_file        = '../plots/' + cfg_folder + "/" + curr_figure + "." + filetype 
        # plot_file_exists = os.path.isfile(plot_file)
        plot_file_exists = False
        # print(curr_figure)
        if "custom_cfg" in curr_figure:
            for param in cfg[curr_figure]:
                plt_cfg[param] = cfg[curr_figure][param]
            # print("----------------")
            # pp.pprint(plt_cfg)
        
        elif "default" not in curr_figure and not plot_file_exists:
            fig, ax  = plt.subplots(figsize=(6, 7))

            ca = curr_figure.split("_")
            map  = ca[0]
            ob   = ca[1]
            vel  = ca[2]
            read_scn_file(map, ob) 
            mode =  map + "_" + ob + "_" + vel 
            fig.canvas.set_window_title(curr_figure)

            legend_elements = []
            if plt_cfg["plot_gp"]:
                gp_el = Line2D([0], [0], color="tab:cyan", lw=4, label="Global Plan")
                legend_elements.append(gp_el)


            if not "empty" in map and plt_cfg["plot_sm"]:
                # offs_x = cfg[curr_figure]["map_origin"][0]
                # offs_y = cfg[curr_figure]["map_origin"][1]
                plt.scatter(sm[1], sm[0],s = 0.2 , c = "grey")

            for planner in cfg[curr_figure]["planner"]:
                # config plot param for planner
                plot_param  = cfg[curr_figure]["planner"][planner]
                if "folder" in plot_param:
                    dir = plot_param["folder"]
                else:
                    dir = plt_cfg["folder"]
                if "wpg" in plot_param:
                    wpg     = plot_param["wpg"]
                else:
                    wpg = ""

                model   = plot_param["model"]
                style   = plot_param["linestyle"]
    

                # print(planner, dir, model, style, wpg)

                bag_path = parent_path + "/bags/scenarios/" + dir
                curr_bag = bag_path + model


                style_arr = style.split(",")
                line_clr  = style_arr[0]
                line_stl  = style_arr[1]
                el        = Line2D([0], [0], color=line_clr, lw=4, label=planner, marker = "", linestyle=line_stl)
                legend_elements.append(el)

                for file in os.listdir(curr_bag):

                    # check if file matches ids

                    file_match = map in file and \
                                 ob  in file and \
                                 vel in file and \
                                 wpg in file

                    if file.endswith(".bag") and file_match:

                        fancy_print("Evaluate bag: " + file, 0)
                        planner_wpg = planner.split("_")[0] + "wpg" + wpg 
                        newBag(planner_wpg, curr_figure, curr_bag + "/" + file)
                        fancy_print("Evaluate bag: " + file, 1)

            
            #map0: lower left, empty: upper left, open: upper left
            ax.legend(handles=legend_elements, loc="lower left")

            ax.spines["right"].set_visible(True)
            color_name = "grey"
            ax.spines["top"].set_color(color_name)
            ax.spines["bottom"].set_color(color_name)
            ax.spines["left"].set_color(color_name)
            ax.spines["right"].set_color(color_name)

            #plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
            #plt.title("Trajectories on {0}".format(map) , fontweight='bold', fontsize=16)

            plt.savefig(plot_file, bbox_inches = 'tight', pad_inches = 0.04,  fontsize=24)

            # reset plot cfg to default
            plt_cfg = copy.deepcopy(default_cfg)

    plt.show()

def getMap(msg):
    global ax, sm, map_orig

    map_orig = [0, 0]
    points_x = []
    points_y = []
    # print(msg.markers[0]) map0 -16.6 -6.65  ,  map1 empty: -6 -6  , open field: 0 0
    orig_x = -16.6
    orig_y = -6.65
    for p in msg.markers[0].points:
    #     if  2 < p.y < 25 :
        points_x.append( p.x + orig_x)
        points_y.append(-p.y - orig_y)



    # plt.scatter(points_y, points_x)
    sm = [points_x, points_y]

def run(cfg_file, filetype):
    global ax, sm, grid_step, select_run
    global plt_cfg
    plt_cfg = {}

    select_run = []

    grid_step  = 2
        
    # static map
    rospy.init_node("eval", disable_signals=True)
    rospy.Subscriber('/flatland_server/debug/layer/static',MarkerArray, getMap)
    

    # eval_cfg("eval_run3_empty.yml")
    # eval_cfg("eval_run3_map1.yml")
    # eval_cfg("eval_test.yml")
    fancy_print("Start Evaluation: " + cfg_file, 0)
    eval_cfg(cfg_file, filetype)
    fancy_print("Evaluation finished: " + cfg_file, 1)



    rospy.spin()

if __name__=="__main__":

    try:
        yml_file = sys.argv[1]
        if len(sys.argv) > 2:
            filetype = sys.argv[2]
        else:
            filetype = "png"
    except Exception as e:
        cprint(e, 'red')
        cprint("\nCall this script like this: python scenario_eval.py '$config.yml' '$format'", 'red')
        cprint("Example:  python scenario_eval.py 'test.yml' 'pdf'", 'green')
        cprint("This will generate figures defined in test.yml as pdf files.", 'red')
        cprint("If $format is left empty, output files will default to png.\n", 'red')
        # cprint('Hello, World!', 'red')
    run(yml_file, filetype)
