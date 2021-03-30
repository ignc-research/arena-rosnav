# for data
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
matplotlib.rcParams.update({'font.size': 18})
# 
class newBag():
    def __init__(self, planner, file_name, bag_name):
        # planner
        self.planner = planner
        self.file_name = file_name
        # csv dir
        self.bag_name = bag_name
        self.csv_dir = bag_name.replace(".bag","")
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
        self.evalPath(planner,file_name,eps)


    def make_json(self, data):
        fn = self.file_name
        fa = fn.split("_")

        # adjust file name
        if "0" not in fa[2]:
            fa[2] = "0" + fa[2]
        fn = fa[0] + "_" + fa[1] + "_" + "obs" + fa[2] + "_" + fa[3].replace("_","") + fa[4]

        with open("quantitative/" + fn + ".json", 'w') as outfile:
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
        df_odom = pd.read_csv(odom_csv, error_bad_lines=False)


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
            df_gp    = pd.read_csv(gp, error_bad_lines=False)

            wpg      = self.bag.message_by_topic(self.wpg_topic)
            df_wpg   = pd.read_csv(wpg, error_bad_lines=False)

            # print(df_goal)


        except Exception as e:
            # otherwise run had zero collisions
            print(e)


        t_col = []
   

        for i in range(len(df_collision)): 
            t_col.append(df_collision.loc[i, "Time"])   
            
        self.nc_total = len(t_col)
        # get reset time
        reset_csv   = self.bag.message_by_topic("/scenario_reset")
        df_reset    = pd.read_csv(reset_csv, error_bad_lines=False)
        t_reset     = []
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

            if current_time > reset-6 and n < len(t_reset)-1 and x < start_x:
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
   
            if n+1 in select_run or len(select_run) == 0:

                if  len(pose_x) > 0:
                    pose_x.append(x)
                    pose_y.append(y)
                elif x < start_x:
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


        # remove first 
        if "run_1" in bags:    
            bags.pop("run_1")

        df = pd.DataFrame(data=bags)
        run_csv = self.csv_dir + "/" + self.csv_dir.rsplit('/', 1)[-1] + ".csv"
        df.to_csv(run_csv,index=False)   
        print("csv created in: " + self.csv_dir)    
        return bags
    
    def average(self,lst): 
        if len(lst)>0:
            return sum(lst) / len(lst) 
        else:
             return 0

    def plot_global_plan(self,run_n,pwp):
        global plot_gp

        if plot_gp and self.plot_gp:
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
        global ax, plot_collisions, lgnd
        all_cols_x = []
        all_cols_y = []
        col_exists = False

        for run_a in xya:
            for col_xy in run_a:
                all_cols_x.append(-col_xy[1])
                all_cols_y.append(col_xy[0])

                if plot_collisions:
                    circle = plt.Circle((-col_xy[1], col_xy[0]), 0.3, color=clr, fill = True, alpha = 0.3)
                    ax.add_patch(circle)
                    
                col_exists = True
        

        if col_exists:
            self.make_grid([all_cols_x, all_cols_y], clr)

    def evalPath(self, planner, file_name, bags):
        col_xy = []
        global ax, lgnd, axlim, plot_trj, plot_subgoals, plot_gp, plot_collisions

        durations = [] 
        trajs = []
        vels  = []

        # self.make_txt(file_name, "\n"+"Evaluation of "+planner+":") --txt
        axlim = {}
        axlim["x_min"] = 100
        axlim["x_max"] = -100
        axlim["y_min"] = 100
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
                if path_length > 0 and plot_trj:
                    print(lgnd)
                    ax.plot(y, x, lgnd[planner], alpha=0.2)
                    ax.set_xlabel("x in [m]")
                    ax.set_ylabel("y in [m]")

                pwp = True
                if plot_subgoals:
                    if len(wp_y) > 0 and len(wp_x) > 0:
                        pwp = False
                        ax.plot(wp_y, wp_x, "s", color='g', alpha=0.1)
                    elif len(sg_y) > 0 and len(sg_x) > 0:
                        pwp = False
                        ax.plot(sg_y, sg_x, "^", color='k', alpha=0.1)
                    

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
                if plot_gp:
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

        # self.make_json(json_data)
        if plot_collisions:
            self.plot_collisions(col_xy,lgnd[planner])

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
        global ax, plot_grid, grid_step

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


            if plot_grid:
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
        global ax, plot_zones, grid_step
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
                        #print(cell_nr)
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

        if plot_zones:
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

        # if plot_zones:
        #     for i in merged:
        #         radius = 1
        #         circle = plt.Circle((i[0], i[1]), radius, color=clr, fill = False, alpha = 1, lw = 2)
        #         ax.add_patch(circle)


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
    global start, goal, plot_obst
    # find json path
    rospack = rospkg.RosPack()
    json_path = rospack.get_path('simulator_setup')+'/scenarios/eval/'

    for file in os.listdir(json_path):
        print(map, ob)
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

        if plot_obst:
            plot_dyn_obst(sp)
            plot_dyn_obst(ep)
            plot_arrow(sp,wp)

        
    start = data["robot"]["start_pos"]
    goal  = data["robot"]["goal_pos"]

# def eval_all(a, map, ob, vel, wpg, run, saved_name=""):
#     global ax, sm, lgnd, start, goal, axlim, plot_sm
#     fig, ax = plt.subplots(figsize=(6, 7))
    
#     read_scn_file(map, ob) 

#     mode =  map + "_" + ob + "_" + vel 
#     # fig.suptitle(mode, fontsize=16)
#     # plot static map
#     if not "empty" in map and plot_sm:
#         # img = plt.imread("map_small.png")
#         # ax.imshow(img, extent=[-20, 6, -6, 27.3])
#         plt.scatter(sm[1], sm[0],s = 0.2 , c = "grey")
#         # plt.plot(sm[1], sm[0],"--")
        
#     # return
#     cur_path    = str(pathlib.Path().absolute()) 
#     parent_path = str(os.path.abspath(os.path.join(cur_path, os.pardir)))
#     bag_path    = parent_path + "/bags/scenarios/" + run

def eval_cfg():
    global ax, sm, lgnd, start, goal, axlim, plot_sm, plt_cfg

    cur_path    = str(pathlib.Path().absolute()) 
    parent_path = str(os.path.abspath(os.path.join(cur_path, os.pardir)))
    

    with open("eval.yml", "r") as ymlfile:
        cfg = yaml.safe_load(ymlfile)
    plt_cfg = cfg["plt_default"]
    print(plt_cfg)
    # print(cfg["map1_obs20_vel02"]["plt_selector"])

    # for f in cfg:
    #     fig, ax  = plt.subplots(figsize=(6, 7))
    #     ca = f.split("_")
    #     map  = ca[0]
    #     ob   = ca[1]
    #     vel  = ca[2]
    #     read_scn_file(map, ob) 
    #     mode =  map + "_" + ob + "_" + vel 
    #     fig.canvas.set_window_title(mode)

        
    #     planner  = cfg[f]
    #     for ids in planner:
    #         if planner[ids] != None:
    #             dir  = planner[ids][0]
    #             # lgnd = planner[ids][1]
    #             wpg  = planner[ids][2]


    #             bag_path = parent_path + "/bags/scenarios/" + dir
    #             curr_bag = bag_path + ids
    #             for file in os.listdir(curr_bag):
    #                 if file.endswith(".bag") and map in file and ob in file and vel in file and wpg in file:
    #                     fn = ids + "_" + mode
    #                     # if "subsample" not in fn or "esdf" not in fn:
    #                     #     fn.replace("02","03")
    #                     print(file, fn)
                        
    #                     newBag(ids, fn, curr_bag + "/" + file)







    # # fig.suptitle(mode, fontsize=16)
    # # plot static map
    # if not "empty" in map and plot_sm:
    #     # img = plt.imread("map_small.png")
    #     # ax.imshow(img, extent=[-20, 6, -6, 27.3])
    #     plt.scatter(sm[1], sm[0],s = 0.2 , c = "grey")
    #     # plt.plot(sm[1], sm[0],"--")
        
    # # return
    # cur_path    = str(pathlib.Path().absolute()) 
    # parent_path = str(os.path.abspath(os.path.join(cur_path, os.pardir)))
    # bag_path    = parent_path + "/bags/scenarios/" + run




    #             # print(fn)
    
    # # dhow legend labels once per planner
    
    # legend_elements = []
    # # el = Line2D([0], [0], color="tab:cyan", lw=4, label="Global Plan")
    # # legend_elements.append(el)
    # for l in lgnd:
    #         clr = lgnd[l]
    #         mrk = ""
    #         # if l == "cadrl":
    #         #     l = "STH-WP"
    #         # if l == "esdf":
    #         #     l = "LM-WP"
    #         # if l == "subsample":
    #         #     l = "SUB-WP"
    #         if "_" in clr:
    #             clr = lgnd[l].split("_")[1]
    #             mrk = lgnd[l].split("_")[0]
    #             el = Line2D([0], [0], color=clr, lw=4, label=l, marker = mrk, linestyle='None')
    #         else:
    #             el = Line2D([0], [0], color=clr, lw=4, label=l, marker = mrk)
    #         legend_elements.append(el)
    # ax.legend(handles=legend_elements, loc=0)

    plt.show()


def getMap(msg):
    global ax, sm
    points_x = []
    points_y = []
    # print(msg.markers[0])
    for p in msg.markers[0].points:
        if  2 < p.y < 25 :
            points_x.append(p.x-6)
            points_y.append(-p.y+6)
    # plt.scatter(points_y, points_x)
    sm = [points_x, points_y]

def run():
    global ax, sm, lgnd, grid_step, select_run
    global plt_cfg, plot_trj, plot_zones, plot_obst, plot_collisions, plot_grid, plot_sm, plot_gp, plot_subgoals
    plt_cfg = {}

    select_run = []
    # ToDo: merge nearby zones 
    # legend

    # plots
    grid_step       = 2
    plot_sm         = False
    plot_obst       = True
    plot_trj        = True
    plot_zones      = True
    plot_collisions = True
    plot_grid       = False
    plot_gp         = False
    # static map
    # rospy.init_node("eval", anonymous=False)
    # rospy.Subscriber('/flatland_server/debug/layer/static',MarkerArray, getMap)
    

    # 13 - 18

    select_run = []
    plot_zones    = True
    plot_subgoals = False

    lgnd          = {}
    lgnd["cadrl"] = "tab:red"
    lgnd["teb"]   = "tab:orange"
    lgnd["mpc"]   = "tab:green"

    # lgnd["esdf"] = "tab:blue"
    # lgnd["subsample"] = "tab:purple"


    # all runs ------------------------------
    # print(lgnd)
    planner = ["cadrl"]

    # map 1 
    # eval_all(planner, "map1", "obs20", "vel02", "timespace", "run_3/")
    eval_cfg()



    # rospy.spin()

if __name__=="__main__":
    run()
