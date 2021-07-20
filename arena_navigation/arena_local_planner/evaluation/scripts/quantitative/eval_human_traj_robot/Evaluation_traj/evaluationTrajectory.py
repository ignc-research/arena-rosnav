# for data
import sys
import copy
import pprint as pp
import bagpy
from bagpy import bagreader
import pandas as pd
import json
#import rospkg
import yaml
# for plots
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.pyplot import figure, gray
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
#import gplan_analysis as gplan
matplotlib.rcParams.update({'font.size': 15})
# 
from termcolor import colored, cprint

class newBag():
    def __init__(self, file_name, bag_name,colorCircle,colorTraj,lineStyle): # planner,
        # planner

        self.file_name       = file_name
        self.color_circle=colorCircle
        self.color_traj=colorTraj
        self.line_stl=lineStyle
        # csv dir
        self.csv_dir         = bag_name.replace(".bag","")
        # bag topics
        self.odom_topic      = "/police/odom"         
        self.collision_topic = "/police/collision"
        self.subgoal_topic   = "/police/subgoal"
        self.circle1=None
        self.l_1=None

        self.color=np.array([1.0,0.4,0.7]).reshape(-1,3)
        th = np.linspace(0, 1, 200).reshape(200,-1)[::-1]
        self.color=th.dot(self.color)
        # print(self.color)

        self.nc_total = 0
        # eval bags
        self.bag = bagreader(bag_name)
        eps = self.split_runs()
        if len(eps) != 0:
            self.evalPath(file_name, eps) #self.planner,
        else:
            print("no resets for: " + bag_name)
        # return circle1


    def make_json(self, data):
        fn = self.file_name
        # fa = fn.split("_")
        # fn = fa[0] +"_"+ fa[1] + "_" + fa[2] + "_" + self.wpg

        jfile = "quantitative" + "_" + fn + ".json"
        if not os.path.isfile(jfile): 
            with open(jfile, 'w') as outfile:
                json.dump(data, outfile, indent=2)


    def split_runs(self):
        # get odometry
        odom_csv = self.bag.message_by_topic(self.odom_topic)
        df_odom  = pd.read_csv(odom_csv, error_bad_lines=False)
        df_collision = []

        # get topics
        try:
            # check if collision was published
            collision_csv = self.bag.message_by_topic(self.collision_topic)
            df_collision  = pd.read_csv(collision_csv, error_bad_lines=False)


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


            pose_x = [-2]
            pose_y = [8]
            t = [0.0]

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
            select_run=[10] #die reihenfolge should be descend

            for i in range(len(df_odom)): 
                current_time = df_odom.loc[i, "Time"]
                x = df_odom.loc[i, "pose.pose.position.x"]
                x = round(x,2)
                y = df_odom.loc[i, "pose.pose.position.y"]
                y = round(y,2)
                reset = t_reset[n]
                
                start_x = start[0] + 0.5

                dist2_oldp = 0
                if old_x != None:
                    dist2_oldp = math.sqrt((x-old_x)**2+(y-old_y)**2)

                # if current_time > reset-6 and n < len(t_reset)-1 and x < start_x:
                # if current_time > reset and n < len(t_reset)-1:
                dis_rt = 5
                # print('reach here')
                if dist2_oldp > dis_rt and n < len(t_reset)-1:
                    n += 1

                    if n == select_run[-1]:
                        select_run.pop()
                        bags["run_"+str(n)] = [pose_x, pose_y, t, col_xy] #, subgoal_x, subgoal_y, wpg_x, wpg_y

                    pose_x    = [-2]
                    pose_y    = [8]
                    t         = [0.0]

                    col_xy    = []


                    old_x = None
                    old_y = None

                # if n+1 in select_run or len(select_run) == 0 and dist2_oldp < dis_rt:
                if len(select_run)!=0 and dist2_oldp < dis_rt:
                  
                    pose_x.append(x)
                    pose_y.append(y)

                    t.append(current_time)
                    # get trajectory

                    # check for col
                    if len(t_col) > nc:
                        if current_time >= t_col[nc]:
                            col_xy.append([x,y])
                            nc += 1

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

    def evalPath(self, file_name, bags): # planner,
        col_xy = []
        legend_traj=False

        global ax, axlim, plt_cfg, line_clr, line_stl,l1,circle1

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

                x    =  np.array(pose_x)
                y    = np.array(pose_y)

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
                t_1=t[1]
                t_rate_tail=1.0-(t[1:]-t_1)/(t[-1]-t_1)
                t_rate=[1.0]
                # print(len(t_rate_tail))
                t_rate.extend(t_rate_tail)
                trajs.append(path_length)
                # print('pl',path_length)
                # print(len(t_rate))
                t[1:]=t[1:]-t_1+0.5
                # t
                # print(len(t))
                if path_length > 0 and plt_cfg["plot_trj"]:
                    # print(lgnd)
                    # print('reached here')
                    # print(len(x))
                    # print(len(y))
                    self.l_1=ax.plot(x, y, self.color_traj, linestyle = self.line_stl, alpha=0.8)
                    plt.grid()
                    for i,t_e in enumerate(t):
                        circle_outer = plt.Circle((x[i], y[i]), 0.65, color=self.color_traj, fill = True, alpha = t_rate[i])
                        circle = plt.Circle((x[i], y[i]), 0.65, color=self.color_traj, fill = False,alpha = 0.8)
                        plt.text(x[i]-0.6,y[i],f'{round(t_e,1)}',fontsize=10, alpha = 1,fontweight= 1000, color="k")
                        ax.add_patch(circle_outer)
                        ax.add_patch(circle)
                    # if ~legend_traj:
                    #     plt.legend([l1[0],circle],['traj','robot'])
                    #     legend_traj=True
                    self.circle1=circle
                    
                    #ax.set_title("Comparison of Trajectories")
                    ax.set_xlabel("x in [m]")
                    ax.set_ylabel("y in [m]")

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

                col_xy.append(bags[run][3])

                # prepare dict for json
                json_data['run'].append(run)
                json_data['time'].append(duration)
                json_data['path'].append(path_length)
                json_data['velocity'].append(av_vel)
                json_data['collision'].append(n_col)




        msg_planner = "\n----------------------   "    + " summary: ----------------------"
        msg_at      = "\naverage time:        "        + str(round(self.average(durations),3)) + " s"
        msg_ap      = "\naverage path length: "        + str(round(self.average(trajs),3))     + " m"
        msg_av      = "\naverage velocity:    "        + str(round(self.average(vels),3))      + "  m/s"
        msg_col     = "\ntotal number of collisions: " + str(self.nc_total)+"\n"

        print("----------------------      ----------------------")
        print("average time:        ", round(self.average(durations),3), "s")
        print("average path length: ", round(self.average(trajs),3), "m")
        print("average velocity:    ", round(self.average(vels),3), " m/s")
        print("total collisions:    ",   str(self.nc_total))

        self.make_json(json_data)
        if plt_cfg["plot_collisions"]:
            self.plot_collisions(col_xy,line_clr)
    
        # self.circle1=circle1
        # self.l_1=l1

    def getCircleLegend(self):
        return self.circle1

    def getLineLegend(self):
        return self.l_1



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

def read_scn_file():
    # gets start / goal of each scenario as global param
    global start, goal        
    start = [0,0]
    goal  = [10,10]

def eval_run(filetype):
    global ax, sm, start, goal, axlim, plt_cfg, line_clr, line_stl

    cur_path    = str(pathlib.Path().absolute())

    plt_cfg={}
    plt_cfg['plot_collisions']=True
    plt_cfg['plot_trj']=True
  
    # for curr_figure in cfg:
    # plot file name
    cfg_folder = 'traj'


    plot_file        = 'plots_' + cfg_folder + "." + filetype

    fig, ax  = plt.subplots(figsize=(6, 7))


    read_scn_file()
    #name of the fig
    curr_figure='fooooooooooo'
    # mode =  map + "_" + ob + "_" + vel 
    fig.canvas.set_window_title(curr_figure)

    legend_elements = []

    style   = "tab:orange,--"


    # print(planner, dir, model, style, wpg)

    bag_path = cur_path
    # curr_bag = bag_path + model


    style_arr = style.split(",")
    line_clr  = style_arr[0]
    line_stl  = style_arr[1]

    # file=['HUMAN_88888888888.bag','HUMAN_normal_zone.bag','HUMAN_danger_zone1.bag'] #scenario1
    #scenario files 
    #raw  nz  dz
    file=['HUMAN_vh_raw.bag','HUMAN_vh_nz.bag','HUMAN_vh_dz.bag'] #scenario2
    color=['tab:blue','tab:green','tab:red']
    line_style=['--','-.','-']
    circles_traj_legend=[]
    line_traj_legend=[]
    for k,f in enumerate(file):
        # print(k)
        nb=newBag(curr_figure, bag_path + "/" + f, 'tab:red', color[k], line_style[k])
        circles_traj_legend.append(nb.getCircleLegend())
        line_traj_legend.append(nb.getLineLegend()[0])

    # file_human = 'HUMAN_2021-07-04-00-25-52.bag' #scenario1
    file_human = 'HUMAN_vh.bag'

    bag_human = bagreader(file_human)


    num_humans      = 10
    ns_prefix='eval_sim/'
    human_odom_topic_list=[]
    for i in range(num_humans):
        human_odom_topic_list.append(f'/police/pedsim_agent_{i+1}/agent_state')
    
    # human_odom_csv=[]
    df_human_odom=[]
    delete_idx=[8]
    for i in range(num_humans):
        # if i in delete_idx:
        #     continue
        human_odom_csv = bag_human.message_by_topic(human_odom_topic_list[i])
        df_human = pd.read_csv(human_odom_csv, error_bad_lines=False)
        df_human_odom.append(df_human)
    
    t_0_h=None

    t_h=[]
    x_h=[]
    y_h=[]
    t_rate_h=[]
    color=None
    circle2=None
    circle3=None
    circle4=None

    for i in range(num_humans):
        if i in delete_idx:
            continue

        for j in range(len(df_human_odom[i])):
            current_time = df_human_odom[i].loc[j, "Time"]
            if t_0_h==None:
                t_0_h=current_time
            if t_0_h!=None:
                current_time-=t_0_h
            current_time=round(current_time,1)
            x = df_human_odom[i].loc[j, "pose.position.x"]
            x = round(x,2)
            y = df_human_odom[i].loc[j, "pose.position.y"]
            y = round(y,2)
            ty=df_human_odom[i].loc[j,'type']
            # print(f'the {i} th human at time of {j} ',ty)

            t_h.append(current_time)            
            x_h.append(x)
            y_h.append(y)
            # print(x)
            # print(y)
            # reset = t_reset[n]
        # print(x_h)
        # ax.plot(y_h, x_h, line_clr, linestyle = line_stl, alpha=0.8)
        if ty==0:
            color1='tab:orange'
        elif ty==1:
            color1='tab:purple'
        else:
            color1='tab:pink'
        # print(i+1)
        if(t_h[0]<0):
            t_h+=-t_h[0]
        #here I plot the trajectory of one spefic human
        for k,t_e in enumerate(t_h):
            t_rate=t_e/t_h[-1]
            circle = plt.Rectangle((x_h[k], y_h[k]),0.42, 0.42, color=color1, alpha = 1.0-t_rate) #,edgecolor=color1,ec=color1,
            if k%3==0:
                plt.text(x_h[k]+0.6,y_h[k],f'{t_e}',fontsize=10, fontweight=1000, color=color1, alpha=1)
            plt.gca().add_patch(circle)
            if ty==0:
                if circle2==None:
                    circle2=circle
                else:
                    continue
            elif ty==1:
                if circle3==None:
                    circle3=circle
                else:
                    continue
            else:
                if circle4==None:
                    circle4=circle
                else:
                    continue
        
        t_h=[]
        x_h=[]
        t_rate_h=[]
        y_h=[]

    plt.legend(line_traj_legend  + [circle2,circle3,circle4],['Raw','Static Zone','Dyn. Zone','Adult','Child','Elder'],framealpha=0.4,fontsize=15,loc='upper left')

    ax.spines["right"].set_visible(True)
    color_name = "grey"
    ax.spines["top"].set_color(color_name)
    ax.spines["bottom"].set_color(color_name)
    ax.spines["left"].set_color(color_name)
    ax.spines["right"].set_color(color_name)


    plt.savefig(plot_file, bbox_inches = 'tight', pad_inches = 0.04,  fontsize=24)


if __name__=="__main__":

    try:
        # filetype = sys.argv[1]
        if len(sys.argv) > 1:
            filetype = sys.argv[1]
        else:
            filetype = "png"
    except Exception as e:
        cprint(e, 'red')
        cprint("\nCall this script like this: python scenario_eval.py '$config.yml' '$format'", 'red')
        cprint("Example:  python scenario_eval.py 'test.yml' 'pdf'", 'green')
        cprint("This will generate figures defined in test.yml as pdf files.", 'red')
        cprint("If $format is left empty, output files will default to png.\n", 'red')
        # cprint('Hello, World!', 'red')
    eval_run(filetype)
