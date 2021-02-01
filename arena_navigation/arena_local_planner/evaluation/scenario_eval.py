# 
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.pyplot import figure
import seaborn as sb
import rospy 
from visualization_msgs.msg import Marker, MarkerArray
import glob
import pathlib
import os
# 
class newBag():
    def __init__(self, planner,file_name, plot_style, bag_name, odom_topic="/sensorsim/police/odom", collision_topic="/sensorsim/police/collision"):
        # self.make_txt(file_name,"","w")
        self.bag = bagreader(bag_name)
        eps = self.split_runs(odom_topic, collision_topic)
        self.evalPath(plot_style,planner,file_name,eps)
        self.nc_total = 0

    def make_txt(self,file,msg,ron="a"):
        # f = open(file, ron)
        # f.write(msg)
        # f.close()
        return

    def make_heat_map(self,xy):
        plt.figure()
        data = np.zeros((25, 20))

        for xya in xy:
            for pos in xya:
                y = -int(round(pos[0], 0))
                x = int(round(pos[1], 0))
                data[x,y] += 1
                # print(data[x,y])
        heat_map = sb.heatmap(data,  cmap="YlGnBu")
        heat_map.invert_yaxis()
        plt.show()

    def split_runs(self,odom_topic,collision_topic):
        # get odometry
        
        odom_csv = self.bag.message_by_topic(odom_topic)
        df_odom = pd.read_csv(odom_csv, error_bad_lines=False)

        # get collision time
        try:
            collision_csv = self.bag.message_by_topic(collision_topic)
            df_collision = pd.read_csv(collision_csv, error_bad_lines=False)
        except Exception as e:
            df_collision = []
        t_col = []
        for i in range(len(df_collision)): 
            t_col.append(df_collision.loc[i, "Time"])
        self.nc_total = len(t_col)
        print("total #n of collisions: ",len(t_col))
        # get reset time
        reset_csv = self.bag.message_by_topic("/scenario_reset")
        df_reset = pd.read_csv(reset_csv, error_bad_lines=False)
        t_reset = []
        for i in range(len(df_reset)): 
            t_reset.append(df_reset.loc[i, "Time"])
        print("resets: ",len(t_reset))
        # print(t_reset)

        pose_x = []
        pose_y = []
        t = []
        bags = {}
        # run idx
        n = 0
        # collsion pos
        col_xy = []
        nc = 0


        for i in range(len(df_odom)): 
            current_time = df_odom.loc[i, "Time"]
            x = df_odom.loc[i, "pose.pose.position.x"]
            y = df_odom.loc[i, "pose.pose.position.y"]
            reset = t_reset[n]
            # check if respawned
            # if current_time > reset-6 and n < len(t_reset)-1 and x<0:
            global start_x
            if current_time > reset-6 and n < len(t_reset)-1 and x < start_x:
                n += 1
                # store the run
                bags["run_"+str(n)] = [pose_x,pose_y,t,col_xy]

                # reset 
                pose_x = []
                pose_y = []
                t = []
                col_xy = []
   
            if  len(pose_x) > 0:
                pose_x.append(x)
                pose_y.append(y)
            elif x < start_x:
                # print("run_"+str(n))
                # print("x:",x,"y",y)
                pose_x.append(x)
                pose_y.append(y)

            t.append(current_time)
            # get trajectory

            # check for col
            if len(t_col) > nc:
                if current_time >= t_col[nc]:
                    col_xy.append([x,y])
                    nc += 1

        if "run_1" in bags:    
            bags.pop("run_1")
        return bags
    
    def average(self,lst): 
        if len(lst)>0:
            return sum(lst) / len(lst) 
        else:
             return 0

    def print_patches(self,xya,clr):
        global ax

        for run_a in xya:
            for col_xy in run_a:
                circle = plt.Circle((-col_xy[1], col_xy[0]), 0.3, color=clr, fill = False)
                ax.add_patch(circle)

    def evalPath(self, clr, planner, file_name, bags = None):
        col_xy = []
        global ax
        # circle1 = plt.Circle((-9, 12), 0.3, color='r', fill = False)
        # ax.add_patch(circle1)
        durations = [] 
        trajs = []
        vels  = []

        self.make_txt(file_name, "\n"+"Evaluation of "+planner+":")
        
        for run in bags:
            if run != "nrun_30":
                pose_x = bags[run][0]
                pose_y = bags[run][1]

                x = np.array(pose_x)
                y = -np.array(pose_y)
                
                t = bags[run][2]

                dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
                path_length = np.sum(np.sqrt(dist_array)) 
                # for av
                trajs.append(path_length)
                if path_length > 0:
                    ax.plot(y,x, clr, label='Cosine wave',alpha=0.2)

                duration = t[len(t)-1] - t[0]
                # for av
                durations.append(duration)
                av_vel = path_length/duration
                # for av
                vels.append(av_vel)

                n_col = len(bags[run][3])

                cr = run+": "+str([duration, path_length, av_vel, n_col])
                print(cr)
                self.make_txt(file_name, "\n"+cr)

                col_xy.append(bags[run][3])


        msg_planner = "\n----------------------   "+planner+" summary: ----------------------"
        msg_at = "\naverage time:        "+str(round(self.average(durations),3))+" s"
        msg_ap = "\naverage path length: "+str(round(self.average(trajs),3))+" m"
        msg_av = "\naverage velocity:    "+str(round(self.average(vels),3))+"  m/s"
        msg_col = "\ntotal number of collisions: "+str(self.nc_total)+"\n"

        print("----------------------   "+planner+"   ----------------------")
        print("average time:        ", round(self.average(durations),3), "s")
        print("average path length: ", round(self.average(trajs),3), " m")
        print("average velocity:    ", round(self.average(vels),3), "  m/s")

        self.make_txt(file_name,msg_planner)
        self.make_txt(file_name,msg_at)
        self.make_txt(file_name,msg_ap)
        self.make_txt(file_name,msg_av)
        self.make_txt(file_name,msg_col)

        self.print_patches(col_xy,clr)
        # self.make_heat_map(col_xy)

def eval_all(a,map,ob,vel):
    global ax, start_x, sm

    fig, ax = plt.subplots(figsize=(6, 7))
    mode =  map + "_" + ob + "_" + vel 
    fig.suptitle(mode, fontsize=16)
    if not "empty" in map:
        plt.scatter(sm[1], sm[0])
    plt.xlim((-18, 4.5))
    plt.ylim((-4.5, 25))

    cur_path = str(pathlib.Path().absolute())
    # print(cur_path)
    for planner in a:
        for file in os.listdir(cur_path+"/bags/scenarios/"+planner):
            if file.endswith(".bag") and map in file and ob in file and vel in file:
                print("bags/scenarios/"+planner+"/"+file)
                fn = planner + mode
                if planner == "arena":
                    clr = "g"
                if planner == "cadrl":
                    clr = "b"
                if planner == "dwa":
                    clr = "k"
                if planner == "mpc":
                    clr = "purple"
                if planner == "teb":
                    clr = "r"
                    
                newBag(planner, fn, clr, "bags/scenarios/"+planner+"/"+file)
    plt.savefig(mode+'.pdf')

def getMap(msg):
    global ax, sm
    points_x = []
    points_y = []
    # print(msg.markers[0])
    for p in msg.markers[0].points:
        points_x.append(p.x-6)
        points_y.append(-p.y+6)
    # plt.scatter(points_y, points_x)
    sm = [points_x, points_y]
    # plt.show()

def run():
    global ax, start_x, sm
    # static map
    rospy.init_node("eval", anonymous=False)
    rospy.Subscriber('/flatland_server/debug/layer/static',MarkerArray, getMap)
    
    start_x = 0.5
    # map
    #  5 01
    eval_all(["arena","cadrl","dwa","mpc","teb"],"map1","5","vel_01.")
    start_x = 0
    #  10 01
    eval_all(["arena","cadrl","dwa","mpc","teb"],"map1","10","vel_01.")
    #  20 01
    eval_all(["arena","cadrl","dwa","mpc","teb"],"map1","20","vel_01.")

    # empty map
    #  5 01
    eval_all(["arena","cadrl","dwa","mpc","teb"],"empty","5","vel_01.")    
    #  10 01
    eval_all(["arena","cadrl","dwa","mpc","teb"],"empty","10","vel_01.")    
    #  20 01
    eval_all(["arena","cadrl","dwa","mpc","teb"],"empty","20","vel_01.")
    plt.show()
    rospy.spin()


if __name__=="__main__":
    run()

