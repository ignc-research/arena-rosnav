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

# 
class newBag():
    def __init__(self, planner, plot_style, bag_name, odom_topic="/sensorsim/police/odom", collision_topic="/sensorsim/police/collision"):
        self.bag = bagreader(bag_name)
        eps = self.split_runs(odom_topic, collision_topic)
        self.evalPath(plot_style,planner,eps)

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
        collision_csv = self.bag.message_by_topic(collision_topic)
        df_collision = pd.read_csv(collision_csv, error_bad_lines=False)
        t_col = []
        for i in range(len(df_collision)): 
            t_col.append(df_collision.loc[i, "Time"])
        print("collisions in total: ",len(t_col))
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
            if current_time > reset-6 and n < len(t_reset)-1 and x<0:
                n += 1
                # store the run

                # print("run_"+str(n))
                # print("x:",x,"y",y)
                bags["run_"+str(n)] = [pose_x,pose_y,t,col_xy]

                # reset 
                pose_x = []
                pose_y = []
                t = []
                col_xy = []
   
            if  len(pose_x) > 0:
                pose_x.append(x)
                pose_y.append(y)
            elif x < 0:
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

            
        bags.pop("run_1")
        return bags
    
    def average(self,lst): 
        return sum(lst) / len(lst) 

    def print_patches(self,xya):
        global ax

        for run_a in xya:
            for col_xy in run_a:
                circle = plt.Circle((-col_xy[1], col_xy[0]), 0.3, color='r', fill = False)
                ax.add_patch(circle)

    def evalPath(self,clr,planner,bags = None):
        col_xy = []
        global ax
        # circle1 = plt.Circle((-9, 12), 0.3, color='r', fill = False)
        # ax.add_patch(circle1)
        durations = [] 
        trajs = []
        vels  = []
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
                    ax.plot(y,x, clr, label='Cosine wave')

                duration = t[len(t)-1] - t[0]
                # for av
                durations.append(duration)
                av_vel = path_length/duration
                # for av
                vels.append(av_vel)

                n_col = len(bags[run][3])

                print([duration, path_length, av_vel, n_col])
                # print("\n------------------------\n",bags[run][3])
                col_xy.append(bags[run][3])


        print("----------------------   "+planner+"   ----------------------")
        print("average time:        ", self.average(durations))
        print("average path length: ", self.average(trajs))
        print("average velocity:    ", self.average(vels))

        self.print_patches(col_xy)
        # self.make_heat_map(col_xy)
            


def run():
    global ax
    fig, ax = plt.subplots(figsize=(6, 12))

    # #teb
    # # read bag
    # bag_teb = newBag("bags/scenaries/teb_ob_05_vel_02.bag")
    # # split runs
    # eps_teb = bag_teb.split_runs()
    # # evaluate
    # bag_teb.evalPath("r",eps_teb)

    # #dwa
    # # read bag
    # bag_dwa = newBag("bags/scenaries/dwa_ob_05_vel_02.bag")
    # # split runs
    # eps_dwa = bag_dwa.split_runs()
    # # evaluate
    # bag_dwa.evalPath("black",eps_dwa)

    # #mpc
    # # read bag
    # bag_mpc = newBag("bags/scenaries/mpc_ob_05_vel_02.bag")
    # # split runs
    # eps_mpc = bag_mpc.split_runs()
    # # evaluate
    # bag_mpc.evalPath("purple",eps_mpc)    
    
    # #cadrl
    # # read bag
    # bag_cadrl = newBag("bags/scenaries/cadrl_ob_05_vel_02.bag")
    # # split runs
    # eps_cadrl = bag_cadrl.split_runs()
    # # evaluate
    # bag_cadrl.evalPath("b",eps_cadrl)

    # #arena
    # # read bag
    # bag_arena = newBag("bags/scenaries/arena_ob_05_vel_02.bag")
    # # split runs
    # eps_arena = bag_arena.split_runs()
    # # evaluate
    # bag_arena.evalPath("g",eps_arena)

    # cadrl
    newBag("cadrl_01","b","bags/scenaries/cadrl_map1_ob10_vel_01.bag")
    newBag("cadrl_02","g","bags/scenaries/cadrl_map1_ob10_vel_02.bag")
    newBag("cadrl_03","k","bags/scenaries/cadrl_map1_ob10_vel_03.bag")



    plt.show()

 


if __name__=="__main__":
    run()

