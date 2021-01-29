# 
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.pyplot import figure
# 
class newBag():
    def __init__(self,bag_name):
        self.bag = bagreader(bag_name)
        # self.rbag = rosbag.Bag(bag_name)


    def printBag(self,df):
        for col in df.columns:
            print(col)


    def split_bag(self):
        odom_csv = self.bag.message_by_topic("/odom")
        df_odom = pd.read_csv(odom_csv, error_bad_lines=False)

        collision_csv = self.bag.message_by_topic("/collision")
        df_collision = pd.read_csv(collision_csv, error_bad_lines=False)
        t_col = []
        for i in range(len(df_collision)): 
            t_col.append(df_collision.loc[i, "Time"])
        print("collisions: ",len(t_col))

        old_t = 0 
        pose_x = []
        pose_y = []
        t = []
        bags = {}
        n = 0
        n_c = 0

        for i in range(len(df_odom)): 
            current_time = df_odom.loc[i, "Time"]

            # check if respawned
            if abs(df_odom.loc[i, "pose.pose.position.x"]) <= 0.1 and abs(df_odom.loc[i, "pose.pose.position.y"]) <= 0.1:
                if current_time - old_t > 10 and i> 100:
                    old_t = current_time
                    n += 1
                    # store the run
                    bags["run_"+str(n)] = [pose_x,pose_y,t,n_c]

                    # reset 
                    pose_x = []
                    pose_y = []
                    t = []

            # get trajectory
            t.append(current_time)
            pose_x.append(df_odom.loc[i, "pose.pose.position.x"])
            pose_y.append(df_odom.loc[i, "pose.pose.position.y"])
            # check for col
            if len(t_col) > 0:
                if current_time >= t_col[0]:
                    n_c += 1
                    t_col.pop(0)
            else:
                n_c = 0

            

        return bags
  
                

    def getPos_tb3(self,topic):
        pose_x = []
        pose_y = []
        t = []
        
        odom_csv = self.bag.message_by_topic(topic)
        df_odom = pd.read_csv(odom_csv, error_bad_lines=False)

        print(goal_csv)
        # self.printBag(df_odom)
        for i in range(len(df_odom)): 
            t.append(df_odom.loc[i, "Time"])
            pose_x.append(df_odom.loc[i, "pose.pose.position.x"])
            pose_y.append(df_odom.loc[i, "pose.pose.position.y"])
        return t, pose_x, pose_y

    def evalPath(self,clr,bags = None):
        global ax
        for run in bags:
            pose_x = bags[run][0]
            pose_y = bags[run][1]

            x = np.array(pose_x)
            y = -np.array(pose_y)
            
            t = bags[run][2]
            ax.plot(y,x, color=clr, label='Cosine wave')
            # plt.xlim([-5, 9])
            # plt.ylim([-3, 7])
            


            dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
            path_length = np.sum(np.sqrt(dist_array)) 
            # print(run)
            # print(len(t))
            duration = t[len(t)-1] - t[0]
            av_vel = path_length/duration

            n_col = bags[run][3]

            print([duration, path_length, av_vel, n_col, len(t)])
            break
        print()


def run():
 

    bag_arena = newBag("bags/scenarios/arena_scenario_0.bag")
    bag_cadrl = newBag("bags/scenarios/cadrl_scenario_0.bag")
    bag_teb = newBag("bags/scenarios/teb_scenario_0.bag")
    bag_mpc = newBag("bags/scenarios/mpc_scenario_0.bag")

    bags_arena = bag_arena.split_bag()
    bags_cadrl = bag_cadrl.split_bag()
    bags_teb = bag_teb.split_bag()
    bags_mpc = bag_mpc.split_bag()

    global ax
    fig, ax = plt.subplots(figsize=(6, 12))
    bag_arena.evalPath("black",bags_arena)
    bag_cadrl.evalPath("blue",bags_cadrl)
    bag_teb.evalPath("red",bags_teb)
    bag_mpc.evalPath("purple",bags_mpc)
    plt.show()

    # eval = bag.evalPath()
    # print(eval)



if __name__=="__main__":
    run()

