# 
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy

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

        reset_csv = self.bag.message_by_topic("/scenario_reset")
        # df_reset = pd.read_csv(reset_csv, error_bad_lines=False)

        old_t = 0 
        pose_x = []
        pose_y = []
        t = []
        bags = {}
        n = 0
        for i in range(len(df_odom)): 
            if abs(df_odom.loc[i, "pose.pose.position.x"]) <= 0.1 and abs(df_odom.loc[i, "pose.pose.position.y"]) <= 0.1:
                if df_odom.loc[i, "Time"] - old_t > 10:
                    old_t = df_odom.loc[i, "Time"]
                    # reset 
                    n += 1
                    bags["run_"+str(n)] = ([pose_x,pose_y,t])
                    pose_x = []
                    pose_y = []
                    t = []
            t.append(df_odom.loc[i, "Time"])
            pose_x.append(df_odom.loc[i, "pose.pose.position.x"])
            pose_y.append(df_odom.loc[i, "pose.pose.position.y"])


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

    def evalPath(self,pose_x, pose_y, t, bags = None):
        if bags == None:
            fig, ax = plt.subplots(figsize=(12, 6))
            # plt.plot(pose_y,pose_x)
            # plt.show()
            ax.plot(pose_y,pose_x, color='black', label='Cosine wave')
            # plt.xlim([-5, 9])
            # plt.ylim([-3, 7])
            plt.show()

            x = np.array(pose_x)
            y = np.array(pose_y)
            dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
            path_length = np.sum(np.sqrt(dist_array)) 
            duration = t[len(t)-1] - t[0]
            av_vel = path_length/duration


            return [duration, path_length, av_vel,len(t)]
        else:
            fig, ax = plt.subplots(figsize=(6, 12))
            for run in bags:
                pose_y
                pose_x = bags[run][0]
                pose_y = bags[run][1]
                t = bags[run][2]
                ax.plot(pose_y,pose_x, color='black', label='Cosine wave')
                # plt.xlim([-5, 9])
                # plt.ylim([-3, 7])
                

                x = np.array(pose_x)
                y = np.array(pose_y)
                dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
                path_length = np.sum(np.sqrt(dist_array)) 
                duration = t[len(t)-1] - t[0]
                av_vel = path_length/duration

                print( [duration, path_length, av_vel,len(t)])
            plt.show()


def run():
    rospy.init_node('eval',anonymous=False)
    print("node on")

    # bag = newBag("bags/scenarios/arena_scenario_0.bag")
    bag = newBag("bags/scenarios/cadrl_scenario_0.bag")
    bags = bag.split_bag()
    print(len(bags))
    run = "run_10"
    x = bags[run][0]
    y = bags[run][1]
    t = bags[run][2]

    bag.evalPath(x,y,t,bags)
    # eval = bag.evalPath()
    # print(eval)



    rospy.spin()


if __name__=="__main__":
    run()

