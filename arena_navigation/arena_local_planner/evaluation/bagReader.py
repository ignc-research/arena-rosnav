# 
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
# 
class newBag():
    def __init__(self,bag_name):
        # self.bag = rosbag.Bag(bag_name)
        self.bag = bagreader(bag_name)
        self.rbag = rosbag.Bag(bag_name)


    def printBag(self,df):
        for col in df.columns:
            print(col)


    def getPos_tb3(self,topic):
        pose_x = []
        pose_y = []
        t = []
        odom_csv = self.bag.message_by_topic(topic)
        df_odom = pd.read_csv(odom_csv, error_bad_lines=False)

        # self.printBag(df_odom)
        for i in range(len(df_odom)): 
            t.append(df_odom.loc[i, "Time"])
            pose_x.append(df_odom.loc[i, "pose.pose.position.x"])
            pose_y.append(df_odom.loc[i, "pose.pose.position.y"])
        return t, pose_x, pose_y

    def makeDF_obst(self,topic):
        pos_x = []
        pos_y = []
        vel = []
        for topic, msg, t in self.rbag.read_messages(topics=[topic]):
            if len(msg.mean_points)>0:
                # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                tmp_x = []
                tmp_y = []
                for arr in msg.mean_points:
                    tmp_x.append(arr.x)
                    tmp_y.append(arr.y)
                pos_x.append(tmp_x)
                pos_y.append(tmp_y)
                # pos.append()
        pos = [pos_x, pos_y]        
        return pos, vel

    def evalPath(self):
        t, pose_x, pose_y = self.getPos_tb3("/odom")
        plt.plot(pose_y,pose_x)
        plt.show()

        x = np.array(pose_x)
        y = np.array(pose_y)
        dist_array = (x[:-1]-x[1:])**2 + (y[:-1]-y[1:])**2
        path_length = np.sum(np.sqrt(dist_array)) 
        duration = t[len(t)-1] - t[0]
        av_vel = path_length/duration


        return [duration, path_length, av_vel,len(t)]

    




#  run code
def run():
    bag = newBag("bags/teb.bag")
    eval = bag.evalPath()
    # print(eval)





    #*****************  
    # pos_arr, vel_arr = bag.makeDF_obst("/obst_odom")
    # print(pos_arr)
    # xn = []
    # yn = pos_arr[1]
    # idx = 0
    # for x in pos_arr[0]:
    #     for i in x:
    #        xn.append(i)
    # for y in pos_arr[1]:
    #     for i in x:
    #        xn.append(i)

    # for x, y in zip(pos_arr[0], pos_arr[1]):
    #     for xi,yi in zip(x,y):
    #         xn.append(xi)
    #         yn.append(yi)
    # print(xn)
    # print(yn)


if __name__=="__main__":
    run()

