# 
import rosbag
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy
from visualization_msgs.msg import Marker, MarkerArray
# 
class newBag():
    def __init__(self,bag_name):
        self.bag = bagreader(bag_name)
        # self.rbag = rosbag.Bag(bag_name)


    def printBag(self,df):
        for col in df.columns:
            print(col)

    def collision(self):
        n_col = 0

    def getPos_tb3(self,topic):
        pose_x = []
        pose_y = []
        t = []
        dist = []
        
        odom_csv = self.bag.message_by_topic(topic)
        df_odom = pd.read_csv(odom_csv, error_bad_lines=False)

        # self.printBag(df_odom)
        for i in range(len(df_odom)): 
            t.append(df_odom.loc[i, "Time"])
            pose_x.append(df_odom.loc[i, "pose.pose.position.x"])
            pose_y.append(df_odom.loc[i, "pose.pose.position.y"])
        return t, pose_x, pose_y

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
def getMap(msg):
    points_x = []
    points_y = []
    print(msg.markers[0])
    for p in msg.markers[0].points:
        points_x.append(p.x)
        points_y.append(-p.y)
    plt.scatter(points_y, points_x)
    plt.show()

def run():
    rospy.init_node('eval',anonymous=False)
    print("node on")

    rospy.Subscriber('/flatland_server/debug/layer/static',MarkerArray,getMap)
    # bag = newBag("bags/teb.bag")
    # eval = bag.evalPath()
    # print(eval)



    rospy.spin()


if __name__=="__main__":
    run()

