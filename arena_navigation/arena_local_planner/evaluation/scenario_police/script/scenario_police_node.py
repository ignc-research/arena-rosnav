#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
# 
class police():
    def __init__(self):
        self.n_col = 0
        self.collision_flag = False
        self.scan = rospy.Subscriber('/scan',LaserScan, self.cbScan)
        self.pub_col = rospy.Publisher('/collision', Int16, queue_size=10)


    def cbScan(self,msg):
        scan_array = np.asarray(msg.ranges)
        d_min = np.nanmin(scan_array)
        if np.isnan(d_min):
            d_min = 3.5

        if d_min > 0.5:
            self.collision_flag = False
        if d_min <= 0.35 and not self.collision_flag:
            self.collision_flag = True
            self.n_col += 1 
            self.pub_col.publish(self.n_col)
            print(self.n_col)
            



def run():
    rospy.init_node('scenario_police',anonymous=False)
    print("watching scene")
    police()
    rospy.spin()


if __name__=="__main__":
    run()

