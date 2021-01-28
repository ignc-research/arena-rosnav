#!/usr/bin/env python
import numpy as np
import rospkg
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    with open("positions.txt", "a") as f:
        f.write("{} {}\n".format(x, y))

    vector = data.twist.twist.linear
    vector = np.array([vector.x, vector.y, vector.z])
    vel = np.linalg.norm(vector)

    with open("velocities.txt", "a") as f:
        f.write("{}\n".format(vel))

    
def listener():
    # empty contents of the files
    open('positions.txt', 'w').close()
    open('velocities.txt', 'w').close()

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("odom", Odometry, callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    # rospack = rospkg.RosPack()
    # path = rospack.get_path('record_movement')
    # print(path)

    listener()