#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanMapper():
    """
    A class that switches the laserscan representations from the /scan topic for different navigation algorithms, see more in the map_scan.launch
    Parameters: angle_min, angle_max, increment from roslaunch 
    Subscribed topics: /scan
    Published topics: /scan_mapped
    """

    def __init__(self):

        # Subscribers (callback functions are triggered on incoming data and written to 'data' as defined by ROS)
        self._robot_state_sub = rospy.Subscriber('/scan', LaserScan, self.callback_change_laserscan)

        # Publishers
        self._new_scan_pub = rospy.Publisher('/scan_mapped', LaserScan, queue_size=1)



    def callback_change_laserscan(self, data):
        print("test")
        data.angle_min = rospy.get_param('angle_min')
        data.angle_max = rospy.get_param('angle_max')
        data.angle_increment = rospy.get_param('increment')
        data.ranges = self.shift_scan_data(data.ranges)
        data.intensities = self.shift_scan_data(data.intensities)
        self._new_scan_pub.publish(data)

    def shift_scan_data(self, array):
        print("[scan_mapping]: shifting data")
        tmp = np.array(array)
        shift = int(len(tmp) * 0.25)
        shifted_data = np.roll(tmp, -shift)
        return shifted_data




if __name__ == '__main__':
    rospy.init_node('add_offset', anonymous=True, disable_signals=False)

    scanMapper = ScanMapper()
    rospy.spin()