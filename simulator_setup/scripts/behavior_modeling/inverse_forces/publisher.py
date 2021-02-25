#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Header

def talker():
    pub_goal = rospy.Publisher('forces_goal', PointStamped, queue_size=10)
    pub_obstacle = rospy.Publisher('forces_obstacle', PointStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goal = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(1, 1, 0))
        obstacle = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(1, 5, 0))
        # rospy.loginfo(goal)
        pub_goal.publish(goal)
        pub_obstacle.publish(obstacle)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass