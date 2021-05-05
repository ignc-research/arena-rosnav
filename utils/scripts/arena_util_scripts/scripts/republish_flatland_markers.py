#!/usr/bin/env python
import rospy
import time
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from flatland_msgs.msg import DebugTopicList

class FlatlandSubscriber:
    def __init__(self, topic, subscriber):
        self.topic = topic
        self.subscriber = subscriber
        self.markers = []


class MarkerPublisher:
    def __init__(self):
        rospy.init_node('marker_publisher', anonymous=True)
        self.markers = {}  # key: flatland debug topic name, value: FlatlandSubscriber object
        self.markers_pub = rospy.Publisher(rospy.get_namespace() + 'flatland_markers', MarkerArray, queue_size=10)
        self.debug_topics_sub = rospy.Subscriber(rospy.get_namespace() + "flatland_server/debug/topics", DebugTopicList, self.debug_topics_callback)

    
    def debug_topics_callback(self, msg):
        # remove topics/markers/subscribers of deleted models
        current_topics = list(self.markers.keys())  # make list so we get a copy of the keys
        for topic in current_topics:
            if topic not in msg.topics:
                self.markers[topic].subscriber.unregister()
                self.markers.pop(topic)

        # create subscribers for new topics
        for topic in msg.topics:
            if topic not in self.markers.keys():
                sub = rospy.Subscriber(rospy.get_namespace() + "flatland_server/debug/" + topic, MarkerArray, self.flatland_marker_array_callback, callback_args=(topic,))
                self.markers[topic] = FlatlandSubscriber(topic, sub)


    def flatland_marker_array_callback(self, marker_array, args):
        topic = args[0]
        self.markers[topic].markers = marker_array.markers


    def publish_markers(self):
        rate = 1.0 / 50.0  # 50Hz
        while not rospy.is_shutdown():
            marker_array = MarkerArray()

            # collect markers
            for flatland_sub in self.markers.values():
                marker_array.markers.extend(flatland_sub.markers)

            # give every marker its own id
            for i in range(len(marker_array.markers)):
                marker_array.markers[i].id = i

            self.markers_pub.publish(marker_array)
            time.sleep(rate)


    def run(self):
        self.publish_markers()


# This node takes all visualization markers that
# are published by Flatland over a number of
# different topics and publishes them all under
# a single topic.
if __name__ == '__main__':
    publisher = MarkerPublisher()
    try:
        publisher.run()
    except rospy.ROSInterruptException:
        pass
