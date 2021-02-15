#! /usr/bin/env python
import os, sys
# for me the python path must be added manuelly , could be deleted directly if your machine could identify the corresponding paths
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner/learning_based/')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner/learning_based//arena_local_planner_drl')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/arena_navigation/arena_local_planner/learning_based//arena_local_planner_drl/scripts')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/task_generator')
sys.path.append('/home/junhui/study/Masterarbeit/arenarosnav/test_ws/src/arena_rosnav/task_generator/task_generator')

from logging import setLogRecordFactory
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
from task_generator.tasks import get_predefined_task

class TaskGenerator:
    def __init__(self):
        mode = rospy.get_param("/task_generator_node/task_mode")
        scenerios_json_path = rospy.get_param("/task_generator_node/scenerios_json_path")
        paths = {"scenerios_json_path": scenerios_json_path}
        self.task = get_predefined_task(ns="sim_01",mode=mode, PATHS=paths)

        # if auto_reset is set to true, the task generator will automatically reset the task
        # this can be activated only when the mode set to 'ScenerioTask'
        auto_reset = rospy.get_param("/task_generator_node/auto_reset")
        # if the distance between the robot and goal_pos is smaller than this value, task will be reset
        self.delta_ = rospy.get_param("/task_generator_node/delta")
        robot_odom_topic_name = rospy.get_param(
            "robot_odom_topic_name", "odom")
        auto_reset = auto_reset and mode == "ScenerioTask"
        self.curr_goal_pos_ = None
        if auto_reset:
            rospy.loginfo(
                "Task Generator is set to auto_reset mode, Task will be automatically reset as the robot approaching the goal_pos")
            self.reset_task()
            self.robot_pos_sub_ = rospy.Subscriber(
                robot_odom_topic_name, Odometry, self.check_robot_pos_callback)
        else:
            # declare new service task_generator, request are handled in callback task generate
            self.task_generator_srv_ = rospy.Service(
                'task_generator', Trigger, self.reset_srv_callback)

    def reset_srv_callback(self, req):
        rospy.loginfo("Task Generator received task-reset request!")
        self.task.reset()
        return TriggerResponse(
            success= True,
            message= self.task.obstacles_manager.obstacle_name_str
        )

    def reset_task(self):
        info = self.task.reset()
        if info is not None:
            self.curr_goal_pos_ = info['robot_goal_pos']
        rospy.loginfo("".join(["="]*80))
        rospy.loginfo("goal reached and task reset!")
        rospy.loginfo("".join(["="]*80))

    def check_robot_pos_callback(self, odom_msg: Odometry):
        robot_pos = odom_msg.pose.pose.position
        robot_x = robot_pos.x
        robot_y = robot_pos.y
        goal_x = self.curr_goal_pos_[0]
        goal_y = self.curr_goal_pos_[1]

        if (robot_x-goal_x)**2+(robot_y-goal_y)**2 < self.delta_**2:
            self.reset_task()


if __name__ == '__main__':
    rospy.init_node('task_generator')
    task_generator = TaskGenerator()
    rospy.spin()
