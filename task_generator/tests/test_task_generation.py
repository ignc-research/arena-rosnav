from task_generator.tasks import get_predefined_task
import rospy
rospy.init_node('test_task')
task = get_predefined_task("test_1")
task.reset()