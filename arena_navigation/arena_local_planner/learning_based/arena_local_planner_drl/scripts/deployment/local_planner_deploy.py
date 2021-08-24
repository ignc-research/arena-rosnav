#!/usr/bin/env python
# modified based on johanes'scripts


import os
import rospkg, rospy
import yaml
from rl_agent.envs.all_in_one_models.drl.drl_agent import DrlAgent
from rl_agent.utils.all_in_one_observation_collector import ObservationCollectorAllInOne
from geometry_msgs.msg import Twist




if __name__ == "__main__":
    import rospkg, rospy
    import yaml
    from rl_agent.envs.all_in_one_models.drl.drl_agent import DrlAgent
    from rl_agent.utils.all_in_one_observation_collector import ObservationCollectorAllInOne
    from geometry_msgs.msg import Twist
    local_agent_dir = rospkg.RosPack().get_path('arena_local_planner_drl')
    drl_model_path = os.path.join(local_agent_dir,'agents','rule_04')
    robot_setting_path = os.path.join(rospkg.RosPack().get_path('simulator_setup'), 'robot', 'myrobot.model.yaml')
    settings_yaml_path = os.path.join(rospkg.RosPack().get_path('arena_local_planner_drl'), 'configs',
                                     'default_settings.yaml'),
    def setup_robot_configuration(robot_yaml_path: str, settings_yaml_path: str):
        with open(robot_yaml_path, "r") as fd:
            robot_data = yaml.safe_load(fd)
            # get robot radius
            for body in robot_data["bodies"]:
                if body["name"] == "base_footprint":
                    for footprint in body["footprints"]:
                        if footprint["type"] == "circle":
                            robot_radius = (
                                footprint.setdefault("radius", 0.3) * 1.05
                            )
                        if footprint["radius"]:
                            robot_radius = footprint["radius"] * 1.05
            # get laser related information
            for plugin in robot_data["plugins"]:
                if plugin["type"] == "Laser":
                    laser_angle_min = plugin["angle"]["min"]
                    laser_angle_max = plugin["angle"]["max"]
                    laser_angle_increment = plugin["angle"]["increment"]
                    laser_num_beams = int(
                        round(
                            (laser_angle_max - laser_angle_min) / laser_angle_increment
                        )
                        + 1
                    )
                    laser_max_range = plugin["range"]
        return laser_num_beams,laser_max_range
    laser_num_beams,laser_max_range = setup_robot_configuration(robot_yaml_path=robot_setting_path,settings_yaml_path=settings_yaml_path)
    agent = DrlAgent(drl_model_path,'drl_rule_04')

    rospy.init_node("local_planner",anonymous=True)
    ns = rospy.get_namespace()
    ns = ns.strip('/')
    if len(ns):
        ns_prefix = '/'+ns+'/'
    else:
        ns_prefix = '/'
    observation_collector = ObservationCollectorAllInOne(ns=ns,num_lidar_beams=laser_num_beams,lidar_range=laser_max_range,subgoal_topic_name="waypoint",train_mode=False,no_wait_for_next_circle=True)
    agent_action_pub = rospy.Publisher(
                f"{ns_prefix}cmd_vel", Twist, queue_size=1
            )
    
    def cb_pub_action(agent:DrlAgent,observation_collector:ObservationCollectorAllInOne,agent_action_pub):

        merged_obs, obs_dict = observation_collector.get_observations()
        action = agent.get_next_action(obs_dict)
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        agent_action_pub.publish(action_msg)

    cb_func_wrapper = lambda _: cb_pub_action(agent,observation_collector,agent_action_pub)


    timer = rospy.Timer(rospy.Duration(0.1),cb_func_wrapper)
    rospy.spin()
    