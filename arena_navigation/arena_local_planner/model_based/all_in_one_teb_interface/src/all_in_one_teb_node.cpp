//
// Created by johannes on 10.06.21.
//

#include <tf2_ros/transform_listener.h>
#include "all_in_one_teb_interface/all_in_one_teb_node.h"


AllInOneTebNode::AllInOneTebNode(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer) {
    ROS_INFO("Load teb configs...");
    // load teb config from ros
    AllInOneTebNode::cfg_.loadRosParamFromNodeHandle(nh);

    // create and start local costmap
    ROS_INFO("Load costmap node...");
    std::string costmap_name = "local_costmap";

    AllInOneTebNode::local_costmap = new LocalCostmapNode();
    local_costmap->initialize(tfBuffer, costmap_name);

    // create new teb planner object
    AllInOneTebNode::teb_interface = new AllInOneInterface();
    teb_interface->initialize(&(AllInOneTebNode::cfg_), AllInOneTebNode::local_costmap, nh);

    // create a service to call teb
    AllInOneTebNode::getVelSrv_ = nh.advertiseService("getCmdVel",
                                                     &AllInOneInterface::service_callback, teb_interface);
    // create a service to reset costmap
    AllInOneTebNode::resetCostmapSrv_ = nh.advertiseService("resetCostmap",
                                                     &LocalCostmapNode::clearCostmap_service, local_costmap);

    ROS_INFO("Teb service is ready!");
}

int main(int argc, char *argv[]) {
    ROS_INFO("Initialize teb node...");

    ros::init(argc, argv, "all_in_one_teb_node");

    // create ROS handle
    ros::NodeHandle nh("~");

    // create tf listener
    ROS_INFO("Create tf listener...");
    tf2_ros::Buffer tfBuffer(ros::Duration(10));
    tf2_ros::TransformListener tfListener(tfBuffer);

    AllInOneTebNode all_in_one_teb_node(nh, tfBuffer);

    ros::spin();
}
