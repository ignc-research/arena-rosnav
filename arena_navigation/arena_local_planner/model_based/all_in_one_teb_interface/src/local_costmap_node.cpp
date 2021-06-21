//
// Created by johannes on 14.06.21.
//

#include "all_in_one_teb_interface/local_costmap_node.h"

LocalCostmapNode::LocalCostmapNode(tf2_ros::Buffer &tf, std::string &name) {
    LocalCostmapNode::costmap_ros_ = new costmap_2d::Costmap2DROS(name, tf);
    ROS_INFO("Started local costmap!");
    // LocalCostmapNode::costmap_ros_->start();
}

void LocalCostmapNode::clearCostmap() {
    LocalCostmapNode::costmap_ros_->resetLayers();
}

costmap_2d::Costmap2DROS *LocalCostmapNode::getCostmapRos() const {
    return costmap_ros_;
}
