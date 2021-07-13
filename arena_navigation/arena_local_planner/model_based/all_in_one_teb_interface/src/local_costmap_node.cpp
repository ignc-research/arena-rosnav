//
// Created by johannes on 14.06.21.
//

#include "all_in_one_teb_interface/local_costmap_node.h"

LocalCostmapNode::LocalCostmapNode(): costmap_ros_(nullptr){}

void LocalCostmapNode::initialize(tf2_ros::Buffer &tf, std::string &name) {
    LocalCostmapNode::costmap_ros_ = new costmap_2d::Costmap2DROS(name, tf);
    LocalCostmapNode::costmap_ros_->start();
    ROS_INFO("Started local costmap!");
}

void LocalCostmapNode::clearCostmap(bool force_map_update) {
//    // This method is based on the clear costmap recovery of the navigation stack.
//    ROS_INFO("Clear local costmap!");
//
//    if (costmap_ros_ == nullptr) {
//        ROS_ERROR("Costmap cannot be resetted because it hasn't been initialized yet");
//        return;
//    }
//
//    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap_ros_->getLayeredCostmap()->getPlugins();
//
//    geometry_msgs::PoseStamped pose;
//    if(!costmap_ros_->getRobotPose(pose)){
//        ROS_ERROR("Cannot clear map because pose cannot be retrieved");
//        return;
//    }
//    double robot_pose_x = pose.pose.position.x;
//    double robot_pose_y = pose.pose.position.y;
//
//    for (const auto& plugin : *plugins) {
//
//        std::string name = plugin->getName();
//        int slash = name.rfind('/');
//        if (slash != std::string::npos) {
//            name = name.substr(slash + 1);
//        }
//        if (name == "obstacle_layer") {
//            boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
//            costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
//            clearMap(costmap, robot_pose_x, robot_pose_y);
//        }
//    }
//    // force costmap update
//    if (force_map_update) {
//        costmap_ros_->updateMap();
//    }
    costmap_ros_->resetLayers();
}

costmap_2d::Costmap2DROS *LocalCostmapNode::getCostmapRos() {
    return costmap_ros_;
}

void LocalCostmapNode::clearMap(const boost::shared_ptr<costmap_2d::CostmapLayer>& costmap,
                                double robot_pose_x, double robot_pose_y) {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    double size_x = costmap_ros_->getCostmap()->getSizeInMetersX() - 1;
    double size_y = costmap_ros_->getCostmap()->getSizeInMetersY() - 1;

    double start_point_x = robot_pose_x - size_x / 2;
    double start_point_y = robot_pose_y - size_y / 2;
    double end_point_x = start_point_x + size_x;
    double end_point_y = start_point_y + size_y;

    int start_x, start_y, end_x, end_y;
    costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
    costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

    costmap->clearArea(start_x, start_y, end_x, end_y);

    double ox = costmap->getOriginX(), oy = costmap->getOriginY();
    double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
    costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

bool LocalCostmapNode::clearCostmap_service(all_in_one_teb_interface::ResetCostmap::Request &req,
                                            all_in_one_teb_interface::ResetCostmap::Response &rep) {
    clearCostmap(false);
    rep.costmaps_resetted = true;
    return true;
}
