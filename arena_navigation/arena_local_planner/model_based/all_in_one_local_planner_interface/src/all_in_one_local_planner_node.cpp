//
// Created by johannes on 19.07.21.
//

#include "../include/all_in_one_local_planner_interface/all_in_one_local_planner_node.h"


AllInOneLocalPlannerNode::AllInOneLocalPlannerNode() :
    _blp_loader("nav_core", "nav_core::BaseLocalPlanner"),
    tfBuffer(ros::Duration(10)),
    tfListener(tfBuffer) {

    // create ROS handle
    ros::NodeHandle nh("~");

    // create Costmap
    _costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tfBuffer);
    _costmap_ros->start();
    _time_last_resetted = ros::Time::now();
    ROS_INFO("Started local costmap!");

    // read parameters
    nh.param("local_planner_type", _local_planner_type, _local_planner_type);
    nh.param("local_planner_name", _local_planner_name, _local_planner_name);
    nh.param("min_tolerated_x_vel", _minimum_x_vel, _minimum_x_vel);
    nh.param("max_time_between_resets", _max_time_between_resets, _max_time_between_resets);
    nh.param("reset_costmap_automatically", _reset_costmap_automatically, _reset_costmap_automatically);
    nh.param("reset_costmap_interval", _reset_costmap_interval, _reset_costmap_interval);

    ROS_INFO_STREAM("Initialize " << _local_planner_type << "...");

    // load and initialize local planner
    try {
        _local_planner = _blp_loader.createInstance(_local_planner_type);
        ROS_INFO_STREAM("Loaded local planner " + _local_planner_type);
        _local_planner->initialize(_local_planner_name, &tfBuffer, _costmap_ros);
    } catch (const pluginlib::PluginlibException &ex) {
        ROS_FATAL_STREAM("Failed to create local planner " << _local_planner_type << "!");
        exit(1);
    }

    // create services
    _getVelCommandWithGlobalPlanService = nh.advertiseService("getVelCommandWithGlobalPlan",
                                                                                 &AllInOneLocalPlannerNode::getVelCommandWithGlobalPlanCallback,
                                                                                 this);
    _getVelCmdCallbackService = nh.advertiseService("getVelCmd",
                                                                       &AllInOneLocalPlannerNode::getVelCmdCallback,
                                                                       this);
    _setGlobalPlanService = nh.advertiseService("setGlobalPlan",
                                                                   &AllInOneLocalPlannerNode::setGlobalPlanCallback,
                                                                   this);
    _resetCostmap = nh.advertiseService("resetCostmap",
                                                           &AllInOneLocalPlannerNode::resetCostmapCallback,
                                                           this);

    ROS_INFO("All services were created!");
}

bool AllInOneLocalPlannerNode::getVelCommandWithGlobalPlanCallback(
        all_in_one_local_planner_interface::GetVelCmdWithGlobalPlan::Request &req,
        all_in_one_local_planner_interface::GetVelCmdWithGlobalPlan::Response &rep) {

    ROS_DEBUG("Called service");
    bool global_plan_success = _local_planner->setPlan(req.global_plan.poses);
    if (!global_plan_success) {
        ROS_ERROR("Could not update the global plan!");
        return false;
    }
    ROS_DEBUG("Updated global path.");
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap_ros->getCostmap()->getMutex()));
    rep.successful = _local_planner->computeVelocityCommands(rep.vel);
    ROS_DEBUG("Planner done");
    if (check_recovery(rep.vel)) {
        do_recovery();
        rep.costmaps_resetted = true;
    } else {
        rep.costmaps_resetted = false;
    }
    ROS_DEBUG("Service done");
    return true;
}

bool AllInOneLocalPlannerNode::getVelCmdCallback(all_in_one_local_planner_interface::GetVelCmd::Request &req,
                                                 all_in_one_local_planner_interface::GetVelCmd::Response &rep) {
    {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap_ros->getCostmap()->getMutex()));
    rep.successful = _local_planner->computeVelocityCommands(rep.vel);

    if (check_recovery(rep.vel)) {
        do_recovery();
        rep.costmaps_resetted = true;
    } else {
        rep.costmaps_resetted = false;
    }}
    return true;
}

bool AllInOneLocalPlannerNode::setGlobalPlanCallback(all_in_one_local_planner_interface::SetGlobalPlan::Request &req,
                                                     all_in_one_local_planner_interface::SetGlobalPlan::Response &rep) {
    return _local_planner->setPlan(req.global_plan.poses);
}

bool AllInOneLocalPlannerNode::check_recovery(geometry_msgs::Twist &vel_cmd) {
    // check if costmap needs to be resetted
    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff = (current_time - _time_last_resetted);

    bool vel_x_too_low = std::abs(vel_cmd.linear.x < _minimum_x_vel);
    bool time_diff_sufficient = time_diff.toSec() > _max_time_between_resets;
    bool automatic_reset = _reset_costmap_automatically && time_diff.toSec() >= _reset_costmap_interval;

    bool reset = time_diff_sufficient && (automatic_reset || vel_x_too_low);

    return reset;
}

void AllInOneLocalPlannerNode::do_recovery() {
    _costmap_ros->resetLayers();
    _costmap_ros->updateMap();
    _time_last_resetted = ros::Time::now();
}

bool AllInOneLocalPlannerNode::resetCostmapCallback(all_in_one_local_planner_interface::ResetCostmap::Request &req,
                                                    all_in_one_local_planner_interface::ResetCostmap::Response &rep) {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap_ros->getCostmap()->getMutex()));
    ROS_DEBUG("Service called");
    _costmap_ros->resetLayers();
    ROS_DEBUG("Resetted costmaps");
    _time_last_resetted = ros::Time::now();
    rep.costmaps_resetted = true;
    ROS_DEBUG("Service done!");
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "all_in_one_local_planner_node");

    auto* local_planner_node = new AllInOneLocalPlannerNode();

    ros::spin();

    return 0;
}
