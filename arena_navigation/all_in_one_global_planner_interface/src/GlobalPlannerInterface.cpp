//
// Created by johannes on 27.07.21.
//

#include "../include/all_in_one_global_planner_interface/GlobalPlannerInterface.h"

GlobalPlannerInterface::GlobalPlannerInterface() : bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
                                                   tfBuffer(ros::Duration(10)),
                                                   tfListener(tfBuffer) {

    // create ROS handle
    ros::NodeHandle nh("~");

    // create Costmap
    _costmap_ros = new costmap_2d::Costmap2DROS("global_costmap", tfBuffer);
    _costmap_ros->start();
    _time_last_resetted = ros::Time::now();

    ROS_INFO("Started global costmap!");

    nh.param("global_planner_type", _global_planner_type, _global_planner_type);
    nh.param("reset_costmap_automatically", _reset_costmap_automatically, _reset_costmap_automatically);
    nh.param("reset_costmap_interval", _reset_costmap_interval, _reset_costmap_interval);

    try {
        _global_planner = bgp_loader_.createInstance(_global_planner_type);
        _global_planner->initialize(bgp_loader_.getName(_global_planner_type), _costmap_ros);
    } catch (const pluginlib::PluginlibException &ex) {
        ROS_FATAL_STREAM("Failed to create global planner " << _global_planner_type << "!");
        exit(1);
    }

    // create services
    _getGlobalPlan = nh.advertiseService("makeGlobalPlan",
                                         &GlobalPlannerInterface::makeNewPlanCallback,
                                         this);

    _resetCostmap = nh.advertiseService("resetGlobalCostmap", &GlobalPlannerInterface::resetCostmapCallback, this);
}

bool GlobalPlannerInterface::makeNewPlanCallback(all_in_one_global_planner_interface::MakeNewPlan::Request &req,
                                                 all_in_one_global_planner_interface::MakeNewPlan::Response &rep) {
    geometry_msgs::PoseStamped robot_pose;
    _costmap_ros->getRobotPose(robot_pose);

    std::vector<geometry_msgs::PoseStamped> global_plan;
    if (!_global_planner->makePlan(robot_pose, req.goal, global_plan) || global_plan.empty()) {
        reset_costmap();
        if (!_global_planner->makePlan(robot_pose, req.goal, global_plan) || global_plan.empty()) {
            ROS_WARN("Couldn't find global plan!");
            rep.global_plan.poses = {robot_pose, req.goal};
        }
    }

    automatic_reset();

    rep.global_plan.poses.resize(global_plan.size());
    for (unsigned int i = 0; i < global_plan.size(); ++i) {
        rep.global_plan.poses[i] = global_plan[i];
    }
    return true;
}

void GlobalPlannerInterface::reset_costmap() {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap_ros->getCostmap()->getMutex()));
    _costmap_ros->resetLayers();
    _costmap_ros->updateMap();
    _time_last_resetted = ros::Time::now();
}

bool GlobalPlannerInterface::resetCostmapCallback(std_srvs::Empty::Request &request,
                                                  std_srvs::Empty::Response &response) {
    reset_costmap();
    return true;
}

void GlobalPlannerInterface::automatic_reset() {
    ros::Time current_time = ros::Time::now();
    ros::Duration time_diff = (current_time - _time_last_resetted);
    bool automatic_reset = _reset_costmap_automatically && time_diff.toSec() >= _reset_costmap_interval;

    if (automatic_reset) {
        reset_costmap();
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "all_in_one_global_planner_node");

    auto *local_planner_node = new GlobalPlannerInterface();

    ros::spin();

    return 0;
}
