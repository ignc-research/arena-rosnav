//
// Created by johannes on 19.07.21.
//

#ifndef ALL_IN_ONE_LOCAL_PLANNER_INTERFACE_ALL_IN_ONE_LOCAL_PLANNER_NODE_H
#define ALL_IN_ONE_LOCAL_PLANNER_INTERFACE_ALL_IN_ONE_LOCAL_PLANNER_NODE_H

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "nav_core/base_local_planner.h"
#include "all_in_one_local_planner_interface/GetVelCmd.h"
#include "all_in_one_local_planner_interface/GetVelCmdWithGlobalPlan.h"
#include "all_in_one_local_planner_interface/SetGlobalPlan.h"
#include "all_in_one_local_planner_interface/ResetCostmap.h"
#include <tf2_ros/transform_listener.h>

class AllInOneLocalPlannerNode {

public:
    AllInOneLocalPlannerNode();
    bool getVelCommandWithGlobalPlanCallback(all_in_one_local_planner_interface::GetVelCmdWithGlobalPlan::Request &req,
                                             all_in_one_local_planner_interface::GetVelCmdWithGlobalPlan::Response &rep);
    bool getVelCmdCallback(all_in_one_local_planner_interface::GetVelCmd::Request &req,
                           all_in_one_local_planner_interface::GetVelCmd::Response &rep);
    bool setGlobalPlanCallback(all_in_one_local_planner_interface::SetGlobalPlan::Request &req,
                               all_in_one_local_planner_interface::SetGlobalPlan::Response &rep);
    bool resetCostmapCallback(all_in_one_local_planner_interface::ResetCostmap::Request &req,
                              all_in_one_local_planner_interface::ResetCostmap::Response &rep);
private:
    bool check_recovery(geometry_msgs::Twist &vel_cmd);
    void do_recovery();

    ros::Time _time_last_resetted;

    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> _blp_loader;
    boost::shared_ptr<nav_core::BaseLocalPlanner> _local_planner;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    costmap_2d::Costmap2DROS* _costmap_ros;

    //params
    std::string _local_planner_type = "teb_local_planner/TebLocalPlannerROS";
    std::string _local_planner_name = "teb";
    float _minimum_x_vel = 0.1f;
    float _max_time_between_resets = 0.5f;
    bool _reset_costmap_automatically = true;
    float _reset_costmap_interval = 3.0f;

    //services
    ros::ServiceServer _getVelCommandWithGlobalPlanService;
    ros::ServiceServer _getVelCmdCallbackService;
    ros::ServiceServer _setGlobalPlanService;
    ros::ServiceServer _resetCostmap;
};


#endif //ALL_IN_ONE_MPC_INTERFACE_ALL_IN_ONE_LOCAL_PLANNER_NODE_H
