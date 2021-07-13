//
// Created by johannes on 26.05.21.
//

#ifndef ALL_IN_ONE_INTERFACE_H
#define ALL_IN_ONE_INTERFACE_H

#include <teb_local_planner/optimal_planner.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <all_in_one_teb_interface/TebService.h>
#include "local_costmap_node.h"

class AllInOneInterface {
public:
    AllInOneInterface();

    void initialize(teb_local_planner::TebConfig *config, LocalCostmapNode *local_costmap,
                      ros::NodeHandle &nh

    );

    bool service_callback(all_in_one_teb_interface::TebService::Request &req,
                          all_in_one_teb_interface::TebService::Response &rep);

    bool calculateNewPlan(const teb_local_planner::PoseSE2 &goal,
                          const geometry_msgs::Twist *start_vel,
                          all_in_one_teb_interface::TebService::Response &rep);

private:
    teb_local_planner::TebConfig *config_;
    std::vector<teb_local_planner::ObstaclePtr> obst_vector;
    boost::shared_ptr<teb_local_planner::PlannerInterface> planner;
    LocalCostmapNode *local_costmap_node_;
    teb_local_planner::PoseSE2 robot_pose_;
    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
    boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;
    teb_local_planner::TebVisualizationPtr visualizer;

    teb_local_planner::RobotFootprintModelPtr getRobotModel();

    void updateObstacleContainerWithCostmapConverter();

    void updateObstacleContainerWithCostmap();
};


#endif //ALL_IN_ONE_INTERFACE_H
