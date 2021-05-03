#ifndef _DYNAMIC_PLAN_MANAGER_H_
#define _DYNAMIC_PLAN_MANAGER_H_

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string.h>

// msgs or srvs
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>          // visulization
//#include <arena_plan_msgs/MakeGlobalPlan.h>     // arena plan msg

// mapping
#include <arena_mapping/mapping.h>

// bspline & polynomial
#include <arena_traj_planner/bspline/uniform_bspline.h>
#include <arena_traj_planner/polynomial/polynomial_traj.h>

// global planner
#include <arena_path_search/kinodynamic_astar.h>
#include <arena_traj_planner/bspline_opt/bspline_optimizer_esdf.h>

// mid planner
#include "arena_dynamic_channel/dynamic_obstacle.h"
#include "arena_dynamic_channel/timed_astar_search.h"

// local planner/ traj optimizer
#include <arena_traj_planner/bspline_opt/bspline_optimizer.h>
#include <arena_traj_planner/bspline_opt/dyn_a_star.h>
// data container
#include <arena_timespace_planner/dynamic_plan_container.hpp>


class DynamicPlanManager{
private:
    ros::NodeHandle node_;

    // global planer
    KinodynamicAstar::Ptr global_planner_kino_astar_;

    // mid planner
    TimedAstarSearch::Ptr mid_planner_timed_astar_;

    // global traj optimizer
    BsplineOptimizerESDF::Ptr bspline_optimizer_esdf_;

    // local traj optimizer
    BsplineOptimizer::Ptr bspline_optimizer_;
    int continous_failures_count_{0};

    bool adjustStartAndTargetPoint( Eigen::Vector2d & start_pt, Eigen::Vector2d &target_pt);


    
public: 
    DynamicPlanManager(){}
    ~DynamicPlanManager(){}

    PlanParameters pp_;
    GridMap::Ptr grid_map_;

    GlobalData global_data_;
    MidData mid_data_;
    TargetTrajData local_traj_data_;

    
    
    // dynamic obstacle info
    std::string str_dynamic_obs_;
    std::vector<DynamicObstacleInfo::Ptr> obs_info_provider_;

    void initPlanModules(ros::NodeHandle &nh);

    void updateDynamicObstacleInfo();

    bool kinoAstarTraj(Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,Eigen::Vector2d &end_pos);


    bool planGlobalTraj( Eigen::Vector2d &start_pos,  Eigen::Vector2d &end_pos);
    
    bool optimizeGlobalTraj(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives, UniformBspline & bspline_traj);
    
    bool planMidTraj( Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,  double & start_dir,  Eigen::Vector2d & end_pos,std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &line_sets);

    bool planLocalTraj( Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,  double & start_dir,  Eigen::Vector2d & target_pos, Eigen::Vector2d & target_vel);
    
    bool genOneshotTraj(Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,  double & start_dir,  Eigen::Vector2d & target_pos, Eigen::Vector2d & target_vel, UniformBspline & oneshot_traj);

    bool optimizeBsplineTraj(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives, UniformBspline & optimized_traj);

    bool checkCollision(const Eigen::Vector2d &pos);

    typedef std::unique_ptr<DynamicPlanManager> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
















#endif