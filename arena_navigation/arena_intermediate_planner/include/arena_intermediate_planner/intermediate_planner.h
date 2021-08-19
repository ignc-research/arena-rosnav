
#ifndef _INTERMEDIATE_PLANNER_COLLECTOR_H
#define _INTERMEDIATE_PLANNER_COLLECTOR_H

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// mapping
#include "arena_mapping/mapping.h"
// path search
#include "arena_path_search/astar.h"
#include "arena_path_search/kinodynamic_astar.h"

// b-spline
#include "arena_traj_planner/bspline_optimizer_esdf.h"

#include "arena_traj_planner/uniform_bspline.h"
#include "arena_traj_planner/bspline_optimizer_astar.h"
#include "arena_traj_planner/polynomial_traj.h"

// plan container
#include "arena_intermediate_planner/plan_container_mid.hpp"

// visulization
#include "plan_visualization/planning_visualization.h"

// arena plan msg
#include <arena_plan_msgs/MakeGlobalPlan.h>



class InterPlanner{
private:
    // ros node
    ros::NodeHandle node_;

    // subscriber
    ros::Subscriber goal_sub_, odom_sub_;

    // publisher
    ros::Publisher kino_astar_path_pub_,kino_astar_traj_pub_;
    
    // service server
    ros::ServiceServer global_plan_service_server_;


    // map & enviornment
    GridMap::Ptr grid_map_;
    PlanParameters pp_; //double ctrl_pt_dist_; double max_vel_; double max_acc_;
    GlobalData global_data_;
    
    // flags
    bool have_odom_;

    // planner variables
    Eigen::Vector2d odom_pos_, odom_vel_;                           // odometry state
    Eigen::Quaterniond odom_orient_;                                // orient

    //Eigen::Vector2d current_pt_,current_vel_,current_acc_;          // current state

    Eigen::Vector2d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
    Eigen::Vector2d end_pt_, end_vel_, end_acc_;                              // target state

    // global planners
    Astar::Ptr global_planner_astar_;
    KinodynamicAstar::Ptr global_planner_kino_astar_;
    PolynomialTraj::Ptr global_planner_oneshot_;

    // bspline optimizer
    BsplineOptimizerESDF::Ptr bspline_optimizer_esdf_;
    BsplineOptimizerAstar::Ptr bspline_optimizer_rebound_; 

    
    // visualization
    PlanningVisualization::Ptr visualization_;

    // performance time
    double dur_;
    ros::WallTime t1_, t2_;

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector2d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector2d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

public:
    InterPlanner(){};
    ~InterPlanner(){};

    //enum class PlanState { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };
    enum class OptimizerType { GRADIENT_ASTAR = 1, GRADIENT_ESDF = 2};

    void init(ros::NodeHandle & nh);

    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    bool makeGlobalPlan(arena_plan_msgs::MakeGlobalPlan::Request  &req, arena_plan_msgs::MakeGlobalPlan::Response &res);


    bool planGlobalTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                      const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc);



    bool planOneshotTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                      const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc);

    bool planKinoAstarTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc, const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel);

    bool planAstarTraj(const Eigen::Vector2d &start_pos,const Eigen::Vector2d &end_pos);

    bool optimizePath(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives, UniformBspline &bspline_traj, OptimizerType type_optimizer=OptimizerType::GRADIENT_ASTAR);

    void visualize_path(std::vector<Eigen::Vector2d> path, const ros::Publisher & pub);



    typedef std::shared_ptr<InterPlanner> Ptr;

};









#endif