
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
#include "arena_path_search/jps.h"
#include "arena_path_search/kinodynamic_astar.h"

// b-spline
#include "arena_traj_planner/bspline_optimizer_esdf.h"

#include "arena_traj_planner/uniform_bspline.h"
#include "arena_traj_planner/bspline_optimizer_astar.h"
#include "arena_traj_planner/polynomial_traj.h"

// plan container
#include "arena_intermediate_planner/plan_container_mid.hpp"

// visulization
#include <visualization_msgs/Marker.h>
#include "plan_visualization/planning_visualization.h"

// arena plan msg
#include <arena_plan_msgs/MakeGlobalPlan.h>




class InterPlanner{
private:
    // ros node
    ros::NodeHandle node_;

    // subscriber
    ros::Subscriber goal_sub_, odom_sub_;

    // vis publisher
    ros::Publisher kino_astar_path_pub_,    kino_astar_traj_pub_,   kino_astar_waypoints_pub_;
    ros::Publisher astar_path_pub_,         astar_traj_pub_,        astar_waypoints_pub_;
    ros::Publisher jps_path_pub_,           jps_traj_pub_,          jps_waypoints_pub_;
    ros::Publisher oneshot_path_pub_,       oneshot_traj_pub_,      oneshot_waypoints_pub_;
    
    ros::Publisher vis_control_pts_pub_,vis_control_pts_optimized_pub_;
    ros::Publisher vis_control_pts_astar_pub_,vis_control_pts_oneshot_pub_,vis_control_pts_kino_pub_;
    ros::Publisher vis_control_pts_astar_optimized_pub_,vis_control_pts_oneshot_optimized_pub_,vis_control_pts_kino_optimized_pub_;

    ros::Publisher vis_goal_pub_,vis_subgoal_pub_,vis_global_path_pub_,vis_local_traj_pub_;
    // service server
    ros::ServiceServer global_plan_service_server_;

    // map & enviornment
    GridMap::Ptr grid_map_;
    PlanParameters pp_;  

    
    // flags
    bool have_odom_;

    // planner variables
    Eigen::Vector2d odom_pos_, odom_vel_;                               // odometry state
    Eigen::Quaterniond odom_orient_;                                    // orient

    //Eigen::Vector2d current_pt_,current_vel_,current_acc_;            // current state

    Eigen::Vector2d start_pt_, start_vel_, start_acc_, start_yaw_;      // start state
    Eigen::Vector2d end_pt_, end_vel_, end_acc_;                        // target state

    // global planners
    //Astar::Ptr global_planner_astar_;
    JPS::Ptr global_planner_astar_;
    KinodynamicAstar::Ptr global_planner_kino_astar_;
    PolynomialTraj::Ptr global_planner_oneshot_;

    // bspline optimizer
    BsplineOptimizerESDF::Ptr bspline_optimizer_esdf_;
    BsplineOptimizerAstar::Ptr bspline_optimizer_rebound_; 

    // performance time
    double dur_;
    ros::WallTime t1_, t2_;

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector2d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector2d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    bool adjustStartAndTargetPoint( Eigen::Vector2d &target_pt, Eigen::Vector2d &start_pt);
public:
    InterPlanner(){};
    ~InterPlanner(){};

    // global plan data                                               
    GlobalData global_data_;

    // subgoal traj
    MidData mid_data_;

    //enum class PlanState { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };
    enum class OptimizerType { GRADIENT_ASTAR = 1, GRADIENT_ESDF = 2};

    void init(ros::NodeHandle & nh);

    /* subscribe callbacks */
    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    /* global plan service */
    bool makeGlobalPlanService(arena_plan_msgs::MakeGlobalPlan::Request  &req, arena_plan_msgs::MakeGlobalPlan::Response &res);

    bool makeGlobalPlan(Eigen::Vector2d start_pos,Eigen::Vector2d end_pos);

    /* subgoal service */
    bool makeOneshotPlan(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                                const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc);

    bool makeSubgoal(Eigen::Vector2d curr_pos, Eigen::Vector2d curr_vel, double T_subgoal=3.0);

    /* global plan method */
    bool planGlobalTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                      const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc, OptimizerType type_optimizer=OptimizerType::GRADIENT_ASTAR);



    bool planOneshotTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                      const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc,OptimizerType type_optimizer=OptimizerType::GRADIENT_ASTAR);

    bool planKinoAstarTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc, const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, OptimizerType type_optimizer=OptimizerType::GRADIENT_ESDF);

    bool planAstarTraj(Eigen::Vector2d &start_pos,Eigen::Vector2d &end_pos, OptimizerType type_optimizer=OptimizerType::GRADIENT_ASTAR);

    /* optimization */
    bool optimizePath(double ts,vector<Eigen::Vector2d> point_set, vector<Eigen::Vector2d> start_end_derivatives, UniformBspline &bspline_traj, OptimizerType type_optimizer=OptimizerType::GRADIENT_ASTAR);

    /* visualization */
    void visualizePath(const vector<Eigen::Vector2d> path, const ros::Publisher & pub);

    void visualizePoints(const vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);

    /* helper */
    bool checkCollision(const Eigen::Vector2d &pos);

    bool checkColiisionSegment(Eigen::Vector2d pt2, Eigen::Vector2d pt1);

    bool findCollisionWithinSegment(const Eigen::Vector2d &pt1,const Eigen::Vector2d &pt2,vector<Eigen::Vector2d> &inter_points);

    bool getPointSet(const vector<Eigen::Vector2d> &path, vector<Eigen::Vector2d> &point_set, double &ts);

    typedef std::shared_ptr<InterPlanner> Ptr;

};









#endif