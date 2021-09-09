#ifndef _SPACIAL_HORIZON_NODE_H_
#define _SPACIAL_HORIZON_NODE_H_

#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <vector>


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

#include <geometry_msgs/Twist.h>

#define PI 3.14159265

class SpacialHorizon{
private:
    //enum FSM_EXEC_STATE {INIT, WAIT_GOAL};
    /* planning data */
    bool have_goal_, have_odom_;

    geometry_msgs::PoseStamped ps_odom;

    Eigen::Vector2d odom_pos_, odom_vel_, initial_pose_; 
    Eigen::Quaterniond odom_orient_;
    double odom_dir_;

    Eigen::Vector2d start_pos_, start_vel_;  
    Eigen::Vector2d end_pos_, end_vel_;

    // subscriber
    ros::Subscriber goal_sub_, odom_sub_, initialPose_sub_, amcl_pose_sub_;

    // publisher
    ros::Publisher subgoal_DRL_pub_, globalPlan_DRL_pub_;

    // vis publisher
    ros::Publisher vis_subgoal_drl_pub_;
    ros::Publisher vis_goal_pub_;
    ros::Publisher vis_global_path_pub_;
    //ros::Publisher vis_local_path_pub_;

    // plan with global path from move base
    nav_msgs::GetPlan global_plan;

    /* parameters */
    double goal_tolerance_;         // meter
    double subgoal_tolerance_;      // meter
    double subgoal_pub_period_;
    double planning_horizen_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer subgoal_DRL_timer_;
    
    /* ros related callback*/
    void amcl_poseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);
    void handle_initial_pose(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);

    bool getSubgoalSpacialHorizon(Eigen::Vector2d &subgoal);
    void updateSubgoalDRLCallback(const ros::TimerEvent &e);

    /* get global plan from move base */
    void getGlobalPath_MoveBase();
    void fillPathRequest(nav_msgs::GetPlan::Request &request);
    void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);

    /* Visualization */
    void visualizePoints(const std::vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);
    void visualizeGlobalPath(const ros::Publisher & pub);

public:
    SpacialHorizon(/* args */){}
    ~SpacialHorizon(){}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif


