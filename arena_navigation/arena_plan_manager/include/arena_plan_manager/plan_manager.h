#ifndef PLAN_MANAGER_H
#define PLAN_MANAGER_H

#include <random> // for test

#include <Eigen/Eigen>
#include <iostream>

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>  // goal, subgoal needs time stamp
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/Marker.h>
#include <arena_plan_msgs/RobotStateStamped.h>

#include <arena_intermediate_planner/intermediate_planner.h>





class PlanManager{

private:
    /*
        INIT: wait for odom
        WAIT_GOAL: wait for goal(global goal)
        GEN_NEW_GLOBAL: generate new global path
        REPLAN_MID: replan for a new subgoal
        EXEC_LOCAL: no interference, let local planner do his job
    */
    enum FSM_EXEC_STATE { INIT, WAIT_GOAL, GEN_NEW_GLOBAL, REPLAN_MID, EXEC_LOCAL };
    enum MODE_TYPE { TRAIN = 1, TEST = 2};


    /* planning utils */
    InterPlanner::Ptr inter_planner_;

    /* parameters */
    int mode_;  // 1 train , 2 test
    double timeout_goal_;
    double timeout_subgoal_;
    double dist_tolerance_;
    double dist_lookahead_;
    
    /* planning data */
    bool have_goal_, have_odom_;
    FSM_EXEC_STATE exec_state_;

    //geometry_msgs::PoseStamped goal_;              // global goal
    //geometry_msgs::PoseStamped subgoal_;           // mid goal(waypoint/subgoal)
    
    Eigen::Vector2d odom_pt_, odom_vel_;  
    Eigen::Vector2d start_pt_, start_vel_;  
    Eigen::Vector2d end_pt_, end_vel_;

    Eigen::Vector2d curr_landmark_;

    //Eigen::Vector2d subgoal_;
    //std::vector<Eigen::Vector2d> global_path_;

    /* timing */
    ros::Time goal_start_time_;
    ros::Time subgoal_start_time_;
    ros::Time landmark_start_time_;

    double landmark_timeout_;
    

    
    /* ROS utils */
    ros::NodeHandle node_;
    
    ros::Timer exec_timer_;//, safety_timer_, vis_timer_;
    
    // subscriber
    ros::Subscriber goal_sub_, odom_sub_;
    
    // publisher
    ros::Publisher global_plan_pub_;
    ros::Publisher subgoal_pub_;
    ros::Publisher robot_state_pub_;


    /* helper functions */
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();


    /* ROS functions */
    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    
    void execFSMCallback(const ros::TimerEvent& e);
    //void checkCollisionCallback(const ros::TimerEvent& e);

    
    void publishGlobalPath(const std::vector<Eigen::Vector2d> & global_path);

    void publishSubgoal(const Eigen::Vector2d & subgoal);

    /* test purpose*/
    

public:
    PlanManager(/* args */) {
    }

    ~PlanManager() {
    }

    void init(ros::NodeHandle & nh);

    


};


#endif