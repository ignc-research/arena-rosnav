#ifndef _DYNAMIC_PLAN_FSM_H_
#define _DYNAMIC_PLAN_FSM_H_


#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <vector>


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Twist.h>

#include <arena_timespace_planner/dynamic_plan_manager.h>


class DynamicReplanFSM{
private:
    enum FSM_EXEC_STATE { INIT, WAIT_GOAL, GEN_NEW_GLOBAL, REPLAN_MID, EXEC_LOCAL };
    enum MODE_TYPE { TRAIN = 1, TEST = 2};
    int mode_;  // 1 train , 2 test

     /* planning utils */
    DynamicPlanManager::Ptr planner_manager_;

    /* planning data */
    bool have_goal_, have_odom_;
    FSM_EXEC_STATE exec_state_;

    Eigen::Vector2d odom_pos_, odom_vel_; 
    Eigen::Quaterniond odom_orient_;
    double odom_dir_;

    Eigen::Vector2d start_pos_, start_vel_;  
    Eigen::Vector2d end_pos_, end_vel_;

    std::vector<Eigen::Vector2d> landmark_wps_;
    std::vector<Eigen::Vector2d> sample_wps_;
    size_t curr_wp_index_;

    Eigen::Vector2d mid_target_;
    //Eigen::Vector2d local_target_;
    TargetTrajData target_traj_data_;


    /* parameters */
    double planning_horizen_, planning_horizen_time_;
    double goal_tolerance_;         // meter
    double subgoal_tolerance_;      // meter
    double t_replan_thresh_;        // sec
    int subgoal_drl_mode_;       // 0: spacial horizon, 1:time_astar
    double subgoal_pub_period_;
    int mid_replan_count_= 0;

    int in_collision_cnt=0;


    /* ROS utils */
    ros::NodeHandle node_;
    
    ros::Timer exec_timer_, safety_timer_;// vis_timer_;
    ros::Timer traj_tracker_timer_;
    ros::Timer subgoal_DRL_timer_;
    //ros::Timer dynamic_occ_map_timer_;
    
    // subscriber
    ros::Subscriber goal_sub_, odom_sub_;

    // publisher
    ros::Publisher cmd_vel_pub_;
    ros::Publisher subgoal_DRL_pub_;
    ros::Publisher global_plan_pub_;
    
    // vis publisher
    ros::Publisher vis_goal_pub_;
    ros::Publisher vis_subgoal_drl_pub_;
    ros::Publisher vis_global_path_pub_,vis_landmark_pub_;
    ros::Publisher vis_triangle_pub_,vis_timed_astar_path_pub_,vis_timed_astar_wp_pub_;
    ros::Publisher vis_local_path_pub_,vis_local_multi_path_pub_;

    /* ros related callback*/
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);

    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void trackTrajCallback(const ros::TimerEvent &e);
    void updateSubgoalDRLCallback(const ros::TimerEvent &e);
    //void updateDynamicMapCallback(const ros::TimerEvent& e);
    
    bool planFromCurrentTraj(const int trial_times=1 /*=1*/);

    bool checkCollision();
    

    Eigen::Vector2d getNextWaypoint();
    bool getSubgoalSpacialHorizon(Eigen::Vector2d &subgoal);
    bool getSubgoalTimedAstar(Eigen::Vector2d &subgoal);
    bool getSubgoalSimpleSample(Eigen::Vector2d &subgoal);
    bool getSubgoalGlobal(Eigen::Vector2d &subgoal);


    /* helper functions */
    void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
    void printFSMExecState();

    /* viusalization*/
    void visualizePoints(const std::vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);
    void visualizeLines(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);
    void visualizePath(const std::vector<Eigen::Vector2d> path, const ros::Publisher & pub);

public:
    DynamicReplanFSM(/* args */){}
    ~DynamicReplanFSM(){}

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};








#endif