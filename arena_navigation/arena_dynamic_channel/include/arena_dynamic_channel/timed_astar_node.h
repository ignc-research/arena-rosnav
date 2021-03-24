#ifndef _TIMED_ASTAR_H
#define _TIMED_ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/master.h>

#include"boost/algorithm/string.hpp"

#include <memory>
#include <map>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>


#include <arena_mapping/mapping.h>

#include "arena_dynamic_channel/timed_astar/timed_astar_anytime.hpp"
#include "arena_dynamic_channel/timed_astar/timed_astar.hpp"

#include "arena_dynamic_channel/timed_astar/data.hpp"
#include "arena_dynamic_channel/graph/accessor.h"
#include "arena_dynamic_channel/graph/delaunator.h"


namespace timed_astar
{

//typedef std::shared_ptr<ObstacleInfo> ObstacleInfoPtr;

struct ObstacleInfo{
public:
    ros::NodeHandle node_;
    std::string topic_name_;
    ros::Subscriber obs_odom_sub_;

    Vec2d pos_;
    Vec2d vel_;
    double last_time_;
    bool is_init_;

    Vec2d getPosition(){
        return pos_;
    }

    Vec2d getVelocity(){
        return vel_;
    }

    ObstacleInfo(ros::NodeHandle &nh, std::string topic_name){
        node_=nh;
        topic_name_=topic_name;

        // init subscriber for obstacles state
        ros::NodeHandle public_nh_;
        obs_odom_sub_=public_nh_.subscribe(topic_name_, 1, &ObstacleInfo::updateOdomCallback,this);
        
        // init pos, vel, is_init
        pos_=Vec2d(0.0,0.0);
        vel_=Vec2d(0.0,0.0);
        is_init_=true;
    }
    
    void updateOdomCallback(visualization_msgs::MarkerArray::ConstPtr msg){
        
        // for nav_msgs::Odometry::ConstPtr msg
        //curr_pos(msg->pose.pose.position.x,msg->pose.pose.position.y);

        Vec2d curr_pos(msg->markers[0].pose.position.x,msg->markers[0].pose.position.y); 
        
        double curr_time=msg->markers[0].header.stamp.toSec();
        if(curr_time==last_time_){
            return;
        }
        
        if(is_init_){
            vel_= Vec2d(0.0,0.0);
            is_init_=false;
        }else{
            vel_=(curr_pos-pos_)/(curr_time-last_time_);
        }
        
        pos_=curr_pos;
        last_time_=curr_time;
        //std::cout<<topic_name_<<"vel="<<vel_<<std::endl;
    }

    ~ObstacleInfo(){std::cout<<"DELTETED obstacle info object "<<std::endl;}

    typedef std::shared_ptr<ObstacleInfo> Ptr;
};

class TimeAstarSearch{
private:
    // a instance of graph
    GraphPtr dt_graph_;
    TimedAstar::Ptr timed_astar_planner_;

    // current states of nodes (init, goal, obstacles)
    std::vector<double> coords_;
    std::vector<double> speeds_;
    std::vector<double> angles_;
    
    // map
    GridMap::Ptr grid_map_;
    Eigen::Vector2d occ_map_origin_,occ_map_size_2d_;
    

    // robot state variables
    Vec2d odom_pos_, odom_vel_;                               // odometry state
    Eigen::Quaterniond odom_orient_;                          // orient
    double odom_dir_;                                         // yaw angle 

    //robot params
    double robot_avg_vel_;                                              // robot avg velocity
    double robot_max_vel_;                                              // robot max velocity
    double radius_robot_;
    double radius_obs_;

    // plan variable
    Vec2d start_pt_,start_vel_;
    double start_dir_;
    Vec2d end_pt_;
    std::vector<Vec2d> waypoints_;
    bool have_odom_;
    bool have_goal_;
     
    // bspline
    //BsplineOptimizer::Ptr bspline_optimizer_;

    // obstacle info
    std::vector<ObstacleInfo::Ptr> obs_info_provider_;

    // ros private node
    ros::NodeHandle node_;

    // subscriber
    ros::Subscriber goal_sub_, odom_sub_;

    // publisher
    ros::Publisher vis_triangle_pub_, vis_goal_pub_, vis_wp_pub_,vis_path_pub_;

    /* timer */
    ros::Timer update_timer_; 
    ros::Timer vis_timer_; 

    // performance
    double dc_time_,max_dc_time_;
    int dc_update_num_=0;

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);

    void UpdateCallback(const ros::TimerEvent&);

    /* Helper */
    inline void boundPosition(Vec2d& pos);

    
    

public:

    TimeAstarSearch(){}
    ~TimeAstarSearch();

    typedef std::shared_ptr<TimeAstarSearch> Ptr;

    void init(ros::NodeHandle & nh);

    /* reset graph */
    void resetGraph();

    void StateTimeAstarSearch();

    /* visualization */
    void visualizePoints(const std::vector<Vec2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);

    void visualizeLines(const std::vector<std::pair<Vec2d,Vec2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);

    void visualizePath(const std::vector<Vec2d> path, const ros::Publisher & pub);

    void publishVisGraph();


    


    
    
};

inline void TimeAstarSearch::boundPosition(Vec2d& pos) {

  pos.x = std::max(std::min(pos.x, occ_map_size_2d_.x), occ_map_origin_.x);
  pos.y = std::max(std::min(pos.y, occ_map_size_2d_.y), occ_map_origin_.y);

}
}
#endif