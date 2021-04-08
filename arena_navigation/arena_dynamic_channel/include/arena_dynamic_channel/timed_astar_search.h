#ifndef _TIMED_ASTAR_SEARCH_H
#define _TIMED_ASTAR_SEARCH_H
#pragma once

#include <Eigen/Eigen>
#include "arena_dynamic_channel/dynamic_obstacle.h"
#include "arena_dynamic_channel/timed_astar/timed_astar_anytime.h"


#include "arena_dynamic_channel/timed_astar/data.h"

#include <arena_mapping/mapping.h>
#include <arena_traj_planner/bspline/uniform_bspline.h>


using namespace timed_astar;

class TimedAstarSearch{
private:
    // ros private node
    ros::NodeHandle node_;

    // timed astar
    TimedAstar::Ptr timed_astar_planner_;

    TimedAstarParam tap_;

    // obstacle info
    std::vector<DynamicObstacleInfo::Ptr> obs_info_provider_;

    // gridmap
    GridMap::Ptr grid_map_;
    Eigen::Vector2d occ_map_origin_,occ_map_size_2d_;

    // visual
    ros::Publisher visited_points_pub_;
    visualization_msgs::MarkerArray target_marker_list_;
    
    // helper
    inline void boundPosition(Eigen::Vector2d & pos);
    inline Eigen::Vector2d Vec2dToEigen2d(Vec2d pt_vec);
    inline Vec2d Eigen2dToVec2d(Eigen::Vector2d pt_eigen);

public:
    TimedAstarSearch(){}
    ~TimedAstarSearch();
    typedef std::shared_ptr<TimedAstarSearch> Ptr;

    void init(ros::NodeHandle & nh,GridMap::Ptr grid_map, std::vector<DynamicObstacleInfo::Ptr> obs_info_provider);

    bool stateTimeAstarSearch(const Eigen::Vector2d & start_pos,const Eigen::Vector2d & start_vel,const double & start_dir,const Eigen::Vector2d & end_pos, double & ts, double &local_time_horizon, std::vector<Eigen::Vector2d> &point_set,std::vector<Eigen::Vector2d> &start_end_derivatives,std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &line_sets);

    void getTriangleEdges(std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &line_sets);

    std::vector<Eigen::Vector2d> getWaypoints();

    void visualizePoints(const std::vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);
    
};

inline void TimedAstarSearch::boundPosition(Eigen::Vector2d & pos) {
  pos(0) = std::max(std::min(pos(0), occ_map_size_2d_(0)), occ_map_origin_(0));
  pos(1) = std::max(std::min(pos(1), occ_map_size_2d_(1)), occ_map_origin_(1));
}

inline Eigen::Vector2d TimedAstarSearch::Vec2dToEigen2d(Vec2d pt_vec){
    return Eigen::Vector2d(pt_vec.x,pt_vec.y);
}

inline Vec2d TimedAstarSearch::Eigen2dToVec2d(Eigen::Vector2d pt_eigen){
    return Vec2d(pt_eigen(0),pt_eigen(1));
}









#endif
