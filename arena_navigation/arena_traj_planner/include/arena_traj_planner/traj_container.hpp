#ifndef _TRAJ_CONTAINER_H_
#define _TRAJ_CONTAINER_H_


#include <Eigen/Eigen>
#include <vector>

#include <ros/ros.h>

#include "arena_traj_planner/bspline/uniform_bspline.h"
#include <arena_traj_planner/polynomial/polynomial_traj.h>

struct OneTrajDataOfSwarm
  {
    /* info of generated traj */

    int drone_id;
    double duration_;
    ros::Time start_time_;
    Eigen::Vector2d start_pos_;
    UniformBspline position_traj_;
  };

  typedef std::vector<OneTrajDataOfSwarm> SwarmTrajData;
  
#endif