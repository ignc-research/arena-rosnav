#ifndef _FORD_MSGS_UTIL_HPP
#define _FORD_MSGS_UTIL_HPP

#include <ros/ros.h>
#include "ford_msgs/PedTraj.h"
#include "ford_msgs/PedTrajVec.h"
#include "ford_msgs/Pose2DStamped.h"

namespace ford_msgs{
typedef std::vector<ford_msgs::Pose2DStamped> PoseVec;
bool backInterpolate(const PoseVec& posVec, const ros::Time& stamp, PoseVec::const_reverse_iterator& rIt, ford_msgs::Pose2DStamped& pose2DStamped)
{
    if (rIt->header.stamp < stamp){
        // The timestamp of the rIt already pass the stamp (in reverse)
        return false;
    }

    for(;(rIt+1) != posVec.rend();++rIt){ //Iterate through the posVec in reverse starting from rIt
        if( (rIt+1)->header.stamp <= stamp ){ //If the next timestamp is before the interpolation timestamp
            //Do interpolation
            ros::Duration full_duration = rIt->header.stamp - (rIt+1)->header.stamp;
            ros::Duration part_duration = rIt->header.stamp - stamp;
            double ratio = part_duration.toSec()/full_duration.toSec();
            pose2DStamped.pose.x = rIt->pose.x - ((rIt+1)->pose.x - rIt->pose.x)*ratio; 
            pose2DStamped.pose.y = rIt->pose.y - ((rIt+1)->pose.y - rIt->pose.y)*ratio; 
            pose2DStamped.velocity.x = rIt->velocity.x - ((rIt+1)->velocity.x - rIt->velocity.x)*ratio; 
            pose2DStamped.velocity.y = rIt->velocity.y - ((rIt+1)->velocity.y - rIt->velocity.y)*ratio; 
            pose2DStamped.header.frame_id = rIt->header.frame_id;
            pose2DStamped.header.stamp = stamp;
            return true;
        }
    }
    // Reach the end of the posVec (in reverse)
    return false;
}

bool frontInterpolate(const PoseVec& posVec, const ros::Time& stamp, PoseVec::const_iterator& it, ford_msgs::Pose2DStamped& pose2DStamped)
{
    if (it->header.stamp > stamp){
        return false;
    }

    for(;(it+1) != posVec.end();++it){ //Iterate through the posVec starting from it
        if( (it+1)->header.stamp >= stamp ){ //If the next timestamp is after the interpolation timestamp
            //Do interpolation
            ros::Duration full_duration = (it+1)->header.stamp - it->header.stamp;
            ros::Duration part_duration = stamp - it->header.stamp;
            double ratio = part_duration.toSec()/full_duration.toSec();
            pose2DStamped.pose.x = it->pose.x + ((it+1)->pose.x - it->pose.x)*ratio; 
            pose2DStamped.pose.y = it->pose.y + ((it+1)->pose.y - it->pose.y)*ratio; 
            pose2DStamped.velocity.x = it->velocity.x + ((it+1)->velocity.x - it->velocity.x)*ratio; 
            pose2DStamped.velocity.y = it->velocity.y + ((it+1)->velocity.y - it->velocity.y)*ratio; 
            pose2DStamped.header.frame_id = it->header.frame_id;
            pose2DStamped.header.stamp = stamp;
            return true;
        }
    }
    // Reach the end of the posVec
    return false;
}

PoseVec downsample(const PoseVec& pose_vec_in, ros::Duration duration, bool from_last = true)
{ //Downsample a pedTraj message according to duration, starting from the latest point and use interpolation.
    if (duration <= ros::Duration(0.0)){
        return pose_vec_in;
    }
    PoseVec pose_vec_out; 
    if (from_last){
        ros::Time interpolation_time = pose_vec_in.back().header.stamp;
        PoseVec::const_reverse_iterator rIt = pose_vec_in.rbegin();
        ford_msgs::Pose2DStamped pose2DStamped;
        while (backInterpolate(pose_vec_in,interpolation_time,rIt,pose2DStamped)){
            pose_vec_out.push_back(pose2DStamped);
            interpolation_time -= duration;
        }
        std::reverse(pose_vec_out.begin(),pose_vec_out.end());
    }
    else{
        ros::Time interpolation_time = pose_vec_in.front().header.stamp;
        PoseVec::const_iterator it = pose_vec_in.begin();
        ford_msgs::Pose2DStamped pose2DStamped;
        while (frontInterpolate(pose_vec_in,interpolation_time,it,pose2DStamped)){
            pose_vec_out.push_back(pose2DStamped);
            interpolation_time += duration;
        }
    }
    return pose_vec_out;
}

ford_msgs::PedTraj downsample(const ford_msgs::PedTraj& ped_traj, ros::Duration duration, bool from_last = true)
{ //Downsample a pedTraj message according to duration, starting from the latest point and use interpolation.
    if (duration <= ros::Duration(0.0)){
        return ped_traj;
    }
    ford_msgs::PedTraj ped_traj_out;
    ped_traj_out = ped_traj;
    ped_traj_out.traj = downsample(ped_traj.traj,duration,from_last);
    return ped_traj_out;
}

ford_msgs::PedTrajVec downsample(const ford_msgs::PedTrajVec& pedTrajVec,ros::Duration duration, bool from_last = true)
{
    if (duration <= ros::Duration(0.0)){
        return pedTrajVec;
    }
    ford_msgs::PedTrajVec pedTrajVec_out;
    std::vector<ford_msgs::PedTraj>::const_iterator it;
    for (it = pedTrajVec.ped_traj_vec.begin(); it != pedTrajVec.ped_traj_vec.end(); ++it){
        pedTrajVec_out.ped_traj_vec.push_back(downsample(*it,duration,from_last));
    }
    return pedTrajVec_out;
}

} /*namespace ford_msgs*/

#endif /*_FORD_MSGS_UTIL_HPP*/