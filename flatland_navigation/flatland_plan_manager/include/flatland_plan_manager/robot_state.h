#ifndef _ROBOT_STATE_H_
#define _ROBOT_STATE_H_

#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include "tf/transform_datatypes.h"

#include <flatland_plan_msgs/RobotStateStamped.h>
#include <flatland_plan_msgs/RobotState.h>

struct RobotState;
typedef RobotState *RobotStatePtr;

struct RobotState
{

    Eigen::Vector2d pose2d;
    double theta;
    Eigen::Vector2d vel2d;
    double w;

    RobotState(geometry_msgs::Pose pose3d)
    {   // position
        pose2d(0) = pose3d.position.x;
        pose2d(1) = pose3d.position.y;
        // yaw
        tf2::Matrix3x3 mat(tf2::Quaternion(pose3d.orientation.x, pose3d.orientation.y, pose3d.orientation.z, pose3d.orientation.w));
        double roll, pitch, yaw;
        mat.getRPY(roll,pitch,yaw); // yaw pitch roll
        theta = yaw;
        // velocity
        vel2d(0) = 0.0;
        vel2d(1) = 0.0;
        // angle velocity
        w = 0.0;
    }

    RobotState(nav_msgs::Odometry odom){
        pose2d(0) = odom.pose.pose.position.x;
        pose2d(1) = odom.pose.pose.position.y;
        vel2d(0) = odom.twist.twist.linear.x;
        vel2d(1) = odom.twist.twist.linear.y;
        w=odom.twist.twist.angular.z;

        tf2::Matrix3x3 mat(tf2::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
        double roll, pitch, yaw;
        mat.getRPY(roll,pitch,yaw); // yaw pitch roll
        theta = yaw;
    }

    RobotState(Eigen::Vector2d p,double yaw,Eigen::Vector2d v,double wz){
        pose2d=p;
        theta=yaw;
        vel2d=v;
        w=wz;
    }

    geometry_msgs::PoseStamped to_PoseStampted(){
        geometry_msgs::PoseStamped pose_stamped;
       
        pose_stamped.header.stamp=ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x=pose2d(0);
        pose_stamped.pose.position.y=pose2d(1);
        pose_stamped.pose.position.z=0.0;
        //tf2::Quaternion quat_tf;
        //quat_tf.setRPY(0.0, 0.0, theta);
        geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta);
        pose_stamped.pose.orientation=quat_msg;
        return pose_stamped;
    }

    flatland_plan_msgs::RobotStateStamped toRobotStateStamped(){
        flatland_plan_msgs::RobotStateStamped state_stamped;
        flatland_plan_msgs::RobotState state;
        //header
        state_stamped.header.stamp=ros::Time::now();
        state_stamped.header.frame_id = "map";
        
        // pose
        state.pose.position.x=pose2d(0);
        state.pose.position.y=pose2d(1);
        state.pose.position.z=0.0;
        geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, theta);
        state.pose.orientation=quat_msg;

        //twist
        state.twist.linear.x=vel2d(0);
        state.twist.linear.y=vel2d(1);
        state.twist.linear.z=0.0;
        state.twist.angular.x=0.0;
        state.twist.angular.y=0.0;
        state.twist.angular.z=w;
        
        // state
        state_stamped.state=state;

        return state_stamped;
    }

    RobotState();
    ~RobotState();
};

#endif