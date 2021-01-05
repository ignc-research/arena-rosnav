
#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <angles/angles.h>

#include "ros/console.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include "message_filters/subscriber.h"


class FakeLocalization{


private:
    ros::NodeHandle m_nh;
    ros::Publisher m_posePub;
    
    // tf related
    tf2_ros::TransformBroadcaster       *m_tfServer;
    tf2_ros::TransformListener          *m_tfListener;
    tf2_ros::Buffer                     *m_tfBuffer;
    
    tf2_ros::MessageFilter<nav_msgs::Odometry>* filter_;
    ros::Subscriber stuff_sub_; 
    message_filters::Subscriber<nav_msgs::Odometry>* filter_sub_;

    // odom & pose
    nav_msgs::Odometry  m_basePosMsg;
    geometry_msgs::PoseWithCovarianceStamped      m_currentPos;
    

    // param for initialization
    double  delta_x_, delta_y_, delta_yaw_;
    bool    m_base_pos_received;
    double transform_tolerance_;
    tf2::Transform m_offsetTf;

    //parameter for what odom to use
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string global_frame_id_;
    std::string base_pose_ground_truth_;

public:
    FakeLocalization(const ros::NodeHandle node);

    ~FakeLocalization(void);

    
    void stuffFilter(const nav_msgs::OdometryConstPtr& odom_msg);
    
    void update(const nav_msgs::OdometryConstPtr& message);

};