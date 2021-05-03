#include "arena_dynamic_channel/dynamic_obstacle.h"

DynamicObstacleInfo::DynamicObstacleInfo(ros::NodeHandle &nh, std::string topic_name,GridMap::Ptr grid_map){
    node_=nh;
    topic_name_=topic_name;
    // get  radius param
    node_.param("dynamic_obstacle/obstacle_radius",    obstacle_radius_,    0.3);
    node_.param("dynamic_obstacle/inflation_radius",   inflation_radius_,   0.3);
    node_.param("dynamic_obstacle/prediction_forward_time", prediction_forward_time_,   0.3);
    node_.param("dynamic_obstacle/sensor_range",   sensor_range_,   3.0);
    radius_=obstacle_radius_+inflation_radius_;

    // gridmap
    this->grid_map_=grid_map;
    resolution_=grid_map_->getResolution();

    // init subscriber for obstacles state
    ros::NodeHandle public_nh_;
    obs_odom_sub_=public_nh_.subscribe(topic_name_, 1, &DynamicObstacleInfo::updateOdomCallback,this);
    //std::string vel_topic_name=topic_name_+"_vel";
    //obs_vel_pub_ =public_nh_.advertise<std_msgs::Float64>(vel_topic_name,1);

    // init subscriber for robot state
    robot_odom_sub_ = public_nh_.subscribe("odom", 1, &DynamicObstacleInfo::updateRobotOdomCallback,this);

    // init pos, vel, is_init
    pos_=Eigen::Vector2d::Zero();
    vel_=Eigen::Vector2d::Zero();
    is_init_=true;
    last_ros_time_=ros::Time::now();

}
    
void DynamicObstacleInfo::updateOdomCallback(visualization_msgs::MarkerArray::ConstPtr msg){
        
    // for nav_msgs::Odometry::ConstPtr msg
    //curr_pos(msg->pose.pose.position.x,msg->pose.pose.position.y);
    if((ros::Time::now()-last_ros_time_).toSec()<0.01){
        return;
    }
    last_ros_time_= ros::Time::now();

    Eigen::Vector2d curr_pos(msg->markers[0].pose.position.x,msg->markers[0].pose.position.y); 
        
    double curr_time=msg->markers[0].header.stamp.toSec();
    if(curr_time==last_time_){
            return;
    }
        
    if(is_init_)
    {
        vel_= Eigen::Vector2d(0.0,0.0);
        is_init_=false;
    }else{
        vel_=(curr_pos-pos_)/(curr_time-last_time_);
    }
    
    pos_=curr_pos;
    last_time_=curr_time;

    if(!is_init_){
        updateDynamicOcc();
    }
    //std_msgs::Float64 velocity;
    //velocity.data= (double)vel_.norm();
    //obs_vel_pub_.publish(velocity);

}

void DynamicObstacleInfo::updateRobotOdomCallback(const nav_msgs::OdometryConstPtr msg){

    robot_pos_=Eigen::Vector2d(msg->pose.pose.position.x,msg->pose.pose.position.y);
    robot_vel_=Eigen::Vector2d(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
}

void DynamicObstacleInfo::updateDynamicOcc(){
    // reset old occ
    for(size_t i=0;i<last_occ_set_.size();++i){
        grid_map_->setDynamicOccupancy(last_occ_set_[i],0);
    }

    // check if obstacle is within the sensor range
    double dist_to_robot=(pos_-robot_pos_).norm();
    if(dist_to_robot > sensor_range_){
        return;
    }
    // calculate forward_time
    double forward_time= dist_to_robot/(vel_-robot_vel_).norm();
    forward_time = std::min(forward_time,prediction_forward_time_);
    Eigen::Vector2d vel=vel_;
    Eigen::Vector2d pos=pos_+forward_time*vel_;  //pos after 0.1s
    
    
    // add new occ
    
    last_occ_set_.clear();
    //int i=0;
    //std::cout<<"topic="<<topic_name_<<"  pos="<<pos<<std::endl;

    for(double x=pos(0)-radius_; x<pos(0)+radius_; x+=resolution_){
        for(double y=pos(1)-radius_; y<pos(1)+radius_; y+=resolution_){
            for(double t=std::max(0.0,std::min(0.1,forward_time-0.3));t<forward_time;t+=0.1){
                // curr_pos
                Eigen::Vector2d pt(x,y);
                // future pos at time=t
                pt=pt+vel*t;
                grid_map_->setDynamicOccupancy(pt,1);
                last_occ_set_.push_back(pt);
                //i++;
                //if(i==1){
                //    std::cout<<"topic="<<topic_name_<<"  pt="<<pt<<std::endl;
                //}
                
             
                
            }
        }
    } 
}