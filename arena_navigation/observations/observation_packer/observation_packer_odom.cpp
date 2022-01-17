#include <observations/Observation.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "Transform2D.h"
#include <common/f_math.h>
#include <signal.h>
#include <math.h>

#define MAX_LASER_DISTANCE 3.5
#define forEach BOOST_FOREACH

using std::cout;
using std::endl;

// observation
float laser_data[360];// laser scanner
float distance_to_goal = 0; // distance to goal in meters
float angle_to_goal = 0; // angle to goal in degree
float d_t_g = 0;
float a_t_g = 0;

const Transform2D relative_robot_transform(0, 0, 0); // base from which relative distance, angle is calculated
Transform2D relative_goal_transform; // goal transform relative to current robot pose
Transform2D absolute_goal_transform;

Transform2D goal;
Transform2D robot_state;

bool goal_callback_called;
bool odom_callback_called;
bool laser_callback_called;

// last messages
Transform2D odom_transform;// last odometry transform received by callback

void setAbsoluteGoal(const zVector2D & new_pos){
	absolute_goal_transform.setPosition(new_pos);
	relative_goal_transform = absolute_goal_transform;
	relative_goal_transform.toLocalOf(odom_transform);
}

void quaternionFromMsg(tf2::Quaternion & q, const geometry_msgs::Quaternion & q_msg)
{
		q[0] = q_msg.x;
		q[1] = q_msg.y;
		q[2] = q_msg.z;
		q[3] = q_msg.w;
}

void calculateGoal(){
	relative_robot_transform.getDistance(relative_goal_transform, distance_to_goal, angle_to_goal);
	angle_to_goal = f_deg(angle_to_goal);
}

void updateAbsoluteGoal(){
	absolute_goal_transform = odom_transform.getLocalTranslate(relative_goal_transform.position);
}

void goal_callback(const geometry_msgs::PoseStampedPtr & msg){
	float x = msg->pose.position.x - odom_transform.position.x;
	float y = msg->pose.position.y - odom_transform.position.y;
	relative_goal_transform.set(x, y, 0);
	// msg->pose.position.x = - msg->pose.position.x + odom_transform.position.x;
	// msg->pose.position.y = + msg->pose.position.y - odom_transform.position.y;

	// relative_goal_transform.setFromMsg(msg->pose);
	goal.setFromMsg(msg->pose);
	goal_callback_called = true;
	// zVector2D goal2d(msg->pose.position.x, msg->pose.position.y);
	// zVector2D robot2d(odom_transform.position.x, odom_transform.position.y);

	// // ????
	// zVector2D robot_rot = zVector2D(cos(odom_transform.theta), sin(odom_transform.theta));
	// // ????

	// zVector2D from_to = goal2d-robot2d;
	// from_to.rotate(-robot_rot.getRotation());
	// float x = from_to.x;
	// // float y = -from_to.y;// invert y axis because screen coordinates
	// float y = from_to.y;// invert y axis because screen coordinates
	// relative_goal_transform.set(x, y, 0);

	// // relative_goal_transform.set(msg->pose.position.x, msg->pose.position.y, 0);
	// goal.set(msg->pose.position.x, msg->pose.position.y, 0);	
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr & msg){
	for(int i = 0; i < msg->ranges.size(); i++){
		laser_data[i] = msg->ranges[i];
		if(laser_data[i] == 0){
			laser_data[i] = MAX_LASER_DISTANCE;
		}
	}
	laser_callback_called = true;
}

void updateRelativeWithAbsoluteGoal(){
	relative_goal_transform = absolute_goal_transform.getToLocalOf(odom_transform);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr & msg){
	odom_transform.setFromMsg(msg->pose.pose);
	odom_callback_called = true;
}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedPtr & msg){
	odom_transform.setFromMsg(msg->pose.pose);
	robot_state.setFromMsg(msg->pose.pose);
	odom_callback_called = true;
}

// void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg){
// 	relative_robot_transform.setFromMsg(msg->pose.pose);
// }

void cal_d_a_to_goal(){
	// float x = goal.position.x - robot_state.position.x;
	// float y = goal.position.y - robot_state.position.y;
	// return sqrt(x*x + y*y);
	robot_state.getDistance(goal, d_t_g, a_t_g);
	a_t_g = f_deg(a_t_g);
}

float cal_angle_to_goal(){
	float x = goal.position.x - robot_state.position.x;
	float y = goal.position.y - robot_state.position.y;
	float tmp = atan2(y, x);
	return tmp - robot_state.theta + 4*M_PI;
}

void packMsg(observations::Observation & obs){
	// obs.observation[0] = distance_to_goal;
	// obs.observation[1] = angle_to_goal;
	cal_d_a_to_goal();
	obs.observation[0] = d_t_g;
	obs.observation[1] = a_t_g;
	for(int i =0; i < 360; i++)
		obs.observation[i+2] = laser_data[i];
}

volatile bool RUN=true;
void sigint_handler(int a){
	RUN=false;
}

int main(int argc, char ** argv)
{
	puts("Initializing ros node...");
	signal(SIGINT, sigint_handler);
	ros::init(argc, argv, "observeration_packer", ros::init_options::NoSigintHandler);
	ros::NodeHandle n;
	puts("Subscribing to /subgoal ...");
	ros::Subscriber sub_obj = n.subscribe("subgoal", 1, &goal_callback);
	// puts("Subscribing to /initial_pose ...");
	// ros::Subscriber sub_ipose = n.subscribe("initialpose", 1, &initialpose_callback);
	// puts("Subscribing to /goal ...");
	// ros::Subscriber sub_obj = n.subscribe("goal", 1, &goal_callback);
	puts("Subscribing to /scan ...");
	ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan>("scan", 1, laser_callback);
	// puts("Subscribing to /odom ...");
	// ros::Subscriber sub_odom = n.subscribe<nav_msgs::Odometry>("odom", 1, odom_callback);
	puts("Subscribing to /amcl_pose ...");
	ros::Subscriber sub_odom = n.subscribe("amcl_pose", 1, amcl_callback);
	puts("Advertising /observation ...");
	ros::Publisher pub = n.advertise<observations::Observation>("observation", 1);
	ros::Rate publish_rate(10);
	// init laser data
	for(int i = 0; i < 360; i++){
		laser_data[i] = MAX_LASER_DISTANCE;
	}
	distance_to_goal = 0;
	angle_to_goal = 0;
	puts("Running...");
	while(RUN){
		// puts("Packing observations");
		goal_callback_called = false;
		odom_callback_called = false;
		laser_callback_called = false;
		ros::spinOnce();
		// updating absolute goal position if either odom or goal callback
		if(goal_callback_called){
			updateAbsoluteGoal();
		}
		else // updating relative goal position with odometry only if no goal callback and odom callback
		if(odom_callback_called){
			updateRelativeWithAbsoluteGoal();
		}

		// if any callback, send new observation message
		if(goal_callback_called || odom_callback_called || laser_callback_called){
			calculateGoal();
			// packing current observation
			observations::Observation msg;
			packMsg(msg);
			pub.publish(msg);
		}

		// slow down
		publish_rate.sleep();
	}

	// shutdown
	puts("Shutting down ros...");
	ros::shutdown();
	puts("Done!");
}
