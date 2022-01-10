/*
    Use global path of move_base and publish Subgoals based on SpacialHorizon algorithm
*/
#include <arena_spacial_horizon/spacial_horizon_node.h>

using std::cout;
using std::endl;
void SpacialHorizon::init(ros::NodeHandle &nh)
{   
    have_goal_ = false;
    have_odom_ = false;

    /*  fsm param  */
    node_=nh;
    node_.param("fsm/goal_tolerance",    goal_tolerance_, 0.5);
    node_.param("fsm/subgoal_tolerance", subgoal_tolerance_, 2.0);
    node_.param("fsm/subgoal_pub_period", subgoal_pub_period_, 0.5);
    node_.param("fsm/planning_horizen", planning_horizen_, 3.0);

    /* callback */
    //exec_timer_ = node_.createTimer(ros::Duration(0.01), &SpacialHorizon::execStateCallback, this);
    subgoal_DRL_timer_ = node_.createTimer(ros::Duration(0.05), &SpacialHorizon::updateSubgoalDRLCallback, this);

    /* ros communication with public node */
    ros::NodeHandle public_nh;  // sim1/goal
    goal_sub_ = public_nh.subscribe("goal", 1, &SpacialHorizon::goalCallback,this);
    odom_sub_ = public_nh.subscribe("odom", 1, &SpacialHorizon::odomCallback, this);
    // amcl_pose_sub_ = public_nh.subscribe("amcl_pose", 1, &SpacialHorizon::amcl_poseCallback, this);
    initialPose_sub_ = public_nh.subscribe("initialpose", 0, &SpacialHorizon::handle_initial_pose, this);

    subgoal_DRL_pub_  = public_nh.advertise<geometry_msgs::PoseStamped>("subgoal",10);
    globalPlan_DRL_pub_  = public_nh.advertise<nav_msgs::Path>("globalPlan",10);

    /* visualization */
    vis_global_path_pub_ = public_nh.advertise<nav_msgs::Path>("vis_global_path", 10, true);
    vis_goal_pub_	 = public_nh.advertise<visualization_msgs::Marker>("vis_goal", 20);
    vis_subgoal_drl_pub_ = public_nh.advertise<visualization_msgs::Marker>("vis_subgoal", 20);
}

void SpacialHorizon::handle_initial_pose(const geometry_msgs::PoseWithCovarianceStampedPtr& msg) {
    initial_pose_ = Eigen::Vector2d(msg->pose.pose.position.x,msg->pose.pose.position.y);
}

void SpacialHorizon::amcl_poseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg){
    odom_pos_ = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    have_odom_ = true;
}

void SpacialHorizon::odomCallback(const nav_msgs::OdometryConstPtr& msg){
    odom_pos_=Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_vel_=Eigen::Vector2d(msg->twist.twist.linear.x,msg->twist.twist.linear.y);

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    auto euler = odom_orient_.toRotationMatrix().eulerAngles(0, 1, 2); // row,pitch,yaw
    odom_dir_=euler(2); 
    odom_dir_=odom_dir_<0?2*PI+odom_dir_:odom_dir_;
    // cout<<"[SpacialHorizon]: odom direction"<<odom_dir_*180/3.1415<<endl;
    have_odom_ = true;
}

void SpacialHorizon::goalCallback(const geometry_msgs::PoseStampedPtr& msg){
    if(have_odom_==false) return;
    ros::param::set("/bool_goal_reached", false);
    // end pt
    end_pos_=Eigen::Vector2d(msg->pose.position.x,msg->pose.position.y);
    end_vel_=Eigen::Vector2d::Zero();

    have_goal_=true;
    std::cout << "[SpacialHorizon] Goal set!" << std::endl;

    getGlobalPath_MoveBase();

    // vis goal
    std::vector<Eigen::Vector2d> point_set;
    point_set.push_back(end_pos_);
    visualizePoints(point_set,0.5,Eigen::Vector4d(1, 1, 1, 1.0),vis_goal_pub_);

}

bool SpacialHorizon::getSubgoalSpacialHorizon(Eigen::Vector2d &subgoal){
    double dist_to_goal=(odom_pos_-end_pos_).norm();

    if(dist_to_goal <= goal_tolerance_){
        ros::param::set("/bool_goal_reached", true);
        std::cout << "[SpacialHorizon] Goal reached!" << dist_to_goal << std::endl;
        return false;
    }

    // if near goal
    if(dist_to_goal<planning_horizen_){
        subgoal = end_pos_;
        return true;
    }

    getGlobalPath_MoveBase();

    // select the nearst waypoint on global path
    int subgoal_id=0;

    for(size_t i=0; i < global_plan.response.plan.poses.size(); i++){
        Eigen::Vector2d wp_pt = Eigen::Vector2d(global_plan.response.plan.poses[i].pose.position.x, global_plan.response.plan.poses[i].pose.position.y);
        double dist_to_robot=(odom_pos_-wp_pt).norm();

        if((dist_to_robot<planning_horizen_+subgoal_tolerance_)&&(dist_to_robot>planning_horizen_-subgoal_tolerance_)){
            if(i>subgoal_id){
                subgoal_id=i;
            }
        }
    }
    
    if(subgoal_id>0){
        subgoal = Eigen::Vector2d(global_plan.response.plan.poses[subgoal_id].pose.position.x, global_plan.response.plan.poses[subgoal_id].pose.position.y);
        return true;
    }else{
        // because spacial horizon based on global path, so it needs global replan; for timed_astar it doesn't need
        //changeFSMExecState(GEN_NEW_GLOBAL, "SUBGOAL");
        return false;
    }

}

void SpacialHorizon::updateSubgoalDRLCallback(const ros::TimerEvent &e){
    //if there's no goal
    if(!have_goal_) return;
    
    // get subgoal
    bool subgoal_success=false;
    Eigen::Vector2d subgoal;

    subgoal_success=getSubgoalSpacialHorizon(subgoal);

    // std::cout<<subgoal_success<<std::endl;

    if(subgoal_success){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp=ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x=subgoal(0);
        pose_stamped.pose.position.y=subgoal(1);
        pose_stamped.pose.position.z=0.0;
        subgoal_DRL_pub_.publish(pose_stamped);

        // vis subgoal
        std::vector<Eigen::Vector2d> point_set;
        point_set.push_back(subgoal);
        visualizePoints(point_set,0.8,Eigen::Vector4d(1, 0, 1, 1.0),vis_subgoal_drl_pub_);
    }else{
        //changeFSMExecState(GEN_NEW_GLOBAL, "SUBGOAL");
    }
      
}

/* Get global plan from move_base */
void SpacialHorizon::getGlobalPath_MoveBase(){
	/* get global path from move_base */
	ros::NodeHandle nh;
	std::string service_name = "/move_base/NavfnROS/make_plan";
	while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
		ROS_INFO("[SpacialHorizon - GET_PATH] Waiting for service /move_base/NavfnROS/make_plan to become available");
	}

	ros::ServiceClient serviceClient = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
	if (!serviceClient) {
		ROS_FATAL("[SpacialHorizon - GET_PATH] Could not initialize get plan service from %s",
		serviceClient.getService().c_str());
	}

	fillPathRequest(global_plan.request);
	if (!serviceClient) {
		ROS_FATAL("[SpacialHorizon - GET_PATH] Persistent service connection to %s failed",
		serviceClient.getService().c_str());
	}
	// ROS_INFO("[GET_PATH] conntect to %s",serviceClient.getService().c_str());
    callPlanningService(serviceClient, global_plan);
}

void SpacialHorizon::fillPathRequest(nav_msgs::GetPlan::Request &request){
	request.start.header.frame_id ="map";
	request.start.pose.position.x = odom_pos_[0];//x coordinate of the initial position
	request.start.pose.position.y = odom_pos_[1];//y coordinate of the initial position
	request.start.pose.orientation.w = 1.0;//direction
	request.goal.header.frame_id = "map";
	request.goal.pose.position.x = end_pos_[0];//End point coordinates
	request.goal.pose.position.y = end_pos_[1];
	request.goal.pose.orientation.w = 1.0;
	request.tolerance = goal_tolerance_;//If the goal cannot be reached, the most recent constraint
}

void SpacialHorizon::callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv){
	//Perform the actual path planner call
	//Execute the actual path planner
	if (serviceClient.call(srv)) {
		//srv.response.plan.poses is the container to save the result, traverse and take out
		if (!srv.response.plan.poses.empty()) {
			visualizeGlobalPath(vis_global_path_pub_);
            globalPlan_DRL_pub_.publish(srv.response.plan);
		}else{
			ROS_WARN("[SpacialHorizon - GET_PATH] Got empty plan");
		}
	}else {
		ROS_ERROR("[SpacialHorizon - GET_PATH] Failed to call service %s - is the robot moving?",
		serviceClient.getService().c_str());
	}
}


/* Visualization */
void SpacialHorizon::visualizePoints(const std::vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = {};//ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  //mk.id              = id;

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = pt_size;
  mk.scale.y = pt_size;
  mk.scale.z = pt_size;

  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < point_set.size(); i++) {
    pt.x = point_set[i](0);
    pt.y = point_set[i](1);
    pt.z = 0.0;
    mk.points.push_back(pt);
  }
  pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void SpacialHorizon::visualizeGlobalPath(const ros::Publisher & pub){
  //create a path message
  ros::Time plan_time = {};//ros::Time::now();
  std::string global_frame="/map";
  
  nav_msgs::Path gui_path;
  gui_path.poses.resize(global_plan.response.plan.poses.size());
  gui_path.header.frame_id = global_frame;
  gui_path.header.stamp = plan_time;
  
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < global_plan.response.plan.poses.size(); i++){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame;
      pose.pose.position.x = global_plan.response.plan.poses[i].pose.position.x;    //world_x;
      pose.pose.position.y = global_plan.response.plan.poses[i].pose.position.y;    //world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      gui_path.poses[i]=pose;               //plan.push_back(pose);
  }
  pub.publish(gui_path);
}

int main(int argc, char** argv) 
{
    std::cout<<"Spacial Horizon node started"<<std::endl;
    ros::init(argc, argv, "spacial_horizon_node");

    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    //ros::NodeHandle nh("");
    ros::NodeHandle nh("~");
    SpacialHorizon spacial_horizon;
    spacial_horizon.init(nh);
    
    std::string ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM(":\tSpacial_Horizon successfully loaded for namespace\t"<<ns);
    
    ros::spin();
}

