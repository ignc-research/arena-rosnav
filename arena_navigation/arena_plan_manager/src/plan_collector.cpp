#include <arena_plan_manager/plan_collector.h>


PlanCollector::PlanCollector() {}

PlanCollector::~PlanCollector() { std::cout << "delete plan collector" << std::endl; }


void PlanCollector::initPlanModules(ros::NodeHandle &nh){

    goal_=geometry_msgs::PoseStamped();
    
    std::string global_plan_service_name = "/move_base/NavfnROS/make_plan";
    std::string ns = ros::this_node::getNamespace();
    std::string fullName = ns + "/"+global_plan_service_name;
    ros::service::waitForService(fullName);   
    global_plan_client_= nh.serviceClient<nav_msgs::GetPlan>(global_plan_service_name);

    // get plan parameter
    nh.param("/look_ahead_distance", look_ahead_distance_, 1.5);
    nh.param("/tolerance_approach", tolerance_approach_, 0.5);  
}

bool PlanCollector::generate_global_plan(RobotState &start_state,RobotState &end_state){
    //nav_msgs::GetPlan srv;
    std::cout<<"COME to plan global0"<<std::endl;
    arena_plan_msgs::MakeGlobalPlan srv;    
    srv.request.start=start_state.to_PoseStampted();
    srv.request.goal=end_state.to_PoseStampted();

    if (global_plan_client_.call(srv))  
    {   std::cout<<"COME to plan global2"<<std::endl;
        // set global_plan
        global_path_=srv.response.plan;
        std::cout<<"COME to plan global3"<<std::endl;
        
        return true;
    }
    else{
        return false;
    }
}


bool PlanCollector::generate_subgoal(RobotStatePtr cur_state, RobotStatePtr end_state, 
nav_msgs::Path global_path, double obstacle_info, double sensor_info){

    
    double dist_to_goal=(cur_state->pose2d-end_state->pose2d).norm();
    
    // find safe place
    if(dist_to_goal<look_ahead_distance_){
        subgoal_=end_state->to_PoseStampted();
        return true;
    }

    // select the nearst waypoint on global path
    int subgoal_id=0;
    for(int i=0;i<global_path.poses.size();i++){
        Eigen::Vector2d wp_pose2d;
        wp_pose2d(0)=global_path.poses[i].pose.position.x;
        wp_pose2d(1)=global_path.poses[i].pose.position.y;
        double dist_to_robot=(cur_state->pose2d-wp_pose2d).norm();
        if((dist_to_robot<look_ahead_distance_+tolerance_approach_)&&(dist_to_robot>look_ahead_distance_-tolerance_approach_)){
            if(i>subgoal_id){
                subgoal_id=i;
            }
        }
    }
    
    if(subgoal_id>0){
        subgoal_=global_path.poses[subgoal_id];
        subgoal_state_.reset(new RobotState(subgoal_.pose));
        return true;
    }else{
        return false;
    }
    
}