#include <arena_plan_manager/plan_manager.h>

using namespace std;

void PlanManager::init(ros::NodeHandle& nh) {

    node_=nh;
    ros::NodeHandle public_node("");
    
    /*  plan param  */
    bool train_mode;
    public_node.param("/train_mode", train_mode, false);
    mode_=train_mode?TRAIN:TEST;
    node_.param("/timeout_goal",    timeout_goal_, 300.0);      //sec
    node_.param("/dist_tolerance", dist_tolerance_, 1.5);
    node_.param("/dist_lookahead", dist_lookahead_, 3.0);

    /* initialize inter planner modules */
    inter_planner_.reset(new InterPlanner);
    inter_planner_->init(node_);

    /* init variables */
    exec_state_  = FSM_EXEC_STATE::INIT;
    have_goal_ = false;
    have_odom_   = false;
    
    // /* callback */
    exec_timer_   = node_.createTimer(ros::Duration(0.01), &PlanManager::execFSMCallback, this);
    safety_timer_ = node_.createTimer(ros::Duration(0.05), &PlanManager::checkCollisionCallback, this);

    // // subscriber
    goal_sub_ =public_node.subscribe("goal", 1, &PlanManager::goalCallback, this);
    odom_sub_ = public_node.subscribe("odometry/ground_truth", 1, &PlanManager::odometryCallback, this); // odom  //odometry/ground_truth

    // // publisher
    global_plan_pub_ = public_node.advertise<nav_msgs::Path>("globalPlan", 10, true);        // relative name:/ns/node_name/globalPlan
    subgoal_pub_  = public_node.advertise<geometry_msgs::PoseStamped>("subgoal",10);// relative name:/ns/node_name/subgoal
    robot_state_pub_  = public_node.advertise<arena_plan_msgs::RobotStateStamped>("robot_state",10);
}
  
void PlanManager::goalCallback(const geometry_msgs::PoseStampedPtr& msg) {
    cout << " Task Triggered!" << endl;

    // set end_state
    end_pt_(0)=msg->pose.position.x;
    end_pt_(1)=msg->pose.position.y;
    end_vel_=Eigen::Vector2d::Zero();
    
    // change state: to GEN_NEW_GLOBAL
    if (exec_state_ == WAIT_GOAL){
      changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");
    } 
    else if (exec_state_ == EXEC_LOCAL){
      changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");  
    }
    else if (exec_state_ == REPLAN_MID){
      changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");  
    }
    else if (exec_state_ == INIT){
      changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");  
    }
    
    // set have_goal
    cout << "Goal set!" << endl;
    have_goal_ = true;
    // draw goal(visualized in interplanner already)
    //inter_planner->visualizePoints()
    // init start_time for this task
    goal_start_time_=ros::Time::now();
}

void PlanManager::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
  /* set odom*/
  odom_pt_(0) = msg->pose.pose.position.x;
  odom_pt_(1) = msg->pose.pose.position.y;
  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  
  /* set robot state msg */
  arena_plan_msgs::RobotStateStamped state_stamped;
  //header
  state_stamped.header.stamp=ros::Time::now();
  state_stamped.header.frame_id = "map";
  state_stamped.state.pose=msg->pose.pose;
  state_stamped.state.twist=msg->twist.twist;
        
  // publish robot state
  robot_state_pub_.publish(state_stamped);

  // set have_odom(means localization system is ready)
  have_odom_ = true;
}

void PlanManager::publishGlobalPath(const std::vector<Eigen::Vector2d> & global_path){
  //create a path message
  ros::Time plan_time =ros::Time::now();
  std::string global_frame="/map";
  
  nav_msgs::Path gui_path;
  gui_path.poses.resize(global_path.size());
  gui_path.header.frame_id = global_frame;
  gui_path.header.stamp = plan_time;
  
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < global_path.size(); i++){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame;
      pose.pose.position.x = global_path[i](0);    //world_x;
      pose.pose.position.y = global_path[i](1);    //world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      gui_path.poses[i]=pose;               //plan.push_back(pose);
  }
  global_plan_pub_.publish(gui_path);

}

void PlanManager::publishSubgoal(const Eigen::Vector2d & subgoal){
  geometry_msgs::PoseStamped pose_stamped;
       
  pose_stamped.header.stamp=ros::Time::now();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x=subgoal(0);
  pose_stamped.pose.position.y=subgoal(1);
  pose_stamped.pose.position.z=0.0;

  subgoal_pub_.publish(pose_stamped);
}

void PlanManager::execFSMCallback(const ros::TimerEvent& e){
  /* init phase wating for odom & goal */
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    // if (!have_odom_) cout << "[Plan Manager]no odom." << endl;
    // if (!have_goal_) cout << "[Plan Manager]wait for goal." << endl;
    fsm_num = 0;
  }

  /* FSM state-> action */
  switch (exec_state_) {
    /* --------------Case1:INIT ---------------*/
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!have_goal_) {
        return;
      }
      changeFSMExecState(WAIT_GOAL, "FSM");
      break;
    }

    /* --------------Case2:WAIT_GOAL ---------------*/
     case WAIT_GOAL: {
      if (!have_goal_)
        return;
      else {
        // go to GEN_NEW_GLOBAL state, going to do global (re)plan 
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }

    /* --------------Case3:GEN_NEW_GLOBAL ---------------*/
    case GEN_NEW_GLOBAL: {

      bool global_plan_success=inter_planner_->makeGlobalPlan(odom_pt_,end_pt_);
      if(global_plan_success){
        // success:go to REPLAN_MID state, going to do mid horizon replan(subgoal) 
        publishGlobalPath(inter_planner_->global_data_.getGlobalPath());
        
        // init curr_landmark[for landmark timeout mechanism]
        curr_landmark_=odom_pt_;
        landmark_timeout_=20.0;
        landmark_start_time_=ros::Time::now();
        
        //  change FSM
        changeFSMExecState(REPLAN_MID, "FSM");
        
      }else{
        // failed: go to GEN_NEW_GLOBAL state, going to do global (re)plan
        ROS_WARN_STREAM("[Plan Manager] Failed to generate Gloabl Plan with given goal"<<end_pt_); 
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }

    /* --------------Case4:REPLAN_MID ---------------*/
    case REPLAN_MID: {
  
      /* landmark timeout check */
      double time_cost_landmark=ros::Time::now().toSec()-landmark_start_time_.toSec();

      if(time_cost_landmark>landmark_timeout_){
        ROS_WARN_STREAM("[Plan Manager] Landmark timeout, MID_REPLAN to Global Replan, used time:"<<landmark_timeout_);
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
        break;
      }

      // reset landmark start time if target landmark is changed
      if((curr_landmark_-inter_planner_->global_data_.getCurrentLandmark()).norm()>1.0){
        landmark_start_time_=ros::Time::now();
        
        Eigen::Vector2d curr_landmark= inter_planner_->global_data_.getCurrentLandmark();
        Eigen::Vector2d prev_landmark= curr_landmark_;
        double min_vel=0.05;
        landmark_timeout_=(curr_landmark-prev_landmark).norm()/min_vel;
        // reset curr_landmark_
        curr_landmark_=curr_landmark;
      }

      /* get subgoal */
      bool get_subgoal_success=inter_planner_->makeSubgoal(odom_pt_, odom_vel_,3.0);

      if(get_subgoal_success){
        if(mode_==TRAIN)
        {
          publishSubgoal(end_pt_);
        }else{
          publishSubgoal(inter_planner_->mid_data_.getSubgoal());
        }

        // cout<<"[Plan Manager]MID_REPLAN Success"<<endl;
        changeFSMExecState(EXEC_LOCAL, "FSM");
      }else{
        ROS_WARN("[Plan Manager] Replan mid failed, no subgoal generated");
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      
      break;
    }

    /* --------------Case5:EXEC_LOCAL ---------------*/
    case EXEC_LOCAL: 
    {
      if(mode_==TRAIN){
        //cout<<"EXEC_LOCAL"<<"Train mode"<<endl;
        return;
      }

      // distance to (global) goal
      double dist_to_goal, dist_to_subgoal;
      dist_to_goal=(odom_pt_-end_pt_).norm();
      dist_to_subgoal=(odom_pt_-inter_planner_->mid_data_.getSubgoal()).norm();

      
      double time_cost_goal=ros::Time::now().toSec()-goal_start_time_.toSec();
     
      if(dist_to_goal<dist_tolerance_){
        have_goal_=false;
        // cout<<"[Plan Manager]reached to goal success"<<endl;
        changeFSMExecState(WAIT_GOAL, "FSM");
        return;
      }

      if(time_cost_goal>timeout_goal_){
        have_goal_=false;
        ROS_WARN("[Plan Manager] time out,failed to appoaching goal");
        changeFSMExecState(WAIT_GOAL, "FSM");
        return;
      }

      if(dist_to_subgoal<dist_tolerance_*1.5){
        // cout<<"[Plan Manager]reached to subgoal success, to REPLAN_MID"<<endl;
        changeFSMExecState(REPLAN_MID, "FSM");
        return;
      }
    }
  }
}

void PlanManager::checkCollisionCallback(const ros::TimerEvent& e) {
  Eigen::Vector2d subgoal=inter_planner_->mid_data_.getSubgoal();
  bool is_collid=inter_planner_->checkColiisionSegment(odom_pt_,subgoal);

  if(is_collid){
    ROS_INFO("[Plan Manager] Current local traj is in collision, to REPLAN_MID");
    if (exec_state_ == EXEC_LOCAL) {
          changeFSMExecState(REPLAN_MID, "SAFETY");
    }
  }
}


/* helper functions */
void PlanManager::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  // cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void PlanManager::printFSMExecState() {
  string state_str[5] =  {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };

  // cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}


int main(int argc, char** argv) 
{
    std::cout<<"Plan manager node start"<<std::endl;
    ros::init(argc, argv, "plan_manager");

    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    //ros::NodeHandle nh("");
    ros::NodeHandle nh("~");
    PlanManager plan_manager;
    plan_manager.init(nh);

    std::string ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM(":\tPlan manager successfully loaded for namespace\t"<<ns);
    
    ros::spin();
}