#include <arena_plan_manager/plan_manager.h>

using namespace std;

void PlanManager::init(ros::NodeHandle &nh)
{

  /*  plan param  */
  bool train_mode;
  ros::NodeHandle n;
  // not global param, because we want to set it in the launch file
  n.param("train_mode", train_mode, false);
  mode_ = train_mode ? TRAIN : TEST;

  nh.param("/look_ahead_distance", look_ahead_distance_, 1.5);
  nh.param("/tolerance_approach", tolerance_approach_, 0.5);

  nh.param("/timeout_goal", timeout_goal_, 300.);      //sec
  nh.param("/timeout_subgoal", timeout_subgoal_, 60.); //sec

  /* initialize main modules */
  planner_collector_.reset(new PlanCollector);
  planner_collector_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));
  /* init variables */
  exec_state_ = FSM_EXEC_STATE::INIT;
  have_goal_ = false;
  have_odom_ = false;
  cur_state_.reset(new RobotState(Eigen::Vector2d::Zero(), 0.0, Eigen::Vector2d::Zero(), 0.0));
  // DEBUG
  /* callback */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &PlanManager::execFSMCallback, this);
  //safety_timer_ = nh.createTimer(ros::Duration(0.05), &PlanManager::checkCollisionCallback, this);

  // subscriber
  goal_sub_ = nh.subscribe("goal", 1, &PlanManager::goalCallback, this);
  odom_sub_ = nh.subscribe("odometry/ground_truth", 1, &PlanManager::odometryCallback, this,ros::TransportHints().tcpNoDelay()); // odom  //odometry/ground_truth

  // publisher
  global_plan_pub_  = nh.advertise<nav_msgs::Path>("globalPlan",10); // relative name:/ns/node_name/globalPlan
  subgoal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("subgoal", 10); // relative name:/ns/subgoal
  robot_state_pub_ = nh.advertise<arena_plan_msgs::RobotStateStamped>("robot_state", 10);
  /* test purpose*/
}

void PlanManager::goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
  // position z must be zero(2D motion plan)
  if (msg->pose.position.z != 0)
    return;

  cout <<ros::this_node::getNamespace()<<" Task Triggered!" << endl;

  // set end_state
  end_state_.reset(new RobotState(msg->pose));
  end_state_->vel2d.setZero();

  // change state: to GEN_NEW_GLOBAL
  if (exec_state_ == WAIT_GOAL)
  {
    changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");
  }
  else if (exec_state_ == EXEC_LOCAL)
  {
    changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");
  }
  else if (exec_state_ == REPLAN_MID)
  {
    changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");
  }
  else if (exec_state_ == INIT)
  {
    changeFSMExecState(GEN_NEW_GLOBAL, "TRIG");
  }

  // set have_goal
  cout <<ros::this_node::getNamespace() <<"Goal set!" << endl;
  have_goal_ = true;
  visualization_->drawGoal(end_state_->to_PoseStampted(), 0.5, Eigen::Vector4d(1, 1, 1, 1.0));
  cout << ros::this_node::getNamespace()<<"Goal drawed!" << endl;
  // init start_time for this task
  start_time_ = ros::Time::now();
}

void PlanManager::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // get robot state according to odometry msg
  cur_state_.reset(new RobotState(*msg));

  // publish robot state
  robot_state_pub_.publish(cur_state_->toRobotStateStamped());

  // set have_odom(means localization system is ready)
  have_odom_ = true;
}

void PlanManager::execFSMCallback(const ros::TimerEvent &e)
{

  // std::cout<<"---------------------mode"<<mode_<<std::endl;
  // print state with a fixed rate
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100)
  {
    printFSMExecState();
    if (!have_odom_)
      ROS_DEBUG_STREAM("no odom.");
    if (!have_goal_)
      ROS_DEBUG_STREAM("wait for goal.");
    fsm_num = 0;
  }

  // FSM state-> action
  switch (exec_state_)
  {
  case INIT:
  {
    if (!have_odom_)
    {
      return;
    }
    if (!have_goal_)
    {
      return;
    }
    changeFSMExecState(WAIT_GOAL, "FSM");
    break;
  }

  case WAIT_GOAL:
  {
    if (!have_goal_)
      return;
    else
    {
      // go to GEN_NEW_GLOBAL state, going to do global (re)plan
      changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
    }
    break;
  }

  case GEN_NEW_GLOBAL:
  {
    if (mode_ == TRAIN)
    { 
      start_state_.reset(new RobotState(cur_state_->pose2d, cur_state_->theta, cur_state_->vel2d, cur_state_->w));
      bool global_plan_success = planner_collector_->generate_global_plan(*start_state_, *end_state_);
      if(global_plan_success){
        global_plan_pub_.publish(planner_collector_->global_path_);
        changeFSMExecState(REPLAN_MID, "FSM");
      }else{
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      return;
    }
    //set robot start state
    start_state_.reset(new RobotState(cur_state_->pose2d, cur_state_->theta, cur_state_->vel2d, cur_state_->w));
    // DEBUG
    // execute global planning
    bool global_plan_success = planner_collector_->generate_global_plan(*start_state_, *end_state_);
    
    if (global_plan_success)
    {
      // success:go to REPLAN_MID state, going to do mid horizon replan(subgoal)
      std::thread t(&PlanningVisualization::drawGlobalPath,visualization_ ,planner_collector_->global_path_, 0.03, Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),0);
      // visualization_->drawGlobalPath(planner_collector_->global_path_, 0.03, Eigen::Vector4d(0.5, 0.5, 0.5, 0.6));
      t.detach();
      changeFSMExecState(REPLAN_MID, "FSM");
    }
    else
    {
      // failed: go to GEN_NEW_GLOBAL state, going to do global (re)plan
      changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
    }
    break;
  }

  case EXEC_LOCAL:
  {
    if (mode_ == TRAIN)
    {
      //cout<<"EXEC_LOCAL"<<"Train mode"<<endl;
      start_state_.reset(new RobotState(cur_state_->pose2d, cur_state_->theta, cur_state_->vel2d, cur_state_->w));
      bool global_plan_success = planner_collector_->generate_global_plan(*start_state_, *end_state_);
      if(global_plan_success){
        global_plan_pub_.publish(planner_collector_->global_path_);
      }
      return;
    }

    /* check env determine, calculate criterion */
    // fake obstacle info
    //double dist_to_obstacle;
    //random_device rd;
    //uniform_real_distribution<double> rand_obstacle;
    //uniform_real_distribution<double> rand_goal;
    //rand_obstacle = uniform_real_distribution<double>(0.0, 3.0 );
    //default_random_engine eng(rd());
    //dist_to_obstacle=rand_obstacle(eng);

    // calculate: distance to (global) goal
    double dist_to_goal;
    dist_to_goal = (cur_state_->pose2d - end_state_->pose2d).norm();

    // calculate: distance to (mid horizon) subgoal
    double dist_to_subgoal;
    dist_to_subgoal = (cur_state_->pose2d - planner_collector_->subgoal_state_->pose2d).norm();

    // calculate: timecot to avoid task takes too much time
    double time_cost_goal = ros::Time::now().toSec() - start_time_.toSec();
    double time_cost_subgoal = ros::Time::now().toSec() - subgoal_start_time_.toSec();

    /* check state_transfer: Goal Criterion */
    // check if reached goal
    if (dist_to_goal < tolerance_approach_)
    {
      have_goal_ = false;
      ROS_INFO_STREAM(ros::this_node::getNamespace()<<"reached to goal success");
      changeFSMExecState(WAIT_GOAL, "FSM");
      return;
    }

    // check if goal timeout
    if (time_cost_goal > timeout_goal_)
    {
      have_goal_ = false;
      ROS_INFO_STREAM(ros::this_node::getNamespace()<<"failed to goal");
      changeFSMExecState(WAIT_GOAL, "FSM");
      return;
    }

    /* check state_transfer: Subgoal Criterion */
    // check if subgoal timeout
    if (time_cost_subgoal > timeout_subgoal_)
    {
      ROS_DEBUG_STREAM(ros::this_node::getNamespace()<<"subgoal has been published for" << time_cost_subgoal << "sec");
      ROS_DEBUG_STREAM(ros::this_node::getNamespace()<<"subgoal timeout");
      changeFSMExecState(REPLAN_MID, "FSM");
      return;
    }

    // check if subgoal distance to current robot position is too far away
    if (dist_to_subgoal > look_ahead_distance_)
    {
      // if the robot stopped at a pos far from subgoal, then won't replan mid, but wait for timeout and global replan
      bool robot_stopped = cur_state_->vel2d.norm() < 0.1;
      if (!robot_stopped)
      {
        changeFSMExecState(REPLAN_MID, "FSM");
      }
      return;
    }

    // check if subgoal is reached. If reached then replan
    if (dist_to_subgoal < tolerance_approach_)
    {
      changeFSMExecState(REPLAN_MID, "FSM");
      return;
    }
    else
    {

      //cout<<"Normal:Exec local"<<endl;
      return;
    }

    break;
  }

  case REPLAN_MID:
  {
    if (mode_ == TRAIN)
    {
      
      subgoal_pub_.publish(end_state_->to_PoseStampted());
      ROS_DEBUG_STREAM(ros::this_node::getNamespace()<<"subgoal= "<<end_state_->to_PoseStampted());
      //visualization_->drawSubgoal(end_state_->to_PoseStampted(), 0.3, Eigen::Vector4d(0, 0, 0, 1.0));
      ROS_DEBUG("MID_REPLAN Success");
      changeFSMExecState(EXEC_LOCAL, "FSM");
      return;
    }
    /* get current state info */
    //RobotStatePtr mid_start_state=new RobotState(cur_state_->pose2d,cur_state_->theta,cur_state_->vel2d,cur_state_->w);
    double obstacle_info = 1.0;
    double sensor_info = 1.0;

    /* new waypoint generation*/
    bool get_subgoal_success = planner_collector_->generate_subgoal(cur_state_, end_state_, planner_collector_->global_path_, obstacle_info, sensor_info);
  
    if (get_subgoal_success)
    {
      // success: publish new subgoal & going to state EXEC_LOCAL
      subgoal_pub_.publish(planner_collector_->subgoal_);
      visualization_->drawSubgoal(planner_collector_->subgoal_, 0.3, Eigen::Vector4d(0, 0, 0, 1.0));
      // reset subgoal start time(be used for timeout criterion)
      subgoal_start_time_ = ros::Time::now();
      ROS_DEBUG_STREAM(ros::this_node::getNamespace()<<"MID_REPLAN Success");

      changeFSMExecState(EXEC_LOCAL, "FSM");
    }
    else
    {
      // failed: going to state GEN_NEW_GLOBAL, to do new global plan
      changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
    }
    break;
  }
  }
}

/* helper functions */
void PlanManager::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call)
{
  string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL"};
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  ROS_DEBUG_STREAM(ros::this_node::getNamespace()<<"[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)]);
}

void PlanManager::printFSMExecState()
{
  string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL"};
  ROS_DEBUG_STREAM(ros::this_node::getNamespace()<<"[FSM]: state: " + state_str[int(exec_state_)]);
}
