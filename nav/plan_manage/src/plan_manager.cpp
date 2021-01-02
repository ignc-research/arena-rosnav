#include <plan_manage/plan_manager.h>





using namespace std;

void PlanManager::init(ros::NodeHandle& nh) {

    /*  plan param  */
    bool train_mode;
    nh.param("train_mode", train_mode, false);
    mode_=train_mode?TRAIN:TEST;

    /* initialize main modules */
    planner_collector_.reset(new PlanCollector);
    planner_collector_->initPlanModules(nh);
    visualization_.reset(new PlanningVisualization(nh));

    /* init variables */
    exec_state_  = FSM_EXEC_STATE::INIT;
    have_goal_ = false;
    have_odom_   = false;
    cur_state_=new RobotState(Eigen::Vector2d::Zero(),0.0,Eigen::Vector2d::Zero(),0.0);
    
    /* callback */
    exec_timer_   = nh.createTimer(ros::Duration(0.01), &PlanManager::execFSMCallback, this);
    //safety_timer_ = nh.createTimer(ros::Duration(0.05), &PlanManager::checkCollisionCallback, this);

    goal_sub_ =nh.subscribe("/goal", 1, &PlanManager::goalCallback, this);
    odom_sub_ = nh.subscribe("/odom", 1, &PlanManager::odometryCallback, this);
    subgoal_pub_  = nh.advertise<geometry_msgs::PoseStamped>("/subgoal",10);

    /* test purpose*/
    
}
  
void PlanManager::goalCallback(const geometry_msgs::PoseStampedPtr& msg) {
    // position z must be zero(2D motion plan)
    if(msg->pose.position.z !=0) return;
    
    cout << " Task Triggered!" << endl;

    // set end_state
    end_state_=new RobotState(msg->pose);
    end_state_->vel2d.setZero();
    
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
    visualization_->drawGoal(end_state_->to_PoseStampted(), 0.3, Eigen::Vector4d(1, 1, 1, 1.0));

    // init start_time for this task
    start_time_=ros::Time::now();
}

void PlanManager::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
  // get robot state according to odometry msg
  cur_state_= new RobotState(*msg);

  // set have_odom(means localization system is ready)
  have_odom_ = true;
}

void PlanManager::execFSMCallback(const ros::TimerEvent& e) {
  // print state with a fixed rate
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!have_goal_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  // FSM state-> action
  switch (exec_state_) {
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

    case WAIT_GOAL: {
      if (!have_goal_)
        return;
      else {
        // go to GEN_NEW_GLOBAL state, going to do global (re)plan 
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }


    case GEN_NEW_GLOBAL: {
      if(mode_==TRAIN){
        changeFSMExecState(REPLAN_MID, "FSM");
        return;
      }
      // set robot start state
      start_state_=new RobotState(cur_state_->pose2d,cur_state_->theta,cur_state_->vel2d,cur_state_->w);
      
      // execute global planning
      bool global_plan_success=planner_collector_->generate_global_plan(*start_state_,*end_state_);
      
      if(global_plan_success){
        // success:go to REPLAN_MID state, going to do mid horizon replan(subgoal) 
        visualization_->drawGlobalPath(planner_collector_->global_path_,0.03, Eigen::Vector4d(0.5, 0.5, 0.5, 0.6));
        changeFSMExecState(REPLAN_MID, "FSM");
      }else{
        // failed: go to GEN_NEW_GLOBAL state, going to do global (re)plan 
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }

    case EXEC_LOCAL: {
      if(mode_==TRAIN){
        //cout<<"EXEC_LOCAL"<<"Train mode"<<endl;
        return;
      }

      /* check env determine, calculate criterion */
      // fake obstacle info
      double dist_to_obstacle;
      random_device rd;
      uniform_real_distribution<double> rand_obstacle;
      uniform_real_distribution<double> rand_goal;
      rand_obstacle = uniform_real_distribution<double>(0.0, 3.0 );
      default_random_engine eng(rd());
      dist_to_obstacle=rand_obstacle(eng);
   
      // distance to (global) goal
      double dist_to_goal;
      dist_to_goal=(cur_state_->pose2d-end_state_->pose2d).norm();
      
      // distance to (mid horizon) subgoal
      double dist_to_subgoal;
      dist_to_subgoal=(cur_state_->pose2d-planner_collector_->subgoal_state_->pose2d).norm();
      
      // timeout: avoid task too much time 
      double timeout;
      timeout=1000;
      double time_cost=ros::Time::now().toSec()-start_time_.toSec();
      
      // tolerance goal, torlerance subgoal
      double tolerance_goal=0.5;
      double tolerance_subgoal=0.3;

      /*check state_transfer criterion*/
      // check if reached goal
      if(dist_to_goal<tolerance_goal){
        have_goal_=false;
        cout<<"reached to goal success"<<endl;
        changeFSMExecState(WAIT_GOAL, "FSM");
        return;
      }

      // check if timeout
      if(time_cost>timeout){
        have_goal_=false;
        cout<<"failed to goal"<<endl;
        changeFSMExecState(WAIT_GOAL, "FSM");
        return;
      }
      
      // check if need mid_horizon replan 
      if(dist_to_subgoal>2 || dist_to_subgoal<0.2){
        if(cur_state_->vel2d.norm()>0.1){
          changeFSMExecState(REPLAN_MID, "FSM");
        }
      }else{
        //cout<<"Normal:Exec local"<<endl;
        return;
      }

      break;
    }

    case REPLAN_MID: {
      if(mode_==TRAIN){
        subgoal_pub_.publish(end_state_->to_PoseStampted());
        visualization_->drawSubgoal(end_state_->to_PoseStampted(), 0.3, Eigen::Vector4d(0, 0, 0, 1.0));
        cout<<"MID_REPLAN Success"<<endl;
        changeFSMExecState(EXEC_LOCAL, "FSM");
        return;
      }
      /* get current state info */
      //RobotStatePtr mid_start_state=new RobotState(cur_state_->pose2d,cur_state_->theta,cur_state_->vel2d,cur_state_->w);
      double dist_to_goal=1.0;
      double obstacle_info=1.0;
      double sensor_info=1.0;
      
      /* new waypoint generation*/
      bool get_subgoal_success = planner_collector_->generate_subgoal(cur_state_,end_state_, planner_collector_->global_path_,obstacle_info,sensor_info);
      
      if (get_subgoal_success) {
        // success: publish new subgoal & going to state EXEC_LOCAL
        subgoal_pub_.publish(planner_collector_->subgoal_);
        visualization_->drawSubgoal(planner_collector_->subgoal_, 0.3, Eigen::Vector4d(0, 0, 0, 1.0));
        cout<<"MID_REPLAN Success"<<endl;

        
        changeFSMExecState(EXEC_LOCAL, "FSM");
      } else {
        // failed: going to state GEN_NEW_GLOBAL, to do new global plan
        changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
      }
      break;
    }
  }
}

/* helper functions */
void PlanManager::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void PlanManager::printFSMExecState() {
  string state_str[5] =  {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

int main(int argc, char** argv) {
    cout<<"Plan manager node start"<<endl;
    ros::init(argc, argv, "Plan_manager");
    
    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    ros::NodeHandle node_handle;
    ros::WallRate r(100);

    PlanManager plan_manager;
    plan_manager.init(node_handle);
    ros::spin();
}
