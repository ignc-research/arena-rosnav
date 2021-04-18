#include <arena_timespace_planner/dynamic_plan_fsm.h>
using std::cout;
using std::endl;
void DynamicReplanFSM::init(ros::NodeHandle &nh)
{   
    
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_goal_ = false;
    have_odom_ = false;
    target_traj_data_ = TargetTrajData();

    /*  fsm param  */
    node_=nh;
    node_.param("fsm/goal_tolerance",    goal_tolerance_, 0.5); //sim1/plan_manager/fsm/goal_tolerance
    node_.param("fsm/subgoal_tolerance", subgoal_tolerance_, 2.0);
    node_.param("fsm/replan_time_thresh", t_replan_thresh_, 0.5); 

    node_.param("fsm/planning_horizen", planning_horizen_, 3.0);
    node_.param("fsm/subgoal_drl_mode", subgoal_drl_mode_, 1);
    node_.param("fsm/subgoal_pub_period", subgoal_pub_period_, 0.5);

    bool use_drl;
    node_.param("fsm/use_drl", use_drl, false);
    
    /* initialize main modules */
    planner_manager_.reset(new DynamicPlanManager);
    planner_manager_->initPlanModules(node_);

    /* callback */
    exec_timer_ = node_.createTimer(ros::Duration(0.01), &DynamicReplanFSM::execFSMCallback, this);
    //safety_timer_ = node_.createTimer(ros::Duration(0.05), &DynamicReplanFSM::checkCollisionCallback, this);
    if(!use_drl){
        traj_tracker_timer_ = node_.createTimer(ros::Duration(0.01), &DynamicReplanFSM::trackTrajCallback, this);
    }
    subgoal_DRL_timer_ = node_.createTimer(ros::Duration(0.05), &DynamicReplanFSM::updateSubgoalDRLCallback, this);
    //dynamic_occ_map_timer_= node_.createTimer(ros::Duration(1.0), &DynamicReplanFSM::updateDynamicMapCallback,this);

    /* ros communication with public node */
    ros::NodeHandle public_nh;  // sim1/goal
  	goal_sub_ =public_nh.subscribe("goal", 1, &DynamicReplanFSM::goalCallback,this);
  	odom_sub_ = public_nh.subscribe("odom", 1, &DynamicReplanFSM::odomCallback, this);

    cmd_vel_pub_ =public_nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    subgoal_DRL_pub_  = public_nh.advertise<geometry_msgs::PoseStamped>("subgoal",10);
    global_plan_pub_  = public_nh.advertise<nav_msgs::Path>("globalPlan",10);           ///ns/globalPlan


    /* visualization */
    vis_goal_pub_ =	        public_nh.advertise<visualization_msgs::Marker>("vis_goal", 20);
    vis_global_path_pub_ =   public_nh.advertise<nav_msgs::Path>("vis_global_path", 10, true); 
    vis_landmark_pub_ =      public_nh.advertise<visualization_msgs::Marker>("vis_landmarks_", 20);

	vis_triangle_pub_=         public_nh.advertise<visualization_msgs::Marker>("vis_triangle", 20);
    vis_timed_astar_path_pub_= public_nh.advertise<nav_msgs::Path>("vis_path_timed_astar", 20);
    vis_timed_astar_wp_pub_ =  public_nh.advertise<visualization_msgs::Marker>("vis_wps_timed_astar", 20);

    vis_local_path_pub_ =public_nh.advertise<visualization_msgs::Marker>("vis_local_path", 20);

    vis_subgoal_drl_pub_ = public_nh.advertise<visualization_msgs::Marker>("vis_subgoal", 20);

}

void DynamicReplanFSM::odomCallback(const nav_msgs::OdometryConstPtr& msg){

    odom_pos_=Eigen::Vector2d(msg->pose.pose.position.x,msg->pose.pose.position.y);
    odom_vel_=Eigen::Vector2d(msg->twist.twist.linear.x,msg->twist.twist.linear.y);

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    auto euler = odom_orient_.toRotationMatrix().eulerAngles(0, 1, 2); // row,pitch,yaw
    odom_dir_=euler(2); 
    odom_dir_=odom_dir_<0?2*PI+odom_dir_:odom_dir_;
    //cout<<"odom direction"<<odom_dir_*180/3.1415<<endl;
    have_odom_ = true;
}

void DynamicReplanFSM::goalCallback(const geometry_msgs::PoseStampedPtr& msg){
    if(have_odom_==false) return;

    // end pt
    end_pos_=Eigen::Vector2d(msg->pose.position.x,msg->pose.position.y);
    end_vel_=Eigen::Vector2d::Zero();

    have_goal_=true;
    std::cout << "[Plan FSM]Goal set!" << std::endl;

    // update dynamic obstacle state tracker
    planner_manager_->updateDynamicObstacleInfo();
    
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
  
    // vis goal
    std::vector<Eigen::Vector2d> point_set;
    point_set.push_back(end_pos_);
    visualizePoints(point_set,0.5,Eigen::Vector4d(1, 1, 1, 1.0),vis_goal_pub_);

}

void DynamicReplanFSM::execFSMCallback(const ros::TimerEvent& e){
    exec_timer_.stop(); // To avoid blockage

    /* init phase wating for odom & goal */
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        printFSMExecState();
        if (!have_odom_) cout << "[Plan FSM]no odom." << endl;
        if (!have_goal_) cout << "[Plan FSM]wait for goal." << endl;
        fsm_num = 0;
    }

    /* FSM state-> action */
    switch (exec_state_) {
        /* --------------Case1:INIT ---------------*/
        case INIT: {
            if (!have_odom_) {
                goto force_return;
            }
            if (!have_goal_) {
                goto force_return;
            }

            changeFSMExecState(WAIT_GOAL, "FSM");
            break;
        }

        /* --------------Case2:WAIT_GOAL ---------------*/
        case WAIT_GOAL: {
            if (!have_goal_)
                goto force_return;
            else {
                changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
            }
            break;
        }

        /* --------------Case3:GEN_NEW_GLOBAL ---------------*/
        case GEN_NEW_GLOBAL: {
            if (have_odom_ && have_goal_){
                start_pos_ = odom_pos_;
                bool success= planner_manager_->planGlobalTraj(start_pos_, end_pos_);
                if(success){
                    // set mid_target
                    landmark_wps_.clear();
                    landmark_wps_= planner_manager_->global_data_.getLandmarks();
                    mid_target_=getNextWaypoint();

                    // reset simple samples and curr index
                    sample_wps_ = planner_manager_->global_data_.getSamples();
                    curr_wp_index_=0;
                    // publish global path
                    visualizePath(planner_manager_->global_data_.getGlobalPath(),global_plan_pub_);
                    
                    // visualize global path & landmark
                    cout<<"[Plan Manager]GLOBAL_PLAN Success"<<endl;
                    visualizePath(planner_manager_->global_data_.getGlobalPath(),vis_global_path_pub_);
                    
                    visualizePoints(planner_manager_->global_data_.getLandmarks(),0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),vis_landmark_pub_);
                    changeFSMExecState(REPLAN_MID, "FSM");
                    
                }else{
                    ROS_ERROR("[Plan FSM]Failed to generate the global plan!!!");
                    changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
                }
            }else{
                ROS_ERROR("[Plan FSM]No odom or no target! have_odom_=%d, have_goal_=%d", have_odom_, have_goal_);
            }
            break;
        }

        /* --------------Case4:REPLAN_MID ---------------*/
        case REPLAN_MID: {
            // read mid_target
            double dist_to_goal;
            dist_to_goal=(odom_pos_-end_pos_).norm();

            if(dist_to_goal<goal_tolerance_){
                target_traj_data_ = TargetTrajData();
                have_goal_=false;
                cout<<"[Plan FSM]this global target success, return to wait goal"<<endl;
                changeFSMExecState(WAIT_GOAL, "FSM");
                goto force_return;
            }


            // plan mid trajectory
            Eigen::Vector2d curr_pos = odom_pos_;
            Eigen::Vector2d curr_vel = odom_vel_;
            double          curr_dir = odom_dir_;
            Eigen::Vector2d target_vel = Eigen::Vector2d(0.0,0.0);
            std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> line_sets;
            
            bool success=planner_manager_->planMidTraj(curr_pos,curr_vel,curr_dir,mid_target_,line_sets);
            
            visualizeLines(line_sets,0.2,Eigen::Vector4d(0, 1, 1, 1.0),vis_triangle_pub_);
            
            
            //bool success =planner_manager_->planLocalTraj(curr_pos,curr_vel,curr_dir,mid_target_,target_vel);

            if(success){
             
                target_traj_data_ = planner_manager_->local_traj_data_;
                visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);

                cout<<"[Plan FSM]MID_REPLAN Success"<<endl;
                changeFSMExecState(EXEC_LOCAL, "FSM");
                mid_replan_count_=0;
            }else{
                mid_replan_count_++;
                // for debug
                target_traj_data_ = planner_manager_->local_traj_data_;
                visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);
                
                //target_traj_data_ = TargetTrajData();
                ROS_ERROR("[Plan FSM]Failed to generate the mid trajectory!!!");

                if(mid_replan_count_>2000){
                    changeFSMExecState(GEN_NEW_GLOBAL, "FSM");
                }else{
                    changeFSMExecState(REPLAN_MID, "TRY one more");
                }
                
            }
            break;

        }
        /* --------------Case5:EXEC_LOCAL ---------------*/
        case EXEC_LOCAL: {
            // distance to goal
            double dist_to_goal, dist_to_mid_target;
            dist_to_goal=(odom_pos_-end_pos_).norm();

            // reached goal
            if(dist_to_goal<goal_tolerance_){
                target_traj_data_ = TargetTrajData();
                have_goal_=false;
                cout<<"[Plan FSM]this global target success, return to wait goal"<<endl;
                changeFSMExecState(WAIT_GOAL, "FSM");
                goto force_return;
            }

            // reached mid target (not last mid target)
            dist_to_mid_target=(odom_pos_-mid_target_).norm();
            if(dist_to_mid_target<subgoal_tolerance_ && landmark_wps_.size()>1){
                mid_target_=getNextWaypoint();
                cout<<"[Plan FSM]this mid target success, return to wait goal"<<endl;
                changeFSMExecState(REPLAN_MID, "FSM");
            }

            // check time
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - target_traj_data_.start_time_).toSec();
            
            if(t_cur>t_replan_thresh_ || t_cur> target_traj_data_.duration_){
                changeFSMExecState(REPLAN_MID, "FSM");
            }

        }
        break;

    }

    force_return:;
    exec_timer_.start();

}

bool DynamicReplanFSM::planFromCurrentTraj(const int trial_times){
    ros::Time time_now = ros::Time::now();
    double t_curr = (time_now - target_traj_data_.start_time_).toSec();
    Eigen::Vector2d curr_pos = odom_pos_;
    Eigen::Vector2d curr_vel = odom_vel_;
    double          curr_dir = odom_dir_;
    Eigen::Vector2d target_vel =Eigen::Vector2d(0.0,0.0);

    bool success = planner_manager_->planLocalTraj(curr_pos,curr_vel,curr_dir,mid_target_,target_vel);
    if(success){
        target_traj_data_ = planner_manager_->local_traj_data_;
        visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);

    }else{
        // for debug
        target_traj_data_ = planner_manager_->local_traj_data_;
        visualizePath(target_traj_data_.getLocalPath(),vis_timed_astar_path_pub_);
        // real
        //target_traj_data_= TargetTrajData();
    }
    
}

void DynamicReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
    auto map = planner_manager_->grid_map_;

    if(have_goal_==false){
        return;
    }

    if (exec_state_ == WAIT_GOAL || exec_state_ == INIT)
      return;
    
    if(target_traj_data_.flag_is_empty){
        return;
    }
    /* ---------- check trajectory ---------- */
    ros::Time time_now = ros::Time::now();
    double t_curr = (time_now- target_traj_data_.start_time_).toSec();
    Eigen::Vector2d p_curr = target_traj_data_.pos_traj_.evaluateDeBoorT(t_curr);
    //const double CLEARANCE = 0.5;
    constexpr double time_step = 0.01;
    double t_2_3 = 2.0;//target_traj_data_.duration_ * 2 / 3;

    bool occ = false;
    for (double t = t_curr; t < target_traj_data_.duration_; t += time_step){
        
        if (t_curr < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
            break;

        occ = map->getFusedDynamicInflateOccupancy(target_traj_data_.pos_traj_.evaluateDeBoorT(t));
        
        if(occ==true){
            if(planFromCurrentTraj()){
                changeFSMExecState(EXEC_LOCAL, "SAFETY");
            }else{
                changeFSMExecState(REPLAN_MID, "SAFETY");
            }
            return;
        }
    }
}

void DynamicReplanFSM::trackTrajCallback(const ros::TimerEvent &e){
    geometry_msgs::Twist twist_cmd;

    ros::Time time_now = ros::Time::now();
    double v,w;
    if(target_traj_data_.flag_is_empty){
        v=0.0;
        w=0.0;
    }else{
        double t_cur;
        if(mid_replan_count_>5){
             t_cur = (time_now - target_traj_data_.start_time_).toSec();
        }else{
             t_cur = (time_now - target_traj_data_.start_time_).toSec();
        }
        
        v=0.0;
        w=0.0;
        target_traj_data_.getControlInput(t_cur,v,w);
        
        //ROS_INFO_STREAM("vel: "<<v);
        //ROS_INFO_STREAM("rot vel: "<<w);
    }
  
    //bool occ = map->getFusedDynamicInflateOccupancy(odom_pos_);
    // std::cout<<"occ----------------------"<<std::endl;
    // bool occ= planner_manager_->grid_map_->getFusedInflateOccupancy(odom_pos_);
    // std::cout<<"occ----------------------"<<std::endl;
    // std::cout<<"occ"<<occ<<std::endl;
    // if(occ){
    //     in_collision_cnt++;
    //     if(in_collision_cnt>100){
    //         v=-1.5;
    //     }
    // }else{
    //     in_collision_cnt=0;
    // }
    // if(mid_replan_count_>100){
    //     v=-0.5;
    // }
    
    if(mid_replan_count_>5){
        w=15*PI/180;
    }

        
    

    twist_cmd.angular.x=0.0;
    twist_cmd.angular.y=0.0;
    twist_cmd.angular.z=w;
    twist_cmd.linear.x =v;
    twist_cmd.linear.y =0.0;
    twist_cmd.linear.z =0.0;
    
    cmd_vel_pub_.publish(twist_cmd);
}

void DynamicReplanFSM::updateSubgoalDRLCallback(const ros::TimerEvent &e){
    //if there's no goal
    if(!have_goal_) return;

    // if there's no target_traj_data_, subgoal cannot be calculated
    if(target_traj_data_.flag_is_empty){return;}
    
    //if (exec_state_ == WAIT_GOAL || exec_state_ == INIT)
    // get subgoal
    bool subgoal_success=false;
    Eigen::Vector2d subgoal;
    switch (subgoal_drl_mode_) {
        /* --------------Spacial horizon ---------------*/
        case 0: {
            subgoal_success=getSubgoalSpacialHorizon(subgoal);
            break;
        }
        /* --------------Timed astar ---------------*/
        case 1:{
            subgoal_success=getSubgoalTimedAstar(subgoal);
            break;
        }
        case 2:{
            subgoal_success=getSubgoalSimpleSample(subgoal);
            break;
        }
        case 3:{
            subgoal_success=getSubgoalGlobal(subgoal);
            break;
        }
        /* --------------Others ---------------*/
        default: {
            ROS_ERROR_STREAM("Not valid subgoal mode"<<subgoal_drl_mode_);
            break;
        }
    }
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

bool DynamicReplanFSM::getSubgoalSpacialHorizon(Eigen::Vector2d &subgoal){
    double dist_to_goal=(odom_pos_-end_pos_).norm();
    
    // if near goal
    if(dist_to_goal<planning_horizen_){
        subgoal = end_pos_;
        return true;
    }

    // select the nearst waypoint on global path
    int subgoal_id=0;
    std::vector<Eigen::Vector2d> global_path=planner_manager_->global_data_.getGlobalPath();

    for(size_t i=0;i<global_path.size();i++){
        Eigen::Vector2d wp_pt=global_path[i];
        double dist_to_robot=(odom_pos_-wp_pt).norm();

        if((dist_to_robot<planning_horizen_+subgoal_tolerance_)&&(dist_to_robot>planning_horizen_-subgoal_tolerance_)){
            if(i>subgoal_id){
                subgoal_id=i;
            }
        }
    }
    
    if(subgoal_id>0){
        subgoal=global_path[subgoal_id];
        return true;
    }else{
        // because spacial horizon based on global path, so it needs global replan; for timed_astar it doesn't need
        changeFSMExecState(GEN_NEW_GLOBAL, "SUBGOAL");
        return false;
    }

}

bool DynamicReplanFSM::getSubgoalTimedAstar(Eigen::Vector2d &subgoal){
    double dist_to_goal=(odom_pos_-end_pos_).norm();
    
    // if near goal
    if(dist_to_goal<planning_horizen_){
        subgoal = end_pos_;
        return true;
    }
    /* // if traj too short
    Eigen::Vector2d pt_traj_end = target_traj_data_.pos_traj_.evaluateDeBoorT(1000);
    if((odom_pos_- pt_traj_end).norm()< planning_horizen_){
        subgoal = pt_traj_end;
        return true;
    }
    double look_at_time=2;
    Eigen::Vector2d pt_at_time=target_traj_data_.pos_traj_.evaluateDeBoorT(look_at_time);
    double length = (odom_pos_- pt_at_time).norm();

    if(length>planning_horizen_){
        subgoal = pt_at_time;
        return true;
    }else{
        //subgoal = odom_pos_ + (planning_horizen_/length)*(pt_at_time-odom_pos_);
        subgoal = pt_at_time;
        return true;
    }
 */
    // select the nearst waypoint on global path
    int subgoal_id=0;
    std::vector<Eigen::Vector2d> global_path=target_traj_data_.getLocalPath();
    
    for(size_t i=0;i<global_path.size();i++){
        Eigen::Vector2d wp_pt=global_path[i];
        double dist_to_robot=(odom_pos_-wp_pt).norm();

        if((dist_to_robot<planning_horizen_+subgoal_tolerance_)&&(dist_to_robot>planning_horizen_-subgoal_tolerance_)){
            if(i>subgoal_id){
                subgoal_id=i;
            }
        }
    }
    //cout<<"global_path.size()"<<global_path.size()<<endl;
    //cout<<"subgoal_id"<<subgoal_id<<endl;
    
    if(subgoal_id>0){
        subgoal=global_path[subgoal_id];

        return true;
    }else{
        if(global_path.size()>2){
            subgoal=global_path.back();
            return true;
        }
        return false;
    }
}

bool DynamicReplanFSM::getSubgoalSimpleSample(Eigen::Vector2d &subgoal){
    // if near goal
    double dist_to_goal=(odom_pos_-end_pos_).norm();
    if(dist_to_goal<planning_horizen_){
        subgoal = end_pos_;
        return true;
    }
    // check if sample_wps is empty
    if(sample_wps_.empty()){
        return false;
    }

    curr_wp_index_=std::min(curr_wp_index_,sample_wps_.size()-1);
    Eigen::Vector2d wp_pt=sample_wps_[curr_wp_index_];
    subgoal=wp_pt;

    double dist_to_robot=(odom_pos_-wp_pt).norm();
    if(dist_to_robot< 1.2){//subgoal_tolerance_
        curr_wp_index_++;
    }

    return true;
}

bool DynamicReplanFSM::getSubgoalGlobal(Eigen::Vector2d &subgoal){
    subgoal=end_pos_;
    return true;
}


/* --------------------helper functions---------------------------- */
void DynamicReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  string state_str[5] = {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void DynamicReplanFSM::printFSMExecState() {
  string state_str[5] =  {"INIT", "WAIT_GOAL", "GEN_NEW_GLOBAL", "REPLAN_MID", "EXEC_LOCAL" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

Eigen::Vector2d DynamicReplanFSM::getNextWaypoint(){
    
    Eigen::Vector2d next_wp = landmark_wps_.back();
    // if(landmark_wps_.size()>1){
    //     landmark_wps_.erase(landmark_wps_.begin());
    // }

    return next_wp;
}


/* --------------------visualization---------------------------- */
void DynamicReplanFSM::visualizePoints(const std::vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub) {
  
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

void DynamicReplanFSM::visualizeLines(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = pt_size;
    line_list.color.r = color(0);
    line_list.color.g = color(1);
    line_list.color.b = color(2);
    line_list.color.a = color(3);

    geometry_msgs::Point pt1,pt2;
    for (unsigned int i = 0; i < ptr_pair_sets.size(); i++) {
        pt1.x=ptr_pair_sets[i].first(0);
        pt1.y=ptr_pair_sets[i].first(1);
        pt1.z=0.0;

        pt2.x=ptr_pair_sets[i].second(0);
        pt2.y=ptr_pair_sets[i].second(1);
        pt2.z=0.0;

        line_list.points.push_back(pt1);
        line_list.points.push_back(pt2);
    }

    pub.publish(line_list);
    //ROS_INFO("vis once");
}

void DynamicReplanFSM::visualizePath(const std::vector<Eigen::Vector2d> path, const ros::Publisher & pub){

  //create a path message
  ros::Time plan_time = {};//ros::Time::now();
  std::string global_frame="/map";
  
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = global_frame;
  gui_path.header.stamp = plan_time;
  
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame;
      pose.pose.position.x = path[i](0);    //world_x;
      pose.pose.position.y = path[i](1);    //world_y;
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
    std::cout<<"Plan manager node start"<<std::endl;
    ros::init(argc, argv, "plan_manager");

    //ros::NodeHandle node_handle("~"); every topic will be with namespace
    //ros::NodeHandle nh("");
    ros::NodeHandle nh("~");
    DynamicReplanFSM dynamic_replan;
    dynamic_replan.init(nh);

    std::string ns = ros::this_node::getNamespace();
    ROS_INFO_STREAM(":\tPlan manager successfully loaded for namespace\t"<<ns);
    
    ros::spin();
}


















