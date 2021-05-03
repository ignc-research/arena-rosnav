#include <arena_timespace_planner/dynamic_plan_manager.h>


using std::cout;
using std::endl;

void DynamicPlanManager::initPlanModules(ros::NodeHandle &nh)
{
  node_=nh;
  // ros param
  node_.param("plan_manager/max_vel",   pp_.max_vel_, 2.0);
  node_.param("plan_manager/max_acc",   pp_.max_acc_, 3.0);
  node_.param("plan_manager/max_jerk",  pp_.max_jerk_, 4.0);
  //node_.param("plan_manager/time_resolution",  pp_.time_resolution_, 1.0);
  node_.param("plan_manager/feasibility_tolerance",   pp_.feasibility_tolerance_, 0.05);
  node_.param("plan_manager/control_points_distance", pp_.ctrl_pt_dist_, 0.4);
  node_.param("plan_manager/use_distinctive_trajs",   pp_.use_distinctive_trajs_, true);
  node_.param("plan_manager/local_time_horizon",   pp_.local_time_horizon_,    2.0);
  

  

  // init grid_map
  grid_map_.reset(new GridMap);
  grid_map_->initMap(node_);

  // global planner
  global_planner_kino_astar_.reset(new KinodynamicAstar);
  global_planner_kino_astar_->setParam(node_);
  global_planner_kino_astar_->setEnvironment(grid_map_);
  global_planner_kino_astar_->init();
  global_planner_kino_astar_->reset();

  bspline_optimizer_esdf_.reset(new BsplineOptimizerESDF);
  bspline_optimizer_esdf_->setParam(node_);
  bspline_optimizer_esdf_->setEnvironment(grid_map_);

  // MovingObstacleInfo
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);
  node_.param<std::string>("plan_manager/dynamic_obstacle_name",   str_dynamic_obs_,    "obs");
  std::cout << "parameter name="<< str_dynamic_obs_<<std::endl;

	obs_info_provider_.reserve(100);
  for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++)
  {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "topic_name" << it - topic_infos.begin() << ": " << info.name << std::endl;

        if (info.name.find(str_dynamic_obs_) != std::string::npos) 
        {
            std::cout << "topic_" << it - topic_infos.begin() << ": " << info.name << std::endl;
			      obs_info_provider_.emplace_back(std::make_shared<DynamicObstacleInfo>(node_,info.name,grid_map_));
        }

  }

  // mid planner: timed_astar
  mid_planner_timed_astar_.reset(new TimedAstarSearch);
  mid_planner_timed_astar_->init(node_,grid_map_,obs_info_provider_);

  bspline_optimizer_.reset(new BsplineOptimizer);
  bspline_optimizer_->setParam(node_);
  bspline_optimizer_->setEnvironment(grid_map_);
  bspline_optimizer_->a_star_.reset(new AStar);
  bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(200, 200));

}

void DynamicPlanManager::updateDynamicObstacleInfo(){
  //reset
  obs_info_provider_.clear();
  
  // MovingObstacleInfo
  ros::master::V_TopicInfo topic_infos;
  ros::master::getTopics(topic_infos);
  
	obs_info_provider_.reserve(100);
  for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++)
  {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "topic_name" << it - topic_infos.begin() << ": " << info.name << std::endl;

        if (info.name.find(str_dynamic_obs_) != std::string::npos) 
        {
            std::cout << "topic_" << it - topic_infos.begin() << ": " << info.name << std::endl;
			      obs_info_provider_.emplace_back(std::make_shared<DynamicObstacleInfo>(node_,info.name,grid_map_));
        }

  }

  // reset mid_planner_timed_astar_
  mid_planner_timed_astar_.reset(new TimedAstarSearch);
  mid_planner_timed_astar_->init(node_,grid_map_,obs_info_provider_);
}

bool DynamicPlanManager::kinoAstarTraj(Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,Eigen::Vector2d &end_pos){
    global_planner_kino_astar_->reset();
    adjustStartAndTargetPoint(start_pos,end_pos);
    Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_vel   = Eigen::Vector2d::Zero();

    int status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, true);
    
    // second search // search again
    if (status == global_planner_kino_astar_->NO_PATH) {
      // retry searching with discontinuous initial state
      global_planner_kino_astar_->reset();
      status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, false);

      if (status == global_planner_kino_astar_->NO_PATH) {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;

      } else {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
      }
    } else 
    {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    double sample_step_size = 0.1;
    std::vector<Eigen::Vector2d> point_set,start_end_derivatives;
    
    global_planner_kino_astar_->getSamples(sample_step_size, point_set, start_end_derivatives);
    if(point_set.size()<1){
      return false;
    }
    // optimize global trajectory
    UniformBspline global_traj;
    bool optimize_success;
    optimize_success=optimizeGlobalTraj(0.1,point_set,start_end_derivatives,global_traj);
    if(!optimize_success){
      ROS_WARN_STREAM("[kino replan]: trajectory optimize failed.");
    }
    local_traj_data_.resetData(global_traj);
    return true;
}

bool DynamicPlanManager::planGlobalTraj( Eigen::Vector2d &start_pos,  Eigen::Vector2d &end_pos){
    std::cout<<"******************************************************"<<std::endl;
    std::cout<<"[kino replan]start----------------------------------"<<std::endl;
    std::cout<<"******************************************************"<<std::endl;

    // initial reset
    int status;
    global_planner_kino_astar_->reset();
    adjustStartAndTargetPoint(start_pos,end_pos);
    
    // init first search
    Eigen::Vector2d start_vel = Eigen::Vector2d::Zero();
    Eigen::Vector2d start_acc = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_vel   = Eigen::Vector2d::Zero();
    status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, true);

    // second search // search again
    if (status == KinodynamicAstar::NO_PATH) {
      // retry searching with discontinuous initial state
      global_planner_kino_astar_->reset();
      status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, false);

      if (status == KinodynamicAstar::NO_PATH) {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;

      } else {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
      }
    } else 
    {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    // sample points from global intial trajectory
    double sample_step_size = 0.1;   //pp_.ctrl_pt_dist_ / pp_.max_vel_;
    std::vector<Eigen::Vector2d> point_set,start_end_derivatives;
    global_planner_kino_astar_->getSamples(sample_step_size, point_set, start_end_derivatives);

    // get ts, pointset, derivatives
    double ts=sample_step_size;
   
    // optimize global trajectory
    UniformBspline global_traj;
    bool optimize_success;
    optimize_success=optimizeGlobalTraj(ts,point_set,start_end_derivatives,global_traj);
    
    if(!optimize_success){
      ROS_WARN_STREAM("[kino replan]: trajectory optimize failed.");
    }
    
    global_data_.resetData(global_traj);
    // in case start_pos trapped 
    //local_traj_data_.resetData(global_traj);

    return true;

}

bool DynamicPlanManager::optimizeGlobalTraj(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives, UniformBspline & bspline_traj){
  /* -------------------Optimize ESDF----------------------------------- */
  Eigen::MatrixXd ctrl_pts;
  UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

  // define cost function
  int cost_function = BsplineOptimizerESDF::NORMAL_PHASE;//NORMAL_PHASE;

  /*** STEP 2: OPTIMIZE ***/
  ctrl_pts = bspline_optimizer_esdf_->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
  UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

  double to = pos.getTimeSum();
  
  double ratio;
  bool feasible = pos.checkFeasibility(ratio,false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {
      feasible = pos.reallocateTime();
      if (++iter_num >= 3) break;
  }
  double tn = pos.getTimeSum();

  
  //std::cout << "[kino replan]: Reallocate ratio: " << tn / to << std::endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  bspline_traj=pos;
  return true;

}

bool DynamicPlanManager::planMidTraj(Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,  double & start_dir,  Eigen::Vector2d & end_pos,std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &line_sets){
  std::cout<<"[planMidTraj]timed astar search start 1"<<std::endl;
  std::vector<Eigen::Vector2d> point_set,start_end_derivatives;
  
  double ts=pp_.ctrl_pt_dist_ / pp_.max_vel_;
  double local_time_horizon=pp_.local_time_horizon_;
  bool success;

  // if initial pos is in collision
  if(grid_map_->getDistanceStatic(start_pos)<0.3){
    kinoAstarTraj(start_pos,start_vel,end_pos);
    return false;
  }

  success=mid_planner_timed_astar_->stateTimeAstarSearch(start_pos,start_vel,start_dir,end_pos,ts,local_time_horizon, point_set,start_end_derivatives,line_sets);
  
  

  // special case when poinst set too small even success
  if(success && point_set.size()<5){
    success=false;
    end_pos=point_set.back();
  }
  std::cout<<"[planMidTraj]timed astar search finish 1"<<std::endl;
  UniformBspline mid_traj;
  // if no timed_astar solution available
  if(!success){
    Eigen::Vector2d end_vel =Eigen::Vector2d::Zero();
    bool oneshot_success=genOneshotTraj(start_pos, start_vel,start_dir,end_pos,end_vel, mid_traj);
    std::cout<<"[planMidTraj]timed astar search finish 2"<<std::endl;
    if(oneshot_success){
      local_traj_data_.resetData(mid_traj);
      return true;
    }else{
      return false;
    }
  }else{
    Eigen::Vector2d dummy_start_pt  =point_set.front();
    // if(checkCollision(dummy_start_pt)){
    //   Eigen::Vector2d pt1 = dummy_start_pt - (end_pos-dummy_start_pt)*0.5/((end_pos-dummy_start_pt).norm());
    //   Eigen::Vector2d pt2 = dummy_start_pt - (end_pos-dummy_start_pt)*1.0/((end_pos-dummy_start_pt).norm());
    //   point_set.insert(point_set.begin(),pt1);
    //   point_set.insert(point_set.begin(),pt2);
    //   point_set.insert(point_set.begin(),pt1);
    // }
    // optimze traj
    bool optimize_success=optimizeBsplineTraj(ts,point_set,start_end_derivatives,mid_traj);
    local_traj_data_.resetData(mid_traj);
    
    if(!optimize_success){
      Eigen::Vector2d end_vel =Eigen::Vector2d::Zero();
      bool oneshot_success=genOneshotTraj(start_pos, start_vel,start_dir,end_pos,end_vel, mid_traj);
      if(oneshot_success)
      {
        local_traj_data_.resetData(mid_traj);
      }else{
        return false; //false
      }
    }else{
      ROS_WARN_STREAM("end mid optimize");
      local_traj_data_.resetData(mid_traj);
    }
  }
  return true;
}

bool DynamicPlanManager::genOneshotTraj(Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,  double & start_dir,  Eigen::Vector2d & target_pos, Eigen::Vector2d & target_vel, UniformBspline & oneshot_traj){
  // compute total time of traj
  double dist = (start_pos - target_pos).norm();
  double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;//double time =dist / pp_.max_vel_;
  
  // init_one_segment_traj
  PolynomialTraj init_traj;
  init_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, Eigen::Vector2d::Zero(), target_pos, target_vel, Eigen::Vector2d::Zero(), time);

  // get point_set sample from init_one_segment_traj
  std::vector<Eigen::Vector2d> point_set;
  double local_time_horizon=pp_.local_time_horizon_;
  double ts = dist > 0.1 ? pp_.ctrl_pt_dist_ / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist_ / pp_.max_vel_ * 5;
  double t_first, t;
  t_first=std::min(0.2,init_traj.getTimeSum());
  
  if(t_first<0.01){ // make sure not exceed the range
    ROS_ERROR_STREAM("[Oneshot traj] Too short time of traj ");
    return false;
  }

  bool flag_too_far;
  ts *= 1.5; // ts will be divided by 1.5 in the next
  do{
    if(ts<0.001){
      // ts is too small, not reasonable
      return false;
    }
    ts /= 1.5;
    point_set.clear();
    flag_too_far = false;
    Eigen::Vector2d last_pt = init_traj.evaluate(t_first); // t_first should not exceed getTimeSum()

    for ( t = t_first; t < init_traj.getTimeSum(); t += ts) // t should not exceed getTimeSum()
    { 
      Eigen::Vector2d pt = init_traj.evaluate(t);         
    
      if ((last_pt - pt).norm() > pp_.ctrl_pt_dist_ * 1.5)
      {
        flag_too_far = true;
        break;
      }

      if(t>local_time_horizon){
        break;
      }
      last_pt = pt;

      point_set.push_back(pt);

    }
  } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
  t -= ts;

  // check if last pt is in collision or not, if in collision add more points
  while(checkCollision(init_traj.evaluate(t)) && t+ts<init_traj.getTimeSum()){
    t+=ts;
    Eigen::Vector2d pt = init_traj.evaluate(t);
    point_set.push_back(pt);
  }
  // double check
  Eigen::Vector2d dummy_start_pt  =point_set.front();
  // if(checkCollision(dummy_start_pt)){
  //   Eigen::Vector2d pt1 = dummy_start_pt - (target_pos-dummy_start_pt)*1/((target_pos-dummy_start_pt).norm());
  //   Eigen::Vector2d pt2 = dummy_start_pt - (target_pos-dummy_start_pt)*2/((target_pos-dummy_start_pt).norm());
  //   point_set.insert(point_set.begin(),pt1);
  //   point_set.insert(point_set.begin(),pt2);
  //   point_set.insert(point_set.begin(),pt1);
  // }
  //Eigen::Vector2d dummy_target_pt =point_set.back();
  //adjustStartAndTargetPoint(dummy_start_pt,point_set.back());
  //point_set[point_set.size()-1] = dummy_target_pt;

  // adjust direction part of the traj too make traj easy to track for non-holomonic robot
  double new_dir, diff_dir;
  new_dir=atan2((target_pos-start_pos)(1),(target_pos-start_pos)(0));
  new_dir=new_dir<0?2*PI+new_dir:new_dir;
  diff_dir = std::abs(new_dir-start_dir);

  if(diff_dir>PI*0.25){
    ts=ts*1.5; 
  }else if(diff_dir>PI*0.50){
    point_set.insert(point_set.begin(),start_pos);
    point_set.insert(point_set.begin(),start_pos+start_vel*0.1);
    point_set.insert(point_set.begin(),start_pos);
  }

  // check if enough points
  if(point_set.size()<5){
    ROS_ERROR_STREAM("[Oneshot traj]point_set size<5,num="<<point_set.size());
    return false;
  }

  // get derivatives
  std::vector<Eigen::Vector2d> start_end_derivatives;
  start_end_derivatives.push_back(init_traj.evaluateVel(0));
  start_end_derivatives.push_back(init_traj.evaluateVel(t));
  start_end_derivatives.push_back(init_traj.evaluateAcc(0));
  start_end_derivatives.push_back(init_traj.evaluateAcc(t));
  ROS_WARN("[Oneshot traj]start optimize");
  //optimize traj if start_pos is not in collision
  UniformBspline bspline_traj;
  bool optimize_success;
  if(!checkCollision(start_pos)){
    //bspline_optimizer_->setLocalTargetPt(target_pos);
    optimize_success=optimizeBsplineTraj(ts,point_set,start_end_derivatives,bspline_traj);
  }else{
    optimize_success=false;
  }

  if(!optimize_success){
    // if optimize failed, use unoptimized traj
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    bspline_traj = UniformBspline(ctrl_pts, 3, ts);
  }else{
    /* */
  }
  // assign resulted  traj
  oneshot_traj = bspline_traj;
  ROS_WARN("[Oneshot traj]end optimize");
  return optimize_success;
}

bool DynamicPlanManager::planLocalTraj( Eigen::Vector2d & start_pos, Eigen::Vector2d & start_vel,  double & start_dir,  Eigen::Vector2d & target_pos, Eigen::Vector2d & target_vel ){
  
  UniformBspline local_traj;
  // genOneshotTraj will always give trajectory
  bool success=genOneshotTraj(start_pos, start_vel,start_dir,target_pos,target_vel,local_traj);
  if(success){
    local_traj_data_.resetData(local_traj); // [old comment]loacal traj data will always be reset
  }
  return success;
}

bool DynamicPlanManager::optimizeBsplineTraj(double ts,std::vector<Eigen::Vector2d>  point_set, std::vector<Eigen::Vector2d>  start_end_derivatives, UniformBspline & optimized_traj){
  

  // init bspline_optimizer local target(dont need to set target cost for us)
  //Eigen::Vector2d local_target_pt = point_set.back();
  //bspline_optimizer_->setLocalTargetPt(local_target_pt);

  // init bspline
  Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
  UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  
  // get collision segments
  std::vector<std::pair<int, int>> segments;
  segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
  
  /*** STEP 2: OPTIMIZE ***/
  bool flag_step_1_success = false;
  std::vector<vector<Eigen::Vector2d>> vis_trajs;

  if (pp_.use_distinctive_trajs_)
  {
    std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
    cout<<"****************************************"<< endl;
    cout << "\033[1;33m"<< "[optimizeBsplineTraj]multi-trajs=" << trajs.size() << "\033[1;0m" << endl;
    cout<<"****************************************"<< endl;

    double final_cost, min_cost = 999999.0;
    for (int i = trajs.size() - 1; i >= 0; i--)
    {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "[optimizeBsplineTraj] traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          std::vector<Eigen::Vector2d> vis_point_set;
          vis_point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            vis_point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(vis_point_set);
        }
        else
        {
          cout << "[optimizeBsplineTraj] traj " << trajs.size() - i << " failed." << endl;
        }
       
      }

  }
  else
  {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
  }

  cout << "[optimizeBsplineTraj] optimized success=" << flag_step_1_success << endl;
  if (!flag_step_1_success)
  {
      optimized_traj = UniformBspline(ctrl_pts, 3, ts);
      return false;
  }

  UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
 
    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // double ratio;
    // bool flag_step_2_success = true;
    // if (!pos.checkFeasibility(ratio, false))
    // {
    //   cout << "Need to reallocate time." << endl;

    //   Eigen::MatrixXd optimal_control_points;
    //   flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
    //   if (flag_step_2_success)
    //     pos = UniformBspline(optimal_control_points, 3, ts);
    // }

    // if (!flag_step_2_success)
    // {
    //   printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
    //   continous_failures_count_++;
    //   return false;
    // }

    // save planned results
    //updateTrajInfo(pos, ros::Time::now());

    // success. YoY
    //continous_failures_count_ = 0;
  optimized_traj=pos;
  return true;
}

bool DynamicPlanManager::adjustStartAndTargetPoint( Eigen::Vector2d & start_pt, Eigen::Vector2d &target_pt)
{   
    double step_size=0.1;

    if(checkCollision(start_pt)){
      ROS_WARN("This start point is insdide an obstacle.");
      do
        {
            start_pt = (start_pt - target_pt).normalized() * step_size + start_pt;
            if (!grid_map_->isInMap(start_pt))
                return false;
        } while (checkCollision(start_pt));
    }

    if(checkCollision(target_pt)){
      ROS_WARN("This target point is insdide an obstacle.");
      do
        {
            target_pt = (target_pt - start_pt).normalized() * step_size + target_pt;
            if (!grid_map_->isInMap(target_pt))
                return false;
        } while (checkCollision(target_pt));
    }
    
    return true;
}

bool DynamicPlanManager::checkCollision(const Eigen::Vector2d &pos){
  bool is_occ= grid_map_->getFusedDynamicInflateOccupancy(pos);
  return is_occ;
}

