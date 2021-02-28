#include "arena_intermediate_planner/intermediate_planner.h"


void InterPlanner::init(ros::NodeHandle & nh){
  node_=nh;

  /* init Grid map with occ & ESDF */
  grid_map_.reset(new GridMap);
  grid_map_->initMap(node_);

  /* init param: global planner */
  node_.param("global_planner/show_plan_time", pp_.show_plan_time_, true);
  node_.param("global_planner/use_astar", pp_.use_astar_,true);
  node_.param("global_planner/use_kino_astar", pp_.use_kino_astar_, true);
  node_.param("global_planner/use_oneshot", pp_.use_oneshot_, true);

  node_.param("global_planner/use_optimization_esdf", pp_.use_optimization_esdf_, true);
  node_.param("global_planner/use_optimization_astar", pp_.use_optimization_astar_, true);
  node_.param("global_planner/time_alloc_coefficient", pp_.time_alloc_coefficient_, 0.5);
  
  node_.param("subgoal/dist_lookahead", pp_.dist_lookahead_, 3.0);
  node_.param("subgoal/dist_tolerance", pp_.dist_tolerance_, 0.5);
  
  std::cout<<"dist_tolerance:"<<pp_.dist_tolerance_<<std::endl;

  node_.param("b_spline/max_vel",  pp_.max_vel_, 3.0);
  node_.param("b_spline/max_acc",  pp_.max_acc_, 2.0);
  node_.param("b_spline/max_jerk", pp_.max_jerk_, 4.0);
  node_.param("b_spline/control_points_distance", pp_.ctrl_pt_dist_, 0.5);
  node_.param("b_spline/feasibility_tolerance", pp_.feasibility_tolerance_, 0.05);


  /* init global data: save global plan & landmark & local target */
  global_data_.setGlobalDataParam(pp_.dist_lookahead_,pp_.dist_tolerance_);

  /* init planners */
  // use_astar, use_kino_astar
  if(pp_.use_astar_){
    global_planner_astar_.reset(new JPS);
    global_planner_astar_->setEnvironment(grid_map_);
    global_planner_astar_->setSearchMap(Eigen::Vector2i(1000, 1000),0.1,0.20);
  }

  if(pp_.use_kino_astar_){
    global_planner_kino_astar_.reset(new KinodynamicAstar);
    global_planner_kino_astar_->setParam(node_);
    global_planner_kino_astar_->setEnvironment(grid_map_);
    global_planner_kino_astar_->init();
    global_planner_kino_astar_->reset();
  }
  
  if (pp_.use_optimization_esdf_) {
      bspline_optimizer_esdf_.reset(new BsplineOptimizerESDF);
      bspline_optimizer_esdf_->setParam(node_);
      bspline_optimizer_esdf_->setEnvironment(grid_map_);
  }

  if (pp_.use_optimization_astar_) {
      bspline_optimizer_rebound_.reset(new BsplineOptimizerAstar);
      bspline_optimizer_rebound_->setParam(node_);
      bspline_optimizer_rebound_->setEnvironment(grid_map_);
      bspline_optimizer_rebound_->a_star_.reset(new AStar);
      bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(200, 200));
  }

  // odom
  have_odom_=false;

  // init a public node to publish or subscribe without node's private name
  ros::NodeHandle public_nh("");
  // subscriber
  goal_sub_ =public_nh.subscribe("goal", 1, &InterPlanner::goalCallback,this);
  odom_sub_ = public_nh.subscribe("odometry/ground_truth", 1, &InterPlanner::odomCallback, this);

  // publisher
  // kino_astar_path_pub_ =public_nh.advertise<nav_msgs::Path>("kino_astar_path", 1);
  // kino_astar_traj_pub_ =public_nh.advertise<nav_msgs::Path>("kino_astar_traj", 1);
  // kino_astar_waypoints_pub_ = public_nh.advertise<visualization_msgs::Marker>("kino_astar_wps", 20);

  // astar_path_pub_ =public_nh.advertise<nav_msgs::Path>("astar_path", 1);
  // astar_traj_pub_ =public_nh.advertise<nav_msgs::Path>("astar_traj", 1);
  // astar_waypoints_pub_ = public_nh.advertise<visualization_msgs::Marker>("astar_wps", 20);

  // jps_path_pub_ =public_nh.advertise<nav_msgs::Path>("jps_path", 1);
  // jps_traj_pub_ =public_nh.advertise<nav_msgs::Path>("jps_traj", 1);
  // jps_waypoints_pub_ = public_nh.advertise<visualization_msgs::Marker>("jps_wps", 20);

  // oneshot_path_pub_ =public_nh.advertise<nav_msgs::Path>("oneshot_path", 1);
  // oneshot_traj_pub_ =public_nh.advertise<nav_msgs::Path>("oneshot_traj", 1);
  // oneshot_waypoints_pub_=public_nh.advertise<visualization_msgs::Marker>("oneshot_wps", 20);

  // vis_control_pts_astar_pub_=public_nh.advertise<visualization_msgs::Marker>("ControlPoints_astar", 20);
  // vis_control_pts_oneshot_pub_=public_nh.advertise<visualization_msgs::Marker>("ControlPoints_oneshot", 20);
  // vis_control_pts_kino_pub_=public_nh.advertise<visualization_msgs::Marker>("ControlPoints_kino", 20);

  // vis_control_pts_astar_optimized_pub_=public_nh.advertise<visualization_msgs::Marker>("ControlPoints_astar_opt", 20);
  // vis_control_pts_oneshot_optimized_pub_=public_nh.advertise<visualization_msgs::Marker>("ControlPoints_oneshot_opt", 20);
  // vis_control_pts_kino_optimized_pub_=public_nh.advertise<visualization_msgs::Marker>("ControlPoints_kino_opt", 20);
  
  vis_goal_pub_ = node_.advertise<visualization_msgs::Marker>("vis_goal", 20);
  vis_subgoal_pub_ = node_.advertise<visualization_msgs::Marker>("vis_subgoal", 20);
  vis_global_path_pub_=node_.advertise<nav_msgs::Path>("vis_global_path_pub_", 1);
  kino_astar_waypoints_pub_ = node_.advertise<visualization_msgs::Marker>("vis_landmarks_", 20);
  vis_local_traj_pub_=node_.advertise<nav_msgs::Path>("vis_local_traj", 1);

  /* service server */
  global_plan_service_server_=public_nh.advertiseService("global_kino_make_plan", &InterPlanner::makeGlobalPlanService, this);

}

void InterPlanner::odomCallback(const nav_msgs::OdometryConstPtr& msg){
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void InterPlanner::goalCallback(const geometry_msgs::PoseStampedPtr& msg){
  if(have_odom_==false) return;
      
  // start pt
  start_pt_  =  odom_pos_;
  start_vel_ =  odom_vel_;
  start_acc_.setZero();

  // end pt
  end_pt_(0) = msg->pose.position.x;    
  end_pt_(1) = msg->pose.position.y;
  end_vel_.setZero();
  end_acc_.setZero();

  // vis goal
  std::cout << "Goal set!" << std::endl;
  vector<Eigen::Vector2d> point_set;
  point_set.push_back(end_pt_);
  visualizePoints(point_set,0.5,Eigen::Vector4d(1, 1, 1, 1.0),vis_goal_pub_);
  

  // find path
  OptimizerType type_optimizer=InterPlanner::OptimizerType::GRADIENT_ASTAR;
  //planAstarTraj(start_pt_,end_pt_,type_optimizer);
  //planOneshotTraj(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_,end_acc_);
  //planKinoAstarTraj(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_,InterPlanner::OptimizerType::GRADIENT_ESDF);
  makeGlobalPlan(start_pt_,end_pt_);
}

bool InterPlanner::planOneshotTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                      const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc, OptimizerType type_optimizer)
{ 
  
  /* path search */
  std::cout<<"*******************************************************"<<std::endl;
  std::cout<<"[oneshot replan]start----------------------------------"<<std::endl;
  std::cout<<"*******************************************************"<<std::endl;
  

  // select points
  std::vector<Eigen::Vector2d> points;
  points.push_back(start_pos);
  points.push_back(end_pos);

  // insert intermediate points if too far
  std::vector<Eigen::Vector2d> inter_points;
  const double dist_thresh = 4.0;

  for (size_t i = 0; i < points.size() - 1; ++i){
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    
    if (dist > dist_thresh){
      // find how many inter points needed to be added
      int id_num = floor(dist / dist_thresh) + 1;

      // add points
      for (int j = 1; j < id_num; ++j){
        Eigen::Vector2d inter_pt =points.at(i) * (1.0 - double(j) / id_num)+points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());

  // write position matrix
  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(2, pt_num);  //(dim,pt_num)
  for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];
  
  // segements time allocation
  Eigen::Vector2d zero(0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
  {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
  }

  time(0) *= 2.0;                 // start gentlely
  time(time.rows() - 1) *= 2.0;   // stop gentlely

  /* generate init trajectory */
  PolynomialTraj init_traj;

  if (pos.cols() >= 3){
    init_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
  }else if (pos.cols() == 2){
    init_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
  }else{
    std::cout<<"[Oneshot traj]start point & end point are not valid, cannot init traj"<<std::endl;
    return false;
  }


  
  

  /* Optimize Path */
  
  // compute initial path according to time step
  std::vector<Eigen::Vector2d>init_path;
  constexpr double step_size_t = 0.1;
  int i_end = floor(init_traj.getTimeSum()/ step_size_t);
  for (int i = 1; i < i_end; i++){
    init_path.push_back(init_traj.evaluate(double(i * step_size_t)));
  }
  init_path.push_back(end_pos);

  // compute ts, point_set, derivatives
  std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
  double ts;

  bool is_global=true;
  if(is_global)
  {
    point_set.clear();
    bool is_pointset_success=getPointSet(init_path,point_set,ts);
  
    if(!is_pointset_success)
    {
      ROS_WARN_STREAM("[Oneshot replan] initial pointset is not available");
      return false;
    }
  }else
  {
    point_set=init_path;

  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(Eigen::Vector2d::Zero());

  // optimize
  UniformBspline global_traj;
  bool optimize_success;
  optimize_success=optimizePath(ts,point_set,start_end_derivatives,global_traj,type_optimizer);
  
  if(optimize_success) global_data_.resetGlobalData(global_traj);
  
  
  /* visualization */
  if(optimize_success){
    // optimized path, optimized ctrl pts, landmarks
    visualizePath( global_data_.getGlobalPath() ,oneshot_traj_pub_);
    visualizePoints(global_data_.getControlPoints(),0.3,Eigen::Vector4d(0.4,0.1,1.0,1),vis_control_pts_oneshot_optimized_pub_);
    visualizePoints(global_data_.getLandmarks(),0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),oneshot_waypoints_pub_);
  }

  //  initial path
  visualizePath(init_path ,oneshot_path_pub_);
  //  initial point set
  visualizePoints(point_set,0.3,Eigen::Vector4d(0.4,0.1,0.0,1),vis_control_pts_oneshot_pub_);
  
  
  t2_ = ros::WallTime::now();
  dur_=(t2_ - t1_).toSec();
  std::cout<<"[oneshot replan]-------optimize duration="<<dur_<<std::endl;

  return true;
}

bool InterPlanner::planAstarTraj( Eigen::Vector2d &start_pos, Eigen::Vector2d &end_pos, OptimizerType type_optimizer){
  /* path search */
  std::cout<<"******************************************************"<<std::endl;
  std::cout<<"[astar replan]start----------------------------------"<<std::endl;
  std::cout<<"******************************************************"<<std::endl;
  
  auto t_start=ros::WallTime::now();

  int astar_status, jps_status;

  /* astar------------------------------------------------------------------- */
  t1_ = ros::WallTime::now();
  astar_status = global_planner_astar_->search(start_pos, end_pos,false,false);

  if(astar_status==JPS::NO_PATH){
      std::cout << "[Astar replan]: Can't find path." <<std:: endl;
      return false;
  }else{
      std::cout << "[Astar replan]: Astar search success."<< std::endl;
  }

  t2_ = ros::WallTime::now();
  dur_=(t2_ - t1_).toSec();
  std::cout<<"[astar replan]-------search duration="<<dur_<<std::endl;

  visualizePath(global_planner_astar_->getPath() ,astar_path_pub_);
  visualizePoints(global_planner_astar_->getPath(),0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),astar_waypoints_pub_);

  /* jps------------------------------------------------------------------- */
  std::cout << "<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>" <<std:: endl;
  t1_ = ros::WallTime::now();
  jps_status = global_planner_astar_->search(start_pos, end_pos,true,false);

  if(jps_status==JPS::NO_PATH){
      std::cout << "[JPS replan]: Can't find path." <<std:: endl;
      return false;
  }else{
      std::cout << "[JPS replan]: JPS search success."<< std::endl;
  }

  t2_ = ros::WallTime::now();
  dur_=(t2_ - t1_).toSec();
  std::cout<<"[JPS replan]-------search duration="<<dur_<<std::endl;

  visualizePath(global_planner_astar_->getPath() ,jps_path_pub_);
  visualizePoints(global_planner_astar_->getPath(),0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),jps_waypoints_pub_);


  
  
  

  

  // /* traj optimization */
  // t1_ = ros::WallTime::now();
  
  // // get ts, pointset, derivatives
  // double ts;
  // std::vector<Eigen::Vector2d> init_path, point_set, start_end_derivatives;
  // init_path=global_planner_astar_->getPath();
  // std::cout<<"[astar replan]-------getpath success"<<dur_<<std::endl;
  
  // //bool is_pointset_success=getPointSet(init_path,point_set,ts);
  // std::cout<<"[astar replan]-------getpointset success"<<dur_<<std::endl;
  // bool is_pointset_success=true;
  // point_set=init_path;
  
  
  
  // if(!is_pointset_success){
  //   ROS_WARN_STREAM("pointset is not available");
  //   return false;
  // }

  // start_end_derivatives.push_back(Eigen::Vector2d::Zero());
  // start_end_derivatives.push_back(Eigen::Vector2d::Zero());
  // start_end_derivatives.push_back(Eigen::Vector2d::Zero());
  // start_end_derivatives.push_back(Eigen::Vector2d::Zero());
  
  // // optimize
  // UniformBspline global_traj;
  // bool optimize_success;

  // optimize_success=optimizePath(ts,point_set,start_end_derivatives,global_traj,type_optimizer);


  // if(optimize_success){
  //   global_data_.resetGlobalData(global_traj);
  // }else{
  //   global_data_.resetGlobalData(global_traj);
  //   // Eigen::MatrixXd ctrl_pts;
  //   // UniformBspline::parameterizeToBspline(ts, init_path, start_end_derivatives, ctrl_pts);
  //   // global_traj = UniformBspline(ctrl_pts, 3, ts);
  //   // global_data_.resetGlobalData(global_traj);
  // }
  
  
  // /* visualization */
  // if(optimize_success){
  //   std::vector<Eigen::Vector2d> global_traj_optimized,landmark_pts;
  //   global_data_.getGlobalPath(global_traj_optimized);
  //   global_data_.getLandmarks(landmark_pts);
  //   visualizePath(global_traj_optimized ,astar_traj_pub_);
  //   visualizePoints(landmark_pts,0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),astar_waypoints_pub_);
  // }else{
  //   std::vector<Eigen::Vector2d> global_traj_optimized,landmark_pts;
  //   global_data_.getGlobalPath(global_traj_optimized);
  //   global_data_.getLandmarks(landmark_pts);
  //   //visualizePath(global_traj_optimized ,astar_traj_pub_);
  //   //visualizePoints(landmark_pts,0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),astar_waypoints_pub_);

  // }


  
  // visualizePoints(point_set,0.3,Eigen::Vector4d(0.4,0.1,0.0,1),vis_control_pts_astar_pub_);
  // visualizePoints(global_data_.getControlPoints(),0.3,Eigen::Vector4d(0.4,0.1,1.0,1),vis_control_pts_astar_optimized_pub_);
 
  t2_ = ros::WallTime::now();
  dur_=(t2_ - t1_).toSec();
  //std::cout<<"[astar replan]-------optimize duration="<<dur_<<std::endl;
  //std::cout<<"[astar replan]-------astar sum="<<(t2_-t_start).toSec()<<std::endl;
  return true;
}

bool InterPlanner::planKinoAstarTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc, const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, OptimizerType type_optimizer){
    std::cout<<"******************************************************"<<std::endl;
    std::cout<<"[kino replan]start----------------------------------"<<std::endl;
    std::cout<<"******************************************************"<<std::endl;
    
    ros::WallTime t_search_start = ros::WallTime::now();
    /* initialize reset --------------------------------------------*/
    int status;
    global_planner_kino_astar_->reset();
    
    /* path search --------------------------------------------*/
    status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, true);

    if (status == KinodynamicAstar::NO_PATH) {
      // search again
      //std::cout << "[kino replan]: first search fail!" << std::endl;

      // retry searching with discontinuous initial state
      global_planner_kino_astar_->reset();
      status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, false);

      if (status == KinodynamicAstar::NO_PATH) {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;

      } else {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    } else {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    //std::vector<Eigen::Vector2d> global_path=global_planner_kino_astar_->getKinoTraj(0.01); // delta_t=0.01s
    
    ros::WallTime t_search_end = ros::WallTime::now();
    
    
    /* traj optimization --------------------------------------------*/
    ros::WallTime t_opt_start = ros::WallTime::now();

    // compute initial path according to time step
    double step_size_t = pp_.ctrl_pt_dist_ / pp_.max_vel_;
    std::vector<Eigen::Vector2d> init_path,start_end_derivatives;
    global_planner_kino_astar_->getSamples(step_size_t, init_path, start_end_derivatives);

    // get ts, pointset, derivatives
    double ts=step_size_t;
    std::vector<Eigen::Vector2d> point_set;
    point_set=init_path;

    // optimize
    UniformBspline global_traj;
    bool optimize_success;
    optimize_success=optimizePath(ts,point_set,start_end_derivatives,global_traj,type_optimizer);
    
    ros::WallTime t_opt_end = ros::WallTime::now();

    

    if(optimize_success){
      global_data_.resetGlobalData(global_traj);

      std::cout << "[kino replan]: optimization success." << std::endl;
      visualizePath(global_data_.getGlobalPath() ,kino_astar_traj_pub_);
      visualizePoints(global_data_.getControlPoints(),0.3,Eigen::Vector4d(0.4,0.1,1.0,1),vis_control_pts_kino_optimized_pub_);
      visualizePoints(global_data_.getLandmarks(),0.2,Eigen::Vector4d(0.5, 0.5, 0.5, 0.6),kino_astar_waypoints_pub_);
       // init path, init control pts
      visualizePath(init_path ,kino_astar_path_pub_);
      visualizePoints(point_set,0.3,Eigen::Vector4d(0.4,0.1,0.0,1),vis_control_pts_kino_pub_);
    }else{
      std::cout << "[kino replan]: optimization failed." << std::endl;
      // init path, init control pts
      visualizePath(init_path ,kino_astar_path_pub_);
      visualizePoints(point_set,0.3,Eigen::Vector4d(0.4,0.1,0.0,1),vis_control_pts_kino_pub_);
    }


    double dur_search=(t_search_end - t_search_start).toSec();
    double dur_opt=(t_opt_end - t_opt_start).toSec();
    double dur_sum=(t_opt_end-t_search_start).toSec();
    std::cout<<"[kino replan]-------search duration   ="<<dur_search<<std::endl;
    std::cout<<"[kino replan]-------optimize duration ="<<dur_opt<<std::endl;
    std::cout<<"[kino replan]-------kino sum          ="<<dur_sum<<std::endl;
    
    if(!optimize_success)
      return false;

    return true;
}

bool InterPlanner::optimizePath(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives, UniformBspline & bspline_traj, OptimizerType type_optimizer){
    
    
    int trial_num=0;
    double ts0=ts;
    while(trial_num<10){
      
      trial_num++;
      ts=trial_num*ts0;
      //std::cout<<"Optimizing time:"<<trial_num<<"ts="<<ts<<"**********************"<<std::endl;
      // init B-spline control points
      Eigen::MatrixXd ctrl_pts;
      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

      /* -------------------Optimize Astar----------------------------------- */
      if(type_optimizer==InterPlanner::OptimizerType::GRADIENT_ASTAR)
      {
        std::vector<std::vector<Eigen::Vector2d>> a_star_pathes;
        a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
        
        
        if (!flag_step_1_success)
        {   
          std::cout<<"[bspline optimize_astar]:FAILED, with trial num:"<<trial_num<<std::endl;
          
          //return false;
          continue;
        }else{
          std::cout<<"[bspline optimize_astar]:SUCCESS, with trial num:"<<trial_num<<std::endl;
        }

        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;

        if (!pos.checkFeasibility(ratio, false))
        {
            //std::cout << "Need to reallocate time." << std::endl;
            Eigen::MatrixXd optimal_control_points;
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
            if (flag_step_2_success){
                pos = UniformBspline(optimal_control_points, 3, ts);
            }else{
                printf("\033[34mThis refined trajectory hits obstacles.It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            }    
        }

        bspline_traj=pos;
        return true;
      }

      /* -------------------Optimize ESDF----------------------------------- */
      if(type_optimizer==InterPlanner::OptimizerType::GRADIENT_ESDF)
      {

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
    }
    
    return false;
      
}

bool InterPlanner::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector2d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
{
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
}

void InterPlanner::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector2d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
{
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector2d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
}

void InterPlanner::visualizePath(const vector<Eigen::Vector2d> path, const ros::Publisher & pub){

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

void InterPlanner::visualizePoints(const vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub) {
  
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
  for (unsigned int i = 0; i < int(point_set.size()); i++) {
    pt.x = point_set[i](0);
    pt.y = point_set[i](1);
    pt.z = 0.0;
    mk.points.push_back(pt);
  }
  pub.publish(mk);
  ros::Duration(0.001).sleep();
}

/* Helper */
bool InterPlanner::checkCollision(const Eigen::Vector2d &pos){
  bool is_occ= grid_map_->getFusedInflateOccupancy(pos);
  return is_occ;
}

bool InterPlanner::checkColiisionSegment(Eigen::Vector2d pt1, Eigen::Vector2d pt2){
    double dist=(pt1-pt2).norm();                            // distance start pt to end pt
    double dist_step=grid_map_->getResolution()*2;           // collision check step distance
    int id_num = floor(dist / dist_step) + 1;

    Eigen::Vector2d inter_pt;
    for (int j = 1; j < id_num; ++j)
    {
      inter_pt = pt1 * (1.0 - double(j) / id_num) + pt2 * double(j) / id_num;
      if(checkCollision(inter_pt)){
        return true;
      }    
    }
    return false;
}

bool InterPlanner::findCollisionWithinSegment(const Eigen::Vector2d &pt1,const Eigen::Vector2d &pt2,vector<Eigen::Vector2d> & inter_points){
  double dist=(pt1-pt2).norm();                            // distance start pt to end pt
  double dist_step=grid_map_->getResolution()*4;                 // collision check step distance

  int id_num = floor(dist / dist_step) + 1;

  for (int j = 1; j < id_num; ++j)
  {
    Eigen::Vector2d inter_pt = pt1 * (1.0 - double(j) / id_num) + pt2 * double(j) / id_num;
    if(checkCollision(inter_pt)){
      inter_points.push_back(inter_pt);
    }    
  }

  if(inter_points.size()>0){
    //std::cout<<"this segment collision num is "<<inter_points.size()<<std::endl;
    inter_points.push_back(pt2);
    return true;
  }else{
    //std::cout<<"this segment collision is empty"<<std::endl;
    inter_points.push_back(pt2);
    return false;
  }
  
}

bool InterPlanner::getPointSet(const vector<Eigen::Vector2d> &path, vector<Eigen::Vector2d> &point_set, double &ts){
  /* input: a path ,each waypoint has distance small or big 
     output: point set for bspline generation: according to  global path or local path
     param: pt_distance,  collision check resolution
  */

  if(path.size()<2){
    ROS_WARN_STREAM("[Get point set]: not enough initial path point, num="<<path.size());
    return false;
  }
  /* init varibales*/
  Eigen::Vector2d start_pt,end_pt;
  double start_end_dist;
  start_pt=path.front();
  end_pt=path.back();
  start_end_dist=(end_pt-start_pt).norm();
  double dist_thresh;
  int min_num=5;

  /* set dist_thresh */
  if (start_end_dist>min_num*pp_.ctrl_pt_dist_)
  { 
    // if start_end_dist is big, set dist_thresh as ctrl_pt_dist_
    dist_thresh=pp_.ctrl_pt_dist_;
  }else{
    // if start_end_dist is small, set dist_thresh according to minimum num of waypoints
    dist_thresh=start_end_dist/double(min_num+1);
  }

  // dist_thresh should not be too small
  if(dist_thresh< grid_map_->getResolution()){
    ROS_WARN_STREAM("[Get point set]: too short initial path, dist_thresh="<<dist_thresh);
    return false;
  }
  
  /* add to point_set */
  // init point_set
  point_set.clear();
  point_set.push_back(start_pt);          // add start pt
  Eigen::Vector2d last_pt=start_pt;       // init last pt

  for (size_t i = 1; i < path.size() - 1; ++i)
  {
    double dist = (path[i] - last_pt).norm();

    if(dist>dist_thresh)
    {
      while(dist>2*dist_thresh)
      {
        // select a point at distance=dist_thresh
        Eigen::Vector2d curr_pt=last_pt*(1-dist_thresh/dist) + path[i] * (dist_thresh/dist);

        // add intermediate collision points to point_set
        std::vector<Eigen::Vector2d> inter_points;// include the end point in collision point
        findCollisionWithinSegment(last_pt,curr_pt,inter_points);
        
        for(unsigned int j=0;j<inter_points.size();j++)
        {
            point_set.push_back(inter_points[j]);
        }
        
        
        // update last_pt in point_set
        dist=(path[i] - curr_pt).norm();
        if(dist<dist_thresh)
        {
          point_set.push_back(path[i]);
          last_pt=path[i];
          break;
        }else
        {
          last_pt=curr_pt;
        }
      }
    }else if(i<=3){
      // in order to let first 3 point be obs free.
      point_set.push_back(path[i]);
      last_pt=path[i];
    }
  }

  if(point_set.back()!=end_pt)
  {
      // make sure the endpoint is added
      point_set.push_back(end_pt); 
  }


  ts=dist_thresh/pp_.max_vel_*pp_.time_alloc_coefficient_;

  /* lack of points in point_set */
  if(point_set.size()<min_num)
  {
    point_set.clear();
    if(path.size()>=min_num){
      point_set=path;
    }else{
      for(int i=0;i<min_num;i++){
        Eigen::Vector2d pt=start_pt*(1-double(i)/min_num) + end_pt * (double(i)/min_num);
        point_set.push_back(pt);
      }
    }
    ts=start_end_dist/min_num/pp_.max_vel_*pp_.time_alloc_coefficient_;
  }

  if(point_set.size()<min_num){
    ROS_WARN_STREAM("[Get point set]: not enough points in optimized point_set, num="<<point_set.size());
    std::cout<<"dist_thresh:"<<dist_thresh<<std::endl;
    return false;
  }
  
  return true;

}

/* ROS Service */
bool InterPlanner::makeGlobalPlanService(arena_plan_msgs::MakeGlobalPlan::Request  &req, arena_plan_msgs::MakeGlobalPlan::Response &res){

  Eigen::Vector2d start_pos,end_pos;
  start_pos(0)  =  req.start.pose.position.x;
  start_pos(1)  =  req.start.pose.position.y;

  // end pt
  end_pos(0) = req.goal.pose.position.x;
  end_pos(1) = req.goal.pose.position.y;
  

  /* kino astar */
  /* path search */
    int status;
    global_planner_kino_astar_->reset();
    
    // first search
    status = global_planner_kino_astar_->search(start_pos, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), end_pos, Eigen::Vector2d::Zero(), true);

    if (status == KinodynamicAstar::NO_PATH) {
      // search again
      std::cout << "[kino replan]: first search fail!" << std::endl;

      // retry searching with discontinuous initial state
      global_planner_kino_astar_->reset();
      status = global_planner_kino_astar_->search(start_pos, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), end_pos, Eigen::Vector2d::Zero(), false);

      if (status == KinodynamicAstar::NO_PATH) {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;

      } else {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    } else {
        std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    std::vector<Eigen::Vector2d> global_path=global_planner_kino_astar_->getKinoTraj(0.01); // delta_t=0.01s
    
    visualizePath(global_path ,kino_astar_path_pub_);

    std::vector<geometry_msgs::PoseStamped> plan;

    for(unsigned int i=0;i<global_path.size();i++){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map"; //global_frame_;
      pose.pose.position.x = global_path[i](0);
      pose.pose.position.y = global_path[i](1);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }
    res.plan.poses=plan;
    res.plan.header.stamp = ros::Time::now();
    res.plan.header.frame_id = "map";//global_frame_;

  return true;
}

bool InterPlanner::makeGlobalPlan(Eigen::Vector2d start_pos,Eigen::Vector2d end_pos){

  // vis goal
  // std::cout << "Goal set!" << std::endl;
  // vector<Eigen::Vector2d> vis_point_set;
  // vis_point_set.push_back(end_pos);
  // visualizePoints(vis_point_set,0.5,Eigen::Vector4d(1, 1, 1, 1.0),vis_goal_pub_);

  // get plan
  OptimizerType type_optimizer=InterPlanner::OptimizerType::GRADIENT_ESDF;
  bool success;
  // adjust start and target pt to be in free space
  adjustStartAndTargetPoint(start_pos,end_pos);

  success=planKinoAstarTraj(start_pos, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), end_pos, Eigen::Vector2d::Zero(),type_optimizer);
  
  if(success){
    visualizePath(global_data_.getGlobalPath() ,vis_global_path_pub_);
    //makeSubgoal(odom_pos_,odom_vel_);
    return true;
  }else{
    return false;
  }
}

bool InterPlanner::makeOneshotPlan(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc,
                                const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel, const Eigen::Vector2d &end_acc)
{
  ros::WallTime t_plan_start = ros::WallTime::now();

  std::vector<Eigen::Vector2d> points;
  points.push_back(start_pos);
  points.push_back(end_pos);

  // insert intermediate points if too far
  std::vector<Eigen::Vector2d> inter_points;
  const double dist_thresh = 2.0;

  for (size_t i = 0; i < points.size() - 1; ++i){
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    
    if (dist > dist_thresh){
      // find how many inter points needed to be added
      int id_num = floor(dist / dist_thresh) + 1;

      // add points
      for (int j = 1; j < id_num; ++j){
        Eigen::Vector2d inter_pt =points.at(i) * (1.0 - double(j) / id_num)+points.at(i + 1) * double(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());

  // write position matrix
  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(2, pt_num);  //(dim,pt_num)
  for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];
  
  // segements time allocation
  Eigen::Vector2d zero(0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
  {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
  }

  time(0) *= 2.0;                 // start gentlely
  time(time.rows() - 1) *= 2.0;   // stop gentlely

  /* generate init trajectory */
  PolynomialTraj init_traj;

  if (pos.cols() >= 3){
    init_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
  }else if (pos.cols() == 2){
    init_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
  }else{
    std::cout<<"[Oneshot traj]start point & end point are not valid, cannot init traj"<<std::endl;
    return false;
  }

  /* Optimize Path */
  
  // compute initial path according to time step
  std::vector<Eigen::Vector2d>init_path;
  constexpr double step_size_t = 0.1;
  int i_end = floor(init_traj.getTimeSum()/ step_size_t);
  for (int i = 1; i < i_end; i++){
    init_path.push_back(init_traj.evaluate(double(i * step_size_t)));
  }
  init_path.push_back(end_pos);

  // compute ts, point_set, derivatives
  std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
  double ts=step_size_t;
  point_set=init_path;
  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(Eigen::Vector2d::Zero());

  // optimize
  UniformBspline local_traj;
  bool optimize_success;
  optimize_success=optimizePath(ts,point_set,start_end_derivatives,local_traj,OptimizerType::GRADIENT_ASTAR);

  ros::WallTime t_plan_end= ros::WallTime::now();
  double dur_sum=(t_plan_end-t_plan_start).toSec();
  std::cout<<"[local traj replan]-------sum   ="<<dur_sum<<std::endl;

  if(optimize_success){
    mid_data_.subgoal_traj_=local_traj;
    return true;
  }else{
    return false;
  }

}

bool InterPlanner::makeSubgoal(Eigen::Vector2d curr_pos, Eigen::Vector2d curr_vel, double T_subgoal ){
  bool success;
  // get initial local target
  Eigen::Vector2d local_target=global_data_.getLocalTarget(curr_pos);
  
  // make the local target not on obstacle
  if(!adjustStartAndTargetPoint(local_target,curr_pos)){
    return false;
  }
  // std::vector<Eigen::Vector2d> landmarks=global_data_.getLandmarks();
  // for(size_t i=0;i<landmarks.size();++i){
  //   std::cout<<"landmarks:"<<landmarks[i]<<std::endl;
  // }
  
  //std::cout<<"local odom:"<<odom_pos_<<std::endl;
  //std::cout<<"local target:"<<local_target<<std::endl;
  // make oneshot plan
  success=makeOneshotPlan(curr_pos,curr_vel,Eigen::Vector2d::Zero(),local_target,Eigen::Vector2d::Zero(),Eigen::Vector2d::Zero());
  
  // select subgoal
  if(success){
    //double T_subgoal=3.0;
    // select the pt at 3s on the traj
    double tm_start,tm_end;
    mid_data_.subgoal_traj_.getTimeSpan(tm_start, tm_end);
    mid_data_.subgoal_=mid_data_.subgoal_traj_.evaluateDeBoor(tm_start+0.5*(tm_end-tm_start));
    mid_data_.subgoal_traj_end_=mid_data_.subgoal_traj_.evaluateDeBoor(tm_end);
    
    // visualize subgoal
    std::vector<Eigen::Vector2d> point_set;
    point_set.push_back(mid_data_.subgoal_);
    visualizePoints(point_set,0.8,Eigen::Vector4d(0.5, 0.5, 1, 1.0),vis_subgoal_pub_);
    visualizePath(mid_data_.getTraj(),vis_local_traj_pub_);
    return true;
  }

  return false;
}

bool InterPlanner::adjustStartAndTargetPoint( Eigen::Vector2d &target_pt, Eigen::Vector2d &start_pt)
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




int main(int argc, char **argv){
    
    ros::init(argc, argv, "inter");
    std::cout<<"start"<<std::endl;
    ros::NodeHandle nh("~");

    InterPlanner::Ptr gp;

    gp.reset(new InterPlanner);
    gp->init(nh);

    ros::spin();
    return 0;
}



