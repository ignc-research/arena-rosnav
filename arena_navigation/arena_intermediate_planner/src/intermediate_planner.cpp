#include "arena_intermediate_planner/intermediate_planner.h"


void InterPlanner::init(ros::NodeHandle & nh){
  node_=nh;

  // map
  grid_map_.reset(new GridMap);
  grid_map_->initMap(node_);

  // global planner
  node_.param("global_planner/use_astar", pp_.use_astar_,false);
  node_.param("global_planner/use_kino_astar", pp_.use_kino_astar_, true);
  node_.param("global_planner/use_oneshot", pp_.use_oneshot_, false);

  node_.param("global_planner/dist_next_wp", pp_.dist_next_wp_, 133.0);
  node_.param("global_planner/dist_tolerance", pp_.dist_tolerance_, 1.0);
  node_.param("global_planner/rou_thresh", pp_.rou_thresh_, 15.0);
  
  std::cout<<"dist_next_wp_="<<pp_.dist_next_wp_<<std::endl;

  node_.param("b_spline/max_vel",  pp_.max_vel_, 3.0);
  node_.param("b_spline/max_acc",  pp_.max_acc_, 2.0);
  node_.param("b_spline/max_jerk", pp_.max_jerk_, 4.0);
  node_.param("b_spline/control_points_distance", pp_.ctrl_pt_dist_, 0.5);
  node_.param("b_spline/feasibility_tolerance", pp_.feasibility_tolerance_, 0.05);
  node_.param("b_spline/planning_horizon", pp_.planning_horizen_, 7.5);

  node_.param("global_planner/use_optimization_esdf", pp_.use_optimization_esdf_, true);
  node_.param("global_planner/use_optimization_astar", pp_.use_optimization_astar_, true);
  
  // init global data
  global_data_.setGlobalDataParam(pp_.dist_next_wp_,pp_.dist_tolerance_,pp_.rou_thresh_);

  // use_astar, use_kino_astar
  if(pp_.use_astar_){
    //global_planner_type_="astar";
    global_planner_astar_.reset(new Astar);
    global_planner_astar_->setParam(node_);
    global_planner_astar_->setEnvironment(grid_map_);
    global_planner_astar_->init();
    global_planner_astar_->reset();
  }

  if(pp_.use_kino_astar_){
    //global_planner_type_="kino_astar";
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
      bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(100, 100));
  }

  // odom
  have_odom_=false;

  ros::NodeHandle public_nh("");
  // subscriber
  goal_sub_ =public_nh.subscribe("goal", 1, &InterPlanner::goalCallback,this);
  odom_sub_ = public_nh.subscribe("odometry/ground_truth", 1, &InterPlanner::odomCallback, this);

  // publisher
  kino_astar_path_pub_ =public_nh.advertise<nav_msgs::Path>("kino_astar_path", 1);
  kino_astar_traj_pub_ =public_nh.advertise<nav_msgs::Path>("kino_astar_traj", 1);

  
  global_plan_service_server_=public_nh.advertiseService("global_kino_make_plan", &InterPlanner::makeGlobalPlan, this);
  visualization_.reset(new PlanningVisualization(node_));
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
  geometry_msgs::PoseStamped p;
  p.header=msg->header;     
  p.pose=msg->pose;
  visualization_->drawGoal(p, 0.5, Eigen::Vector4d(1, 1, 1, 1.0));
  
  // rotation matrix
  //Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);

  // find path
  //findPath( start_pt_, start_vel_, start_acc_, end_pt_, end_vel_ );
  
  //bool find_global_traj_success;
  //find_global_traj_success=planKinoAstarTraj(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  //find_global_traj_success = planGlobalTraj(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, end_acc_);
}

bool InterPlanner::makeGlobalPlan(arena_plan_msgs::MakeGlobalPlan::Request  &req, arena_plan_msgs::MakeGlobalPlan::Response &res){
  std::cout<<"Service to plan global0"<<std::endl;
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
      //std::cout << "[kino replan]: first search fail!" << std::endl;

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
    
    visualize_path(global_path ,kino_astar_path_pub_);

    std::vector<geometry_msgs::PoseStamped> plan;

    for(int i=0;i<global_path.size();i++){
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

bool InterPlanner::planKinoAstarTraj(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &start_vel, const Eigen::Vector2d &start_acc, const Eigen::Vector2d &end_pos, const Eigen::Vector2d &end_vel){
    /* path search */
    int status;
    global_planner_kino_astar_->reset();
    
    // first search
    status = global_planner_kino_astar_->search(start_pos, start_vel, start_acc, end_pos, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      // search again
      std::cout << "[kino replan]: first search fail!" << std::endl;

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

    std::vector<Eigen::Vector2d> global_path=global_planner_kino_astar_->getKinoTraj(0.01); // delta_t=0.01s

    visualize_path(global_path ,kino_astar_path_pub_);

    /* traj optimization */
    // get ts, pointset, derivatives
    double ts = pp_.ctrl_pt_dist_ / pp_.max_vel_;
    std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
    global_planner_kino_astar_->getSamples(ts, point_set, start_end_derivatives);
    
    // optimize
    UniformBspline global_traj;
    bool optimize_success;
    optimize_success=optimizePath(ts,point_set,start_end_derivatives,global_traj,InterPlanner::OptimizerType::GRADIENT_ESDF);
    
    if(optimize_success){
        std::vector<Eigen::Vector2d> traj_pts_esdf,traj_pts_astar,landmark_pts;
        global_data_.resetGlobalData(global_traj);
        global_data_.getGlobalPath(traj_pts_esdf);
        global_data_.getLandmarks(landmark_pts);
        visualize_path(traj_pts_esdf ,kino_astar_traj_pub_);

        nav_msgs::Path ctrl_points_path;
        ctrl_points_path.poses.resize(int(landmark_pts.size()));
    

        for (int i = 0; i < int(landmark_pts.size()); ++i) {
          Eigen::Vector2d pt = landmark_pts[i];
          ctrl_points_path.poses[i].pose.position.x=pt(0);
          ctrl_points_path.poses[i].pose.position.y=pt(1);
        }

        visualization_->drawGlobalPath(ctrl_points_path,0.2, Eigen::Vector4d(0.5, 0.5, 0.5, 0.6));
    }
    
    
    t1_ = ros::WallTime::now();
    t2_ = ros::WallTime::now();
    dur_=(t2_ - t1_).toSec();
    std::cout<<"-------kino astar end,duration="<<dur_<<std::endl;
    return true;
}

bool InterPlanner::optimizePath(double ts,std::vector<Eigen::Vector2d> point_set, std::vector<Eigen::Vector2d> start_end_derivatives, UniformBspline & bspline_traj, OptimizerType type_optimizer){
    // init B-spline control points
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    
    if(type_optimizer==InterPlanner::OptimizerType::GRADIENT_ASTAR){
        std::vector<std::vector<Eigen::Vector2d>> a_star_pathes;
        a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
        std::cout << "[bspline optimize_astar]first_optimize_step_success=" << flag_step_1_success << std::endl;

        if (!flag_step_1_success)
        {
            return false;
        }

        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;

        if (!pos.checkFeasibility(ratio, false))
        {
            std::cout << "Need to reallocate time." << std::endl;
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

    if(type_optimizer==InterPlanner::OptimizerType::GRADIENT_ESDF){

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

        
        std::cout << "[kino replan]: Reallocate ratio: " << tn / to << std::endl;
        if (tn / to > 3.0) ROS_ERROR("reallocate error.");

        bspline_traj=pos;
        return true;
    }

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

void InterPlanner::visualize_path(std::vector<Eigen::Vector2d> path, const ros::Publisher & pub){

  //create a path message
  ros::Time plan_time = ros::Time::now();
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



