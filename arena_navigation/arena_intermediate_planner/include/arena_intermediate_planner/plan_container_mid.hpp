#ifndef _PLAN_CONTAINER_MID_H_
#define _PLAN_CONTAINER_MID_H_


#include <Eigen/Eigen>
#include <vector>

#include <ros/ros.h>

#include <arena_traj_planner/uniform_bspline.h>
#include <arena_traj_planner/polynomial_traj.h>

using std::vector;


struct LandmarkPoint{
  bool is_visited;
  int id;
  Eigen::Vector2d pos;


  LandmarkPoint(Eigen::Vector2d pos,int id){
    this->pos=pos;
    this->id=id;
    this->is_visited=false;

  }

  ~LandmarkPoint(){};

  void setVisited(bool is_visited){
    this->is_visited=is_visited;
  }
};



class GlobalData{
private:
  
  double dist_lookahead_;     // look ahead distance
  double dist_tolerance_;     // torlerance for reaching the local target
  int id_last_landmark_;      // save last landmark id


  void resetLandmarks()
  {
    global_path_.clear();
    landmark_points_.clear();

    double angle_sum=0;
    double angle_delta;
    double angle_vel, angle_vel_last=0.0;
    
    double delta_t=0.01;

    double angle_thresh=20*3.14/180;
    double dist_thresh=dist_lookahead_;
    double min_angle_vel=5*3.14/180;
    double max_angle_vel=360*3.14/180;


    // init last_landmark pos
    Eigen::Vector2d last_landmark=start_pos_;

    int id=0;
    int iter=0;
    int iter_sum=round((tm_end_-tm_start_)/delta_t);

    for (double t = tm_start_+delta_t; t < tm_end_; t += delta_t) 
    {
      iter++;
      // get pos
      Eigen::Vector2d pos_t = getPosition(t);
      global_path_.push_back(pos_t);

      // get angular vel
      angle_vel=calculateRotationVelocity(getVelocity(t),getAcceleration(t));
      //printf("\033[32miter(+1)=%d,sum=%d: angle vel=%5.3f, angle_sum=%5.3f\n\033[0m", iter, iter_sum,angle_vel*180/3.14,angle_sum*180/3.14);

      // pre-process angular vel( for condition1: to delay add landmark under condition1)  
      if(std::abs(angle_vel)<min_angle_vel){
        angle_vel=0;
      }else if(std::abs(angle_vel)>max_angle_vel){
        angle_vel=angle_vel>0?max_angle_vel:-max_angle_vel;
      }

      // check if near goal
      if((pos_t-end_pos_).norm()<dist_thresh){
        // near end
        //std::cout<<"near="<<std::endl;
        continue;
      }

      /* condition1 : angular velocity direction change condition(calulation using modified angular_vel)*/
      if(angle_vel_last*angle_vel<0){
        //std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^id="<<id<<std::endl;
        //std::cout<<"condition 1: change direction="<<angle_sum*180/3.14<<std::endl;
        //angle_sum=0.0; //(done by condition2 now)

        if((pos_t-last_landmark).norm()>dist_thresh)
        { 
          
          landmark_points_.push_back(LandmarkPoint(pos_t,id));
          last_landmark=pos_t;
          id++;
        }
      }

      if(angle_vel!=0.0)
      { 
        angle_vel_last=angle_vel;
      } 
      
      /* condition2: angular integral condition (calulation using real angular_vel) */
      if(calculateRotationVelocity(getVelocity(t),getAcceleration(t))*calculateRotationVelocity(getVelocity(t-delta_t),getAcceleration(t-delta_t))<0)
      {
        angle_sum=0.0;
      }
      //angle_delta=angle_vel*delta_t;
      angle_delta=calculateRotationVelocity(getVelocity(t),getAcceleration(t))*delta_t; // integral with original angular vel
      angle_sum=angle_sum+angle_delta;
      if(std::abs(angle_sum)>angle_thresh){
        //std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^id="<<id<<std::endl;
        //std::cout<<"condition 2: angle_sum="<<angle_sum*180/3.14<<std::endl;
        
        if((pos_t-last_landmark).norm()>dist_thresh)
        {
          landmark_points_.push_back(LandmarkPoint(pos_t,id));
          last_landmark=pos_t;
          id++;
          angle_sum=0.0;
        }
      } 
      
    }

    // add end_pos to global path
    landmark_points_.push_back(LandmarkPoint(end_pos_,id));

    global_path_.push_back(end_pos_);
    // reset landmark counter
    id_last_landmark_=0;
  }

  Eigen::Vector2d getPosition(double t){
      return global_pos_traj_.evaluateDeBoor(t);
  }

  Eigen::Vector2d getVelocity(double t){
      return global_vel_traj_.evaluateDeBoor(t);
  }

  Eigen::Vector2d getAcceleration(double t){
      return global_acc_traj_.evaluateDeBoor(t);
  }

  double calculateCurveRadius(Eigen::Vector2d vel,Eigen::Vector2d acc){
    double rou=std::pow(vel.norm(),3)/(vel(0)*acc(1)-vel(1)*acc(0));
    return std::abs(rou);
  }

  double calculateRotationVelocity(Eigen::Vector2d vel,Eigen::Vector2d acc){
    double a_norm=(vel(0)*acc(1)-vel(1)*acc(0))/vel.norm();
    double w=a_norm/vel.norm();
    return w;
  }

public:
  UniformBspline global_pos_traj_, global_vel_traj_, global_acc_traj_;
  double tm_start_, tm_end_;
  Eigen::Vector2d start_pos_, end_pos_;

  std::vector<Eigen::Vector2d> global_path_;
  std::vector<LandmarkPoint> landmark_points_;
  
  GlobalData(){};
  ~GlobalData(){};
  
  void setGlobalDataParam(double dist_lookahead, double dist_tolerance){
    dist_lookahead_=dist_lookahead;
    dist_tolerance_=dist_tolerance;

  }

  void resetGlobalData(const UniformBspline & pos_traj){
    global_pos_traj_= pos_traj;
    global_vel_traj_=global_pos_traj_.getDerivative();
    global_acc_traj_=global_vel_traj_.getDerivative();
    global_pos_traj_.getTimeSpan(tm_start_, tm_end_);
    start_pos_=getPosition(tm_start_);
    end_pos_=getPosition(tm_end_);
    resetLandmarks();
  }

  std::vector<Eigen::Vector2d> getGlobalPath()
  {
    return global_path_;   
  }

  std::vector<Eigen::Vector2d> getControlPoints(){
    Eigen::MatrixXd control_pts_matrix=global_pos_traj_.get_control_points();// (dim,cols)
    std::vector<Eigen::Vector2d> ctrl_pts;
    ctrl_pts.clear();
    for(int i=0;i<control_pts_matrix.cols();i++){
      ctrl_pts.push_back(control_pts_matrix.col(i));
    }
    return ctrl_pts;
  }
  
  std::vector<Eigen::Vector2d> getLandmarks(){
    std::vector<Eigen::Vector2d> landmark_pts;
    if(!landmark_points_.empty()){
      for(unsigned int i=0;i<landmark_points_.size();i++){
          landmark_pts.push_back(landmark_points_[i].pos);
      }
      return landmark_pts;
    }else{
      return landmark_pts;
    }
  }

  Eigen::Vector2d getCurrentLandmark(){
    return landmark_points_[id_last_landmark_].pos;
  }

  Eigen::Vector2d getPreviousLandmark(){
    if(id_last_landmark_>0){
      return landmark_points_[id_last_landmark_-1].pos;
    }else{
      return landmark_points_[0].pos;
    }
  }

  Eigen::Vector2d getNextLandmark(){
    if(id_last_landmark_<(int)landmark_points_.size()-1){
      return landmark_points_[id_last_landmark_+1].pos;
    }else{
      return landmark_points_[(int)landmark_points_.size()-1].pos;
    }
  }

  Eigen::Vector2d getLocalTarget(const Eigen::Vector2d &current_pt)


  {
    Eigen::Vector2d target_pt;
    Eigen::Vector2d landmark_pt;
    
    double dist_to_last_landmark;
    dist_to_last_landmark=(landmark_points_[id_last_landmark_].pos-current_pt).norm();

    Eigen::Vector2d vec_two_landmark;
    Eigen::Vector2d vec_to_last_landmark;
    Eigen::Vector2d vec_to_next_landmark;

    if(id_last_landmark_<(int)landmark_points_.size()-1)
    {
      // check if landmark point is arrived, and update target landmark id
      if(dist_to_last_landmark<dist_tolerance_)
      {
        // if near to last landmark
        id_last_landmark_++;
      }else{
        // if arrived at same horizon as next landmark &&  off the track to last landmark
        vec_two_landmark=landmark_points_[id_last_landmark_+1].pos-landmark_points_[id_last_landmark_].pos;
        vec_to_last_landmark=landmark_points_[id_last_landmark_].pos-current_pt;
        vec_to_next_landmark=landmark_points_[id_last_landmark_+1].pos-current_pt;
        double cos_theta_last=vec_two_landmark.dot(vec_to_last_landmark)/(vec_to_last_landmark.norm()*vec_two_landmark.norm());
        double cos_theta_next=vec_two_landmark.dot(vec_to_next_landmark)/(vec_to_next_landmark.norm()*vec_two_landmark.norm());
        
        if(cos_theta_next>0.995 && cos_theta_last<0) // 0.995 means value of cos(5 degree ) 
        {
          id_last_landmark_++;
        }
      }
    }

    // set landmark pos
    landmark_pt=landmark_points_[id_last_landmark_].pos;

    // calculate the target_pt at the direction of landmark pos
    double a;
    double further_factor=1.5;  //further_factor<2
    a=(further_factor*dist_lookahead_)/(current_pt-landmark_pt).norm(); // select a point a little further
    
    //std::cout<<"dist_lookahead_:"<<dist_lookahead_<<std::endl;
    //std::cout<<"dist current & landmark:"<<(current_pt-landmark_pt).norm()<<std::endl;
    //std::cout<<"a:"<<a<<std::endl;
    //std::cout<<"landmark_pt-current_pt="<<landmark_pt-current_pt<<std::endl;
    //std::cout<<"a*(landmark_pt-current_pt)="<<a*(landmark_pt-current_pt)<<std::endl;
    
    if(a<1.0){//further_factor
      // normally a=[0,1]
      target_pt=current_pt+a*(landmark_pt-current_pt);
    }else{
      // if a>1 or further_factor
      target_pt=landmark_pt;
    }

    return target_pt;
  }  
};

struct MidData{
  UniformBspline subgoal_traj_;
  Eigen::Vector2d subgoal_;

  Eigen::Vector2d getSubgoal(){
    return subgoal_;
  }
  MidData(){}
  ~MidData(){}
};



class GlobalTrajData
{
  private:
  public:
    PolynomialTraj global_traj_;
    std::vector<UniformBspline> local_traj_;

    double global_duration_;
    ros::Time global_start_time_;

    double local_start_time_, local_end_time_;
    double time_increase_;
    double last_time_inc_;
    double last_progress_time_;

    GlobalTrajData(/* args */) {}
    ~GlobalTrajData() {}

    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum();
      global_start_time_ = time;

      local_traj_.clear();
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }


    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);
      local_traj_[0] = traj;
      local_traj_[1] = local_traj_[0].getDerivative();
      local_traj_[2] = local_traj_[1].getDerivative();

      local_start_time_ = local_ts;
      local_end_time_ = local_te;
      global_duration_ += time_inc;
      time_increase_ += time_inc;
      last_time_inc_ = time_inc;
    }

    Eigen::Vector2d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector2d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_);
      }
    }

    Eigen::Vector2d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_);
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_);
      }
    }


    // get Bspline paramterization data of a local trajectory within a sphere
    // start_t: start time of the trajectory
    // dist_pt: distance between the discretized points
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector2d> &point_set, vector<Eigen::Vector2d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment
      double seg_time = 0.0;   // duration of the truncated segment
      double radius = 0.0;     // distance to the first point of the segment

      double delt = 0.2;
      Eigen::Vector2d first_pt = getPosition(start_t); // first point of the segment
      Eigen::Vector2d prev_pt = first_pt;              // previous point
      Eigen::Vector2d cur_pt;                          // current point

      // go forward until the traj exceed radius or global time
      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delt;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm();
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm();
      }

      // get parameterization dt by desired density of points
      int seg_num = floor(seg_length / dist_pt);

      // get outputs

      seg_duration = seg_time; // duration of the truncated segment
      dt = seg_time / seg_num; // time difference between two points

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + seg_time));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));
    }
    
    // get Bspline paramterization data of a fixed duration local trajectory
    // start_t: start time of the trajectory
    // duration: time length of the segment
    // seg_num: discretized the segment into *seg_num* parts
    void getTrajByDuration(double start_t, double duration, int seg_num,vector<Eigen::Vector2d> &point_set,vector<Eigen::Vector2d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;
      Eigen::Vector2d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));
      start_end_derivative.push_back(getVelocity(start_t + duration));
      start_end_derivative.push_back(getAcceleration(start_t));
      start_end_derivative.push_back(getAcceleration(start_t + duration));
    }

};

struct PlanParameters
{
    /* planning algorithm parameters */
    double max_vel_, max_acc_, max_jerk_; // physical limits
    double ctrl_pt_dist_;                  // distance between adjacient B-spline control points
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits

    /* global data param: subgoal   */
    double dist_lookahead_;           // distance from current location to next waypoint
    double dist_tolerance_;         // distance tolerance for arriving
  

    /* global plan param */
    bool use_astar_, use_kino_astar_, use_oneshot_;
    bool use_optimization_esdf_, use_optimization_astar_;
    double time_alloc_coefficient_;

    /* performance compare flag */
    bool show_plan_time_;

    /* processing time */
    //double time_search_ = 0.0;
    //double time_optimize_ = 0.0;
    //double time_adjust_ = 0.0;
};

struct LocalTrajData
{
    /* info of generated traj */

    int traj_id_;
    double duration_;
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
    ros::Time start_time_;
    Eigen::Vector2d start_pos_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
};


#endif