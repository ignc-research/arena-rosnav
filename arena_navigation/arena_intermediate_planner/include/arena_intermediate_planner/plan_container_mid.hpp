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
  double dist_next_wp_;
  double dist_tolerance_;
  double rou_thresh_;
  int id_last_landmark_;

  void resetLandmarks(){
    /* init */
    global_path_.clear();
    landmark_points_.clear();
    double rou_t, rou_last=-1;
    double gradient_t,gradient_last=1000;
    Eigen::Vector2d last_landmark=start_pos_;
    int id=0;
    bool flag_landmark_search_done=false;

    int continue_descent_counter=0;
    
    /* search landmark pt according to curvature radius */
    for (double t = tm_start_; t <= tm_end_; t += 0.01) {
      // get pos
      Eigen::Vector2d pos_t = getPosition(t);
      global_path_.push_back(pos_t);
      if(flag_landmark_search_done){
        continue;
      }
      // calculate curvature radius
      rou_t=calculateCurveRadius(getVelocity(t),getAcceleration(t));
      //std::cout<<"********************* rou="<<rou_t<<std::endl;
      // calculate gradient
      gradient_t=rou_t-rou_last;
      
      // set continue_descent_counter
      if(gradient_t<-5.0){
        continue_descent_counter++;
      }else{
        continue_descent_counter=0;
      }

      if(std::abs(gradient_t)<0.2){
        gradient_t=gradient_last;
        rou_t=rou_last;
      }

      /* minimum vaule of the curvature radius || curvature radius is so small */
      if((gradient_t>0 && gradient_last<0)|| rou_t<rou_thresh_|| continue_descent_counter>80){ //  //gradient_t*gradient_last<0
        //std::cout<<"in1************ rou="<<rou_t<<std::endl;
        // negelect the start points 
        if((pos_t-start_pos_).squaredNorm()<=dist_next_wp_ && rou_t>rou_thresh_){
          rou_last=rou_t;
          gradient_last=gradient_t;
          continue;
        }
        // reset continue_descent_counter
        
        if(continue_descent_counter>80){
          //std::cout<<"in counter************ rou="<<rou_t<<std::endl;
        }

        //std::cout<<"in2************ rou="<<rou_t<<std::endl;
        // make sure two landmarks are not too close
        if((pos_t-last_landmark).squaredNorm()>2.0)
        { 
          
          landmark_points_.push_back(LandmarkPoint(pos_t,id));
          last_landmark=pos_t;
          id++;
          continue_descent_counter=0;
          //std::cout<<"add************ rou="<<rou_t<<std::endl;
        }
        
      }else if((pos_t-end_pos_).squaredNorm()<=dist_next_wp_){
        // add end point as last landmark
        landmark_points_.push_back(LandmarkPoint(end_pos_,id));
        flag_landmark_search_done=true;
      }
      rou_last=rou_t;
      gradient_last=gradient_t;
    }

    // add end_pos to global path
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
    double rou=std::pow(vel.squaredNorm(),3)/(vel(0)*acc(1)-vel(1)*acc(0));
    return std::abs(rou);
  }

public:
  UniformBspline global_pos_traj_, global_vel_traj_, global_acc_traj_;
  double tm_start_, tm_end_;
  Eigen::Vector2d start_pos_, end_pos_;

  std::vector<Eigen::Vector2d> global_path_;
  std::vector<LandmarkPoint> landmark_points_;
  
  GlobalData(){};
  ~GlobalData(){};
  
  void setGlobalDataParam(double dist_next_wp, double dist_tolerance, double rou_thresh){
    dist_next_wp_=dist_next_wp;
    dist_tolerance_=dist_tolerance;
    rou_thresh_=rou_thresh;
  }

  void resetGlobalData(const UniformBspline & traj){
    global_pos_traj_= traj;
    global_vel_traj_=global_pos_traj_.getDerivative();
    global_acc_traj_=global_vel_traj_.getDerivative();
    global_pos_traj_.getTimeSpan(tm_start_, tm_end_);
    start_pos_=getPosition(tm_start_);
    end_pos_=getPosition(tm_end_);
    resetLandmarks();
  }

  void getGlobalPath(std::vector<Eigen::Vector2d> &global_path){
     global_path=global_path_;
  }

  void getLandmarks(std::vector<Eigen::Vector2d> & landmark_pts){

     for(int i=0;i<landmark_points_.size();i++){
       landmark_pts.push_back(landmark_points_[i].pos);
     }
  }

  Eigen::Vector2d getLocalTarget(Eigen::Vector2d &current_pt){
    Eigen::Vector2d target_pt;
    Eigen::Vector2d landmark_pt;
    
    double dist_to_last_landmark;
    dist_to_last_landmark=(landmark_points_[id_last_landmark_].pos-current_pt).squaredNorm();

    // check if landmark point is arrived, and update target landmark id
    if(dist_to_last_landmark<dist_tolerance_){
      id_last_landmark_++;

      if(id_last_landmark_>=landmark_points_.size()){
        id_last_landmark_=landmark_points_.size()-1;
      }
    }

    // set landmark pos
    landmark_pt=landmark_points_[id_last_landmark_].pos;

    // calculate the target_pt at the direction of landmark pos
    double a;
    a=dist_next_wp_/(current_pt-landmark_pt).squaredNorm();
    target_pt=current_pt+a*(landmark_pt-current_pt);

    return target_pt;
  }

  
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
    double planning_horizen_;

    /* global data param */
    double dist_next_wp_;           // distance from current location to next waypoint
    double dist_tolerance_;         // distance tolerance for arriving
    double rou_thresh_;             // thresh hold of curature radius for determin landmark points

    /* flags */
    bool use_astar_, use_kino_astar_, use_oneshot_;
    bool use_optimization_esdf_, use_optimization_astar_;

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