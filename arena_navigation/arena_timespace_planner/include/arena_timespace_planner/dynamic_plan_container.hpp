#ifndef _DYNAMIC_PLAN_CONTAINER_H_
#define _DYNAMIC_PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <vector>
#include <arena_traj_planner/bspline/uniform_bspline.h>
#include <arena_traj_planner/polynomial/polynomial_traj.h>

#define PI 3.1415926

using std::vector;

struct LandmarkPoint{
  int id;
  Eigen::Vector2d pos;

  LandmarkPoint(Eigen::Vector2d pos,int id){
    this->pos=pos;
    this->id=id;
  }

  ~LandmarkPoint(){};
};

class GlobalData{
private:
    double dist_thresh_ =2.0;              // [meter] 
    double angle_thresh_=20*PI/180;        // [rad]
    double min_angle_vel_=5*PI/180;        // [rad]
    double max_angle_vel_=360*PI/180;      // [rad]
    
    double dist_wp_ = 3.0;                 // [meter] // for simple wp sampler

    size_t visited_landmark_cnt_;
    double subgoal_tolerance_;

    Eigen::Vector2d getPosition(double t){
      return pos_traj_.evaluateDeBoor(t);
    }

    Eigen::Vector2d getVelocity(double t){
      return vel_traj_.evaluateDeBoor(t);
    }

    Eigen::Vector2d getAcceleration(double t){
      return acc_traj_.evaluateDeBoor(t);
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

    void resetLandmarks()
    {
        global_path_.clear();
        wp_samples_.clear(); 
        landmark_points_.clear();

        double angle_sum=0;
        double angle_delta;
        double angle_vel, angle_vel_last=0.0;
    
        double delta_t=0.01;

        // init last_landmark pos
        Eigen::Vector2d last_landmark=start_pos_;
        Eigen::Vector2d last_wp=start_pos_;
        
        // init id
        int id=0;
        int iter=0;
        //int iter_sum=round((tm_end_-tm_start_)/delta_t);

        for (double t = tm_start_+delta_t; t < tm_end_; t += delta_t) 
        {
            iter++;
            // add to global path
            Eigen::Vector2d pos_t = getPosition(t);
            global_path_.push_back(pos_t);
            //for simple wp sampler
            if((pos_t-last_wp).norm()>dist_wp_){
                wp_samples_.push_back(pos_t);
                last_wp=pos_t;
            }    

            // get angular vel
            angle_vel=calculateRotationVelocity(getVelocity(t),getAcceleration(t));
            //printf("\033[32miter(+1)=%d,sum=%d: angle vel=%5.3f, angle_sum=%5.3f\n\033[0m", iter, iter_sum,angle_vel*180/3.14,angle_sum*180/3.14);

            // pre-process angular vel( for condition1: to delay add landmark under condition1)  
            if(std::abs(angle_vel)<min_angle_vel_){
                angle_vel=0;
            }else if(std::abs(angle_vel)>max_angle_vel_){
                angle_vel=angle_vel>0?max_angle_vel_:-max_angle_vel_;
            }

            // check if near goal
            if((pos_t-end_pos_).norm()<dist_thresh_){
                // near end
                //std::cout<<"near="<<std::endl;
                continue;
            }

            /* condition1 : angular velocity direction change condition(calulation using modified angular_vel)*/
            if(angle_vel_last*angle_vel<0){
                //std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^id="<<id<<std::endl;
                //std::cout<<"condition 1: change direction="<<angle_sum*180/3.14<<std::endl;
                //angle_sum=0.0; //(done by condition2 now)

                if((pos_t-last_landmark).norm()>dist_thresh_)
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
            if(std::abs(angle_sum)>angle_thresh_){
                //std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^id="<<id<<std::endl;
                //std::cout<<"condition 2: angle_sum="<<angle_sum*180/3.14<<std::endl;
                
                if((pos_t-last_landmark).norm()>dist_thresh_)
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
        wp_samples_.push_back(end_pos_);
        // reset landmark counter
        visited_landmark_cnt_=0;
    }

    void refineLandmarks(){
        // raycast function
    }

public:
    UniformBspline pos_traj_, vel_traj_, acc_traj_;
    double tm_start_, tm_end_;
    Eigen::Vector2d start_pos_, end_pos_;

    std::vector<Eigen::Vector2d> global_path_;
    std::vector<LandmarkPoint> landmark_points_;

    std::vector<Eigen::Vector2d> wp_samples_;
    

    GlobalData(){};
    ~GlobalData(){};

    void resetData(const UniformBspline & pos_traj)
    {
        pos_traj_= pos_traj;
        vel_traj_=pos_traj_.getDerivative();
        acc_traj_=vel_traj_.getDerivative();
        // get timespan
        pos_traj_.getTimeSpan(tm_start_, tm_end_);
        // get start & end points
        start_pos_  = getPosition(tm_start_);
        end_pos_    = getPosition(tm_end_);
        // reset landmarks
        resetLandmarks();
    }

    std::vector<Eigen::Vector2d> getGlobalPath()
    {
        return global_path_;   
    }

    std::vector<Eigen::Vector2d> getControlPoints(){
        Eigen::MatrixXd control_pts_matrix=pos_traj_.get_control_points();// (dim,cols)
        std::vector<Eigen::Vector2d> ctrl_pts;
        ctrl_pts.clear();
        for(int i=0;i<control_pts_matrix.cols();i++){
        ctrl_pts.push_back(control_pts_matrix.col(i));
        }
        return ctrl_pts;
    }

    std::vector<Eigen::Vector2d> getLandmarks(){
        std::vector<Eigen::Vector2d> landmark_pts;
        if(!landmark_points_.empty())
        {
            for(unsigned int i=0;i<landmark_points_.size();i++)
            {
                landmark_pts.push_back(landmark_points_[i].pos);
            }
        }
        return landmark_pts;
    }

    std::vector<Eigen::Vector2d> getSamples(){
        return wp_samples_;
    }

};

class MidData{
public:
    bool is_empty{true};
    double duration_;
    ros::Time start_time_;
    Eigen::Vector2d start_pos_;
    UniformBspline pos_traj_, vel_traj_, acc_traj_, jerk_traj_;
    std::vector<Eigen::Vector2d> local_path_;

    MidData(){
        is_empty=true;
    }
    void resetData(const UniformBspline & pos_traj)
    {   
        is_empty=false;
        
        pos_traj_= pos_traj;
        start_time_ = ros::Time::now();
        vel_traj_ = pos_traj_.getDerivative();
        acc_traj_ = vel_traj_.getDerivative();
        jerk_traj_= acc_traj_.getDerivative();
        start_pos_ = pos_traj_.evaluateDeBoorT(0.0);
        duration_ = pos_traj_.getTimeSum();

        local_path_.clear();
        double tm_start,tm_end;
        pos_traj_.getTimeSpan(tm_start, tm_end);
        for(double t = tm_start; t < tm_end; t += 0.1){
            local_path_.push_back(pos_traj_.evaluateDeBoor(t));            
        }
    }

    std::vector<Eigen::Vector2d> getLocalPath()
    {   
        return local_path_;
    }

    
};


class TargetTrajData{
public:
    bool flag_is_empty{true};
    double duration_;
    ros::Time start_time_;
    Eigen::Vector2d start_pos_;
    UniformBspline pos_traj_, vel_traj_, acc_traj_, jerk_traj_;
    std::vector<Eigen::Vector2d> local_path_;
    
    

    TargetTrajData(){flag_is_empty=true;}
    ~TargetTrajData(){}

    void resetData(const UniformBspline & pos_traj)
    {   
        flag_is_empty=false;
        
        pos_traj_= pos_traj;
        start_time_ = ros::Time::now();
        vel_traj_ = pos_traj_.getDerivative();
        acc_traj_ = vel_traj_.getDerivative();
        jerk_traj_= acc_traj_.getDerivative();
        start_pos_ = pos_traj_.evaluateDeBoorT(0.0);
        duration_ = pos_traj_.getTimeSum();

        local_path_.clear();
        
        double tm_start,tm_end;
        pos_traj_.getTimeSpan(tm_start, tm_end);
        for(double t = tm_start; t < tm_end; t += 0.1){
            Eigen::Vector2d pt=pos_traj_.evaluateDeBoor(t);
            local_path_.push_back(pt); 
        }

    }

    std::vector<Eigen::Vector2d> getLocalPath()
    {   
        return local_path_;
    }

    void getControlInput(double t_curr, double &v, double &w){
        //Eigen::Vector2d pos = pos_traj_.evaluateDeBoorT(t_curr);
        Eigen::Vector2d vel = vel_traj_.evaluateDeBoorT(t_curr);
        Eigen::Vector2d acc = acc_traj_.evaluateDeBoorT(t_curr);
        //Eigen::Vector2d jerk= jerk_traj_.evaluateDeBoorT(t_curr);
        v=vel.norm();
        if(std::abs(v)<2.0){
            v=v;
        }else{
            v=2.0;
        }
        
        if(v!=0.0){
            w=(vel(0)*acc(1) - vel(1)*acc(0))/vel.squaredNorm();
            //w=std::max(w,180*3.14/180);
        }

        if(std::abs(w)<60*3.14/180){
            w = w;
        }else{
            if(w<0){
                w=-60*3.14/180;
            }else{
                w= 60*3.14/180;
            }
        }
        

    }

    // Eigen::Vector2d getPositionAtTime(double t_curr){
    //     return pos_traj_.evaluateDeBoorT(t_curr);
    // }

};



struct LocalTrajData
{
    /* info of generated traj */
    double duration_;
    ros::Time start_time_;
    Eigen::Vector2d start_pos_;
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
};

struct PlanParameters{
    /* planning algorithm parameters */
    
    double max_vel_, max_acc_, max_jerk_;   // physical limits
    double ctrl_pt_dist_;                  // distance between adjacient B-spline control points
    //double time_resolution_;                // for select smaple from timed_astar path
    
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits
    
    
    bool use_distinctive_trajs_;

    double local_time_horizon_;

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
};











#endif