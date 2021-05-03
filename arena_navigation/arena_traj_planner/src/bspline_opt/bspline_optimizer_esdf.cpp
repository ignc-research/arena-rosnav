#include <arena_traj_planner/bspline_opt/bspline_optimizer_esdf.h>
#include <nlopt.hpp>


const int BsplineOptimizerESDF::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizerESDF::DISTANCE    = (1 << 1);
const int BsplineOptimizerESDF::FEASIBILITY = (1 << 2);
const int BsplineOptimizerESDF::ENDPOINT    = (1 << 3);
const int BsplineOptimizerESDF::GUIDE       = (1 << 4);
const int BsplineOptimizerESDF::WAYPOINTS   = (1 << 6);

const int BsplineOptimizerESDF::GUIDE_PHASE     = BsplineOptimizerESDF::SMOOTHNESS | BsplineOptimizerESDF::GUIDE;
const int BsplineOptimizerESDF::NORMAL_PHASE    = BsplineOptimizerESDF::SMOOTHNESS | BsplineOptimizerESDF::DISTANCE | BsplineOptimizerESDF::FEASIBILITY;

/* main API */
void BsplineOptimizerESDF::setParam(ros::NodeHandle& nh) {
    nh.param("optimization_ESDF/lambda1", lambda1_, 10.0);
    nh.param("optimization_ESDF/lambda2", lambda2_, 5.0);
    nh.param("optimization_ESDF/lambda3", lambda3_, 0.00001);
    nh.param("optimization_ESDF/lambda4", lambda4_, 0.01);
    nh.param("optimization_ESDF/lambda5", lambda5_, -1.0);
    //nh.param("optimization_ESDF/lambda6", lambda6_, -1.0);
    nh.param("optimization_ESDF/lambda7", lambda7_, 100.0);
    

    nh.param("optimization_ESDF/dist0", dist0_, 0.4);
    nh.param("optimization_ESDF/max_vel", max_vel_, 3.0);
    nh.param("optimization_ESDF/max_acc", max_acc_, 3.0);
    nh.param("optimization_ESDF/visib_min", visib_min_, -1.0);
    nh.param("optimization_ESDF/dlmin", dlmin_, -1.0);
    nh.param("optimization_ESDF/wnl", wnl_, -1.0);

    nh.param("optimization_ESDF/max_iteration_num1", max_iteration_num_[0], 2);
    nh.param("optimization_ESDF/max_iteration_num2", max_iteration_num_[1], 300);
    nh.param("optimization_ESDF/max_iteration_num3", max_iteration_num_[2], 200);
    nh.param("optimization_ESDF/max_iteration_num4", max_iteration_num_[3], 200);
    nh.param("optimization_ESDF/max_iteration_time1", max_iteration_time_[0], 0.0001);
    nh.param("optimization_ESDF/max_iteration_time2", max_iteration_time_[1], 0.005);
    nh.param("optimization_ESDF/max_iteration_time3", max_iteration_time_[2], 0.003);
    nh.param("optimization_ESDF/max_iteration_time4", max_iteration_time_[3], 0.003);

    nh.param("optimization_ESDF/algorithm1", algorithm1_, 15);
    nh.param("optimization_ESDF/algorithm2", algorithm2_, 11);
    nh.param("optimization_ESDF/order", order_, 3);
}

void BsplineOptimizerESDF::setEnvironment(const GridMap::Ptr& env) {
    this->grid_map_ = env;
}

Eigen::MatrixXd BsplineOptimizerESDF::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

/* required inputs */
void BsplineOptimizerESDF::setControlPoints(const Eigen::MatrixXd& points) {
    control_points_ = points;
    dim_            = control_points_.rows();
}

void BsplineOptimizerESDF::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizerESDF::setTerminateCond(const int& max_num_id, const int& max_time_id) {
    max_num_id_  = max_num_id;
    max_time_id_ = max_time_id;
}

void BsplineOptimizerESDF::setCostFunction(const int& cost_code) {
    cost_function_ = cost_code;

    // print optimized cost function
    std::string cost_str;
    if (cost_function_ & SMOOTHNESS)    cost_str += "smooth |";
    if (cost_function_ & DISTANCE)      cost_str += " dist  |";
    if (cost_function_ & FEASIBILITY)   cost_str += " feasi |";
    if (cost_function_ & ENDPOINT)      cost_str += " endpt |";
    if (cost_function_ & GUIDE)         cost_str += " guide |";
    if (cost_function_ & WAYPOINTS)     cost_str += " waypt |";

    ROS_INFO_STREAM("cost func: " << cost_str);
}

/* optional inputs */
void BsplineOptimizerESDF::setGuidePath(const std::vector<Eigen::Vector2d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizerESDF::setWaypoints(const std::vector<Eigen::Vector2d>& waypts,
                                    const std::vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}


/* optimize */
void BsplineOptimizerESDF::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.cols();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.col(pt_num - 3) + 4 * control_points_.col(pt_num - 2) +
         control_points_.col(pt_num - 1));
  } else {
    variable_num_ = std::max(0, dim_ * (pt_num - 2 * order_)) ;
  }

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizerESDF::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  std::vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(j, i);
    }
  }

  if (dim_ != 1) {
    std::vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    // cout << fixed << setprecision(7);
    // vec_time_.clear();
    // vec_cost_.clear();
    // time_start_ = ros::Time::now();

    double        final_cost;
    nlopt::result result = opt.optimize(q, final_cost);

    /* retrieve the optimization result */
    // cout << "Min cost:" << min_cost_ << endl;
  } catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    std::cout << e.what() << std::endl;
  }

  for (int i = order_; i < control_points_.cols(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(j, i) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) ROS_INFO_STREAM("iter num: " << iter_num_);
}


/* cost function */
  /* calculate each part of cost function with control points q as input */


void BsplineOptimizerESDF::calcSmoothnessCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                                          std::vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector2d jerk, temp_j;

  for (unsigned int i = 0; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizerESDF::calcDistanceCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                                        std::vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector2d dist_grad, g_zero(0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    grid_map_->evaluateEDTWithGrad(q[i], dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

void BsplineOptimizerESDF::calcFeasibilityCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                                           std::vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (unsigned int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector2d vi = q[i + 1] - q[i];

    for (int j = 0; j < 2; j++) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (unsigned int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector2d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 2; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizerESDF::calcEndpointCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                                        std::vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector2d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizerESDF::calcWaypointsCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                                         std::vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector2d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (unsigned int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector2d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

void BsplineOptimizerESDF::calcGuideCost(const std::vector<Eigen::Vector2d>& q, double& cost,
                                     std::vector<Eigen::Vector2d>& gradient) {
  cost = 0.0;
  Eigen::Vector2d zero(0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector2d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizerESDF::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(j, i);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(j,control_points_.cols() - order_ + i);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  /*  evaluate costs and their gradient  */
  double f_smoothness, f_distance, f_feasibility, f_endpoint, f_guide, f_waypoints;
  f_smoothness = f_distance = f_feasibility = f_endpoint = f_guide = f_waypoints = 0.0;

  if (cost_function_ & SMOOTHNESS) {
    calcSmoothnessCost(g_q_, f_smoothness, g_smoothness_);
    f_combine += lambda1_ * f_smoothness;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda1_ * g_smoothness_[i + order_](j);
  }
  if (cost_function_ & DISTANCE) {
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += lambda2_ * f_distance;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda2_ * g_distance_[i + order_](j);
  }
  if (cost_function_ & FEASIBILITY) {
    calcFeasibilityCost(g_q_, f_feasibility, g_feasibility_);
    f_combine += lambda3_ * f_feasibility;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda3_ * g_feasibility_[i + order_](j);
  }
  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, f_endpoint, g_endpoint_);
    f_combine += lambda4_ * f_endpoint;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda4_ * g_endpoint_[i + order_](j);
  }
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += lambda5_ * f_guide;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda5_ * g_guide_[i + order_](j);
  }
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += lambda7_ * f_waypoints;
    for (int i = 0; i < variable_num_ / dim_; i++)
      for (int j = 0; j < dim_; j++) grad[dim_ * i + j] += lambda7_ * g_waypoints_[i + order_](j);
  }
  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << lambda8_ * f_view
  //        << ", waypt: " << lambda7_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << lambda1_ * f_smoothness
  //  << " , dist:" << lambda2_ * f_distance
  //  << ", fea: " << lambda3_ * f_feasibility << endl;
  // << ", end: " << lambda4_ * f_endpoint
  // << ", guide: " << lambda5_ * f_guide
  // }
}

double BsplineOptimizerESDF::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizerESDF* opt = reinterpret_cast<BsplineOptimizerESDF*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

std::vector<Eigen::Vector2d> BsplineOptimizerESDF::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  std::vector<Eigen::Vector2d> ctrl_q;
  for (int i = 0; i < ctrl_pts.cols(); ++i) {
    ctrl_q.push_back(ctrl_pts.col(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizerESDF::getControlPoints() { return this->control_points_; }

bool BsplineOptimizerESDF::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}














