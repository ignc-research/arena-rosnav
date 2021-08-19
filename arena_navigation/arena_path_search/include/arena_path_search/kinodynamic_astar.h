#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>


#include <boost/functional/hash.hpp>
#include <map>
#include <queue>
#include <unordered_map>
#include <utility>

#include "arena_mapping/mapping.h"

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

/* Define Node */
class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector2i index;
  Eigen::Matrix<double, 4, 1> state;  // pos, vel
  double g_score, f_score;
  Eigen::Vector2d input;
  double duration;
  double time;  // dyn
  int time_idx;
  PathNode* parent;
  char node_state;

  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};


template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (size_t)matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};


class NodeHashTable {
 private:
  /* data */
  std::unordered_map<Eigen::Vector2i, PathNodePtr, matrix_hash<Eigen::Vector2i>>
      data_2d_;
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;

 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insert(Eigen::Vector2i idx, PathNodePtr node) {
    data_2d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector2i idx, int time_idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(
        Eigen::Vector3i(idx(0), idx(1), time_idx), node));
  }

  PathNodePtr find(Eigen::Vector2i idx) {
    auto iter = data_2d_.find(idx);
    return iter == data_2d_.end() ? NULL : iter->second;
  }
  PathNodePtr find(Eigen::Vector2i idx, int time_idx) {
    auto iter =
        data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_2d_.clear();
    data_3d_.clear();
  }
};


class KinodynamicAstar {
  private:
  /* ---------- main data structure ---------- */
  std::vector<PathNodePtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTable expanded_nodes_;

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;

  std::vector<PathNodePtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector2d start_vel_, end_vel_, start_acc_;

  Eigen::Matrix<double, 4, 4> phi_;  // state transit matrix 

  // shared_ptr<GridMap> sdf_map;
  GridMap::Ptr grid_map_;//edt_environment_;
  

  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_, init_max_tau_;     // delta_t 
  double max_vel_, max_acc_;
  double w_time_, horizon_, lambda_heu_;
  int allocate_num_, check_num_;
  double tie_breaker_;
  bool optimistic_;
  double goal_tolerance_;

  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  Eigen::Vector2d origin_, map_size_2d_;
  double time_origin_;
  

  /* helper */
  Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
  int timeToIndex(double time);
  void retrievePath(PathNodePtr end_node);

  /* shot trajectory */
  std::vector<double> cubic(double a, double b, double c, double d);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal);

  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,double& optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 4, 1>& state0,
                    Eigen::Matrix<double, 4, 1>& state1, Eigen::Vector2d um,
                    double tau);

public:
  KinodynamicAstar(){};
  ~KinodynamicAstar();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };

  /* main API */
  void setParam(ros::NodeHandle& private_nh);
  void setEnvironment(const GridMap::Ptr& env);
  void init();
  void init(ros::NodeHandle& private_nh, const GridMap::Ptr& env);

  void reset();
  int search(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel,
             Eigen::Vector2d start_acc, Eigen::Vector2d end_pt,
             Eigen::Vector2d end_vel, bool init, bool dynamic = false,
             double time_start = -1.0);

  

  std::vector<Eigen::Vector2d> getKinoTraj(double delta_t);

  void getSamples(double& ts, std::vector<Eigen::Vector2d>& point_set,
                  std::vector<Eigen::Vector2d>& start_end_derivatives);

  std::vector<PathNodePtr> getVisitedNodes();

  typedef std::shared_ptr<KinodynamicAstar> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW



};
























#endif