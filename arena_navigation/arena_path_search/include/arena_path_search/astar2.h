
#ifndef _ASTAR_H
#define _ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>

#include <map>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>
#include <memory>

#include "arena_mapping/mapping.h"

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

/* define Node */
class Node {
public:
  /* -------------------- */
  Eigen::Vector2i index;
  Eigen::Vector2d position;
  double g_score, f_score;
  Node* parent;
  char node_state;

  double time;  // dyn
  int time_idx;

  /* -------------------- */
  Node() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~Node(){};
};
typedef Node* NodePtr;

/* Node comparator */
class NodeComparator0 {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

/* define Hashtable */
template <typename T>
struct matrix_hash0 : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (size_t)matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable0 {
private:
  /* data */
  std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash0<Eigen::Vector2i>> data_2d_;
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;

public:
  NodeHashTable0(/* args */) {
  }
  ~NodeHashTable0() {
  }
  void insert(Eigen::Vector2i idx, NodePtr node) {
    data_2d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector2i idx, int time_idx, NodePtr node) {
    data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), time_idx), node));
  }

  NodePtr find(Eigen::Vector2i idx) {
    auto iter = data_2d_.find(idx);
    return iter == data_2d_.end() ? NULL : iter->second;
  }
  NodePtr find(Eigen::Vector2i idx, int time_idx) {
    auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_2d_.clear();
    data_3d_.clear();
  }
};


class Astar{
private:
    /* ---------- main data structure ---------- */
    std::vector<NodePtr> path_node_pool_;
    int use_node_num_, iter_num_;
    NodeHashTable0 expanded_nodes_;
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
    std::vector<NodePtr> path_nodes_;
public:

    /* ---------- record data ---------- */
    GridMap::Ptr grid_map_;
    bool has_path_ = false;

    /* ---------- parameter ---------- */
    /* search */
    double lambda_heu_;
    int allocate_num_;
    double tie_breaker_;

    /* map */
    double resolution_, inv_resolution_;
    Eigen::Vector2d origin_, map_size_2d_;


    /* helper */
    Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
    void retrievePath(NodePtr end_node);

    /* heuristic function */
    double getDiagHeu(Eigen::Vector2d x1, Eigen::Vector2d x2);
    double getManhHeu(Eigen::Vector2d x1, Eigen::Vector2d x2);
    double getEuclHeu(Eigen::Vector2d x1, Eigen::Vector2d x2);

public:
    Astar(){};
    ~Astar();

    enum { REACH_END = 1, NO_PATH = 2 };

    /* main API */
    void setParam(ros::NodeHandle& private_nh);

    void setEnvironment(const GridMap::Ptr& env);

    void init();

    void reset();

    int search(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);

    std::vector<Eigen::Vector2d> getPath();

    std::vector<NodePtr> getVisitedNodes();

    typedef std::shared_ptr<Astar> Ptr;
};






























#endif