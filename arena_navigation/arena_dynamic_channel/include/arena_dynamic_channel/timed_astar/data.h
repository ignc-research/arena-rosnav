#pragma once
#include <Eigen/Eigen>
#include <memory>
#include <map>
#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "arena_dynamic_channel/graph/accessor.h"
#include "arena_dynamic_channel/graph/delaunator.h"


namespace timed_astar
{

/* delaunator types */
typedef delaunator::rank Rank;
typedef delaunator::index Index;
typedef delaunator::Delaunator Graph;
typedef std::shared_ptr<Graph> GraphPtr;


/* header of the pedestrian data */
constexpr size_t INIT_INDEX = 0;
constexpr size_t GOAL_INDEX = 1;
constexpr size_t PHASE1_INDEX = 2;
constexpr size_t PHASE2_INDEX = 3;
constexpr size_t PHASE3_INDEX = 4;
constexpr size_t PHASE4_INDEX = 5;
constexpr size_t PEDS_START = 6;
#define PI 3.14159265

/* hyperpara loaded from launch file */
struct TimedAstarParam{
    // Node allocate num
    int ALLOCATE_NUM;
    
    // REACH
    double GOAL_RADIUS;

    // distance
    double SAFE_DIST;
    double ROBOT_RADIUS;
    double OBSTACLE_RADIUS;

    // action sample
    double MAX_SPEED;   
    double AVG_SPEED;  
    double MIN_SPEED;

    double MAX_ROT_SPEED; // rad/sec

    
    // sample on edge
    int NUM_SAMPLE_EDGE;

    // resolution
    double RESOLUTION;
    double TIME_RESOLUTION;
    
    // time 
    double TIME_HORIZON;
    size_t TIME_SLICE_NUM;

    // sensor range
    double SENSOR_RANGE; 

    double SAFE_TIME;
};

/* build in hyperpara */
const Index NONE_INDEX = std::numeric_limits<Index>::max();



inline int min(int a, int b){
    return a<b? a:b;
}
inline size_t min(size_t a, size_t b){
    return a<b? a:b;
}


struct Vec2i
{
    Index x, y;
    Vec2i(): x(0), y(0) {}
    Vec2i(const Index xi, const Index yi): x(xi), y(yi) {}
    Vec2i(const Vec2i& vec): x(vec.x), y(vec.y) {}
    friend bool operator == (const Vec2i &a, const Vec2i &b);
};


struct Vec2d
{
    double x, y;
    Vec2d():x(0.0), y(0.0){}
    Vec2d(double xp, double yp): x(xp), y(yp){}
    Vec2d(const Vec2d& vec): x(vec.x), y(vec.y) {}

    friend Vec2d operator + (const Vec2d &a, const Vec2d &b);
    friend Vec2d operator - (const Vec2d &a, const Vec2d &b);
    friend Vec2d operator * (const Vec2d &a, const Vec2d &b);
    friend Vec2d operator * (const Vec2d &a, const double scale);
    friend Vec2d operator / (const Vec2d &a, const double scale);    
    friend bool operator == (const Vec2d &a, const Vec2d &b);
    friend bool operator < (const Vec2d &a, const Vec2d &b);
    friend bool operator > (const Vec2d &a, const Vec2d &b);
    friend bool operator <= (const Vec2d &a, const Vec2d &b);
    friend bool operator >= (const Vec2d &a, const Vec2d &b);
    friend double dot (const Vec2d &a, const Vec2d &b);
    friend double cross (const Vec2d &a, const Vec2d &b);

    double sum() const {return x+y;}

    double angle() const { 
       double angle_rad=atan2(y,x);
       return angle_rad<0? 2*PI+angle_rad : angle_rad;
    }

    double length() const { return sqrt(x*x + y*y);}
};


struct Edge
{
    Index a_eid, b_eid;  // the eid of two endpoints in grpah.triangles
    Vec2d a_position, b_position;  // the position of the two points
    Vec2d a_velocity, b_velocity;  // the velocity of the two points
    Vec2d a_radius, b_radius;  // used for funnel construction
    Edge(): a_eid(NONE_INDEX), b_eid(NONE_INDEX){}
    Edge(Index a, Index b): a_eid(a), b_eid(b){}
};

struct Node
{
    Index eid;  // the halfedge index = next
    Index tid;  // the triangle index = floor(eid / 3.0)
    Index sid;  // the slice index of timed graph

    Vec2d position;  // node placement position
    Vec2d velocity;  // node placement velocity

    double G, H;  // accumulated cost-to-go and heuristic
    double time_elapsed; // accumulated time
    std::shared_ptr<Node> parent;  // the parent node pointer
    std::shared_ptr<Edge> shared;  // the shared edge with parent


    Node(Index id):
        eid(id), sid(0), G(DBL_MAX), H(0.0), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        this->tid = static_cast<Index>(floor(eid / 3.0));
    }

    Node(Index id, Index t):
        eid(id), sid(t), G(DBL_MAX), H(0.0), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        this->tid = static_cast<Index>(floor(eid / 3.0));
    }

    Node(Index id, double g, double h):
        eid(id), sid(0), G(g), H(h), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        tid = static_cast<Index>(floor(eid / 3.0));
    }

    Node(Index id, Index t, double g, double h):
        eid(id), sid(t), G(g), H(h), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        tid = static_cast<Index>(floor(eid / 3.0));
    }

    void fetch_states(const Graph* graph){
        auto i = 2 * graph->triangles[eid];
        auto px = graph->coords[i];
        auto py = graph->coords[i+1];
        auto vx = graph->speeds[i];
        auto vy = graph->speeds[i+1];
        position.x = px;
        position.y = py;
        velocity.x = vx;
        velocity.y = vy;
    }

}; 

struct PathNode
{   
    enum state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};
    
	enum state node_state{UNDEFINED};

    //  current
    Index eid;              // the halfedge index = next
    Index tid;              // the triangle index = floor(eid / 3.0)
    Index sid;              // the slice index of timed graph

    double time_elapsed;    // accumulated time

    Vec2d pos;                // current position,  node placement position
    //Vec2i pos_index;        // node placement position
    Vec2d vel;                // current velocity

    double dir;             // [rad] current direction of the car

    //  input
    double v_in;
    double w_in;

    //  duration
    double dur_v;
    double dur_w;

    //  cost
    double G, H;             // accumulated cost-to-go and heuristic

    // collide check
    double time_to_collide;
    double dist_to_collide;
    bool is_unsafe=false;

    // parent node
    std::shared_ptr<PathNode> parent;   // the parent node pointer
    
    PathNode(){
        G=DBL_MAX;
        H= 0.0;
        parent=nullptr;
        node_state=UNDEFINED;
    };

    void setTimedTriangle(const Vec2d &pos, const double &time, const std::vector<GraphPtr> &timed_graph, double time_resolution, size_t slice_num ){
        namespace dl = delaunator;
        this->pos=pos;
        this->time_elapsed=time;
        this->sid = std::min((size_t)(floor((time_elapsed - 0.0) / time_resolution)),slice_num-1);
        
        this->eid=dl::locateCurrentFace(timed_graph[this->sid].get(), pos.x, pos.y);
        
        this->tid = static_cast<Index>(floor(eid / 3.0));
    }

};

template <typename T>
struct matrix_hash : std::unary_function<Eigen::Vector3i, size_t> {
  std::size_t operator()(Eigen::Vector3i const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (size_t)matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename Eigen::Vector3i::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

typedef std::shared_ptr<Edge> EdgePtr;
typedef std::shared_ptr<Node> NodePtr;
typedef std::shared_ptr<PathNode> PathNodePtr;


class NodeComparator {
 public:
  bool operator()(const PathNodePtr& a, const PathNodePtr& b) {
    return (a->G + a->H) > (b->G + b->H);
  }
};


class PathNodeHashTable {
private:
  /* data: Eigen::Vector2i index, int time_index */ 
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;

public:
  PathNodeHashTable(/* args */) {
  }
  ~PathNodeHashTable() {
  }

  void insert(Vec2i pos_idx, int time_idx, PathNodePtr node) {
    data_3d_.insert(std::make_pair(Eigen::Vector3i(pos_idx.x, pos_idx.y, time_idx), node));
  }

  PathNodePtr find(Vec2i pos_idx , int time_idx) {
    auto iter = data_3d_.find(Eigen::Vector3i(pos_idx.x, pos_idx.y, time_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
  }
};

}

