

#ifndef _ASTAR_SAMPLE_H_
#define _ASTAR_SAMPLE_H_


#include <vector>
#include <functional>
#include <set>
#include <cmath>
#include <queue>
#include <iostream>
#include <memory>
#include <algorithm>

#include <Eigen/Eigen>


#include "arena_dynamic_channel/graph/accessor.h"
#include "arena_dynamic_channel/graph/delaunator.h"
#include "arena_dynamic_channel/timed_astar/data.h"

#include <arena_mapping/mapping.h>

namespace dl = delaunator;

namespace timed_astar
{


bool solveRoot(double a0, double a1, double a2, double &x1, double &x2);

double getDistPointToLine(const Vec2d &a, const Vec2d &b1, const Vec2d &b2);

std::vector<double> getLineParam(const Vec2d & p1, const Vec2d &p2);

bool isPointOnSegment(const Vec2d & p1, const Vec2d &p2, const Vec2d & inter_pt);

bool getTwoLineIntersection(const Vec2d & p1, const Vec2d &p2,const Vec2d &q1, const Vec2d &q2, Vec2d & inter_pt);

class TimedAstar{

private:
    // time search graph
    std::vector<GraphPtr> timed_graph_;

    // init and goal condition
    Vec2d start_pos_;
    Vec2d goal_pos_;
    
    // search param
    TimedAstarParam param_;
    std::vector<double> action_v_set_;
    std::vector<double> action_w_set_;
    double SAFE_DIST_,SAFE_TIME_;
    double ROBOT_RADIUS_;
    double OBSTACLE_RADIUS_; 
    double TIME_HORIZON_;
    double MAX_SPEED_,AVG_SPEED_,MIN_SPEED_;
    double MAX_ROT_SPEED_;
    size_t SLICE_NUM_;
    double GOAL_RADIUS_;
    size_t NUM_SAMPLE_EDGE_;
    double SENSOR_RANGE_;
    

    // map
    GridMap::Ptr grid_map_;
    Vec2d occ_map_origin_,occ_map_size_2d_;
    double resolution_, inv_resolution_;
    
    // time
    double time_origin_;// record the time planed, later for trajectory generation
    double time_resolution_, inv_time_resolution_;

    // open_list
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_list_;
    
    // expanded_list(is needed because no fixed grid/array can be used to index the expanded node)
    PathNodeHashTable expanded_nodes_; //(check with node state OPENSET/CLOSESET/UNDEFINED)

    // trace nodes
    std::vector<PathNodePtr> final_path_nodes_;

    // save visted nodes, place node in fixed vector to save memory
    std::vector<PathNodePtr> path_node_pool_;
    int ALLOCATE_NUM_;
    int use_node_num_;

    // counter
    int  iter_num_;
    int  sample_num_;

    // reach_goal
    bool reached_goal_;
    
    Vec2i posToIndex(Vec2d pos);

    int timeToIndex(double time);

    void getTimedGraph(const std::vector<double>& coords,const std::vector<double>& speeds,
                        const std::vector<double>& angles,std::vector<GraphPtr>& timed_graph);

    bool getNeighborNodes(PathNodePtr &curr_node, std::vector<PathNodePtr> & neighbor_ptr_set,std::vector<double> & edge_cost_set);

    double estimateHeuristic(PathNodePtr &curr_node,Vec2d goal_pos, Vec2d start_pos);

    void setNeighborNodeActionDuration(const PathNodePtr &curr_node, PathNodePtr &next_node);

    bool checkCollisionFree(const PathNodePtr &curr_node, PathNodePtr &next_node,Graph *graph_t, double & dist_to_collid, double & time_to_collid);

    double computeCollisionTime(const Vec2d &p_ir, const Vec2d &v_ir);

    //double computeCollisionTimeAll(const Vec2d &p_r, const Vec2d &v_r, Graph *graph_t);

    void checkStartNodeSafety(const PathNodePtr &start_node, Graph *graph_t);

    void retrievePath(PathNodePtr end_node);

public:
    

    TimedAstar(){}
    ~TimedAstar(){};
    typedef std::shared_ptr<TimedAstar> Ptr;

    void init(GridMap::Ptr grid_map,TimedAstarParam param);
    void reset();
    
    bool TimeAstarSearch(const std::vector<double>& coords,const std::vector<double>& speeds,
                    const std::vector<double>& angles,const Vec2d& robot,const Vec2d& goal,
                    const double & dir_start,const double & time_start);

    std::vector<Eigen::Vector2d> getVistedNodes();
    std::vector<Eigen::Vector2d> getPath();

    std::vector<Eigen::Vector2d> getTrajectory(double ts,double local_time_horizon);

    void getGraph(GraphPtr &graph_t, const double time);
             
};


}

#endif
