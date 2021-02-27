#ifndef _GRAPH_SEARCH_H_
#define _GRAPH_SEARCH_H_

#include <Eigen/Eigen>
#include <queue>
#include <ros/ros.h>
#include <ros/console.h>
#include "arena_path_search/grid_node.h"

using namespace std;

class GraphSearch
{
private:

protected:
    // occ map
	//GridMap::Ptr grid_map_;

    // openset
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, GridNodeComparator> openSet_;

    // search map
    GridNodePtr **GridNodeMap_;

    // search map param
    const double tie_breaker_ = 1.0 + 1.0 / 10000;      
    double lambda_heu_;

    Eigen::Vector2i POOL_SIZE_;
    double step_size_, inv_step_size_;                                      // meter/cell self defined
    
    Eigen::Vector2d pos_origin_, pos_bound_min_, pos_bound_max_;            // in meter: pos_origin_=center_pos
    Eigen::Vector2i index_origin_, index_bound_min_, index_bound_max_;      // in cell: index_origin_=center_index_,index_bound_min_=0,0 index_bound_max=pool_size(0),pool_size(1),

    
    
    // container for saved path
    std::vector<GridNodePtr> gridPath_;
    
    // for quick reset
    int rounds_{0};

    /* Heuristic */
	virtual double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	virtual double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	virtual double getEuclHeu(GridNodePtr node1, GridNodePtr node2);

	virtual double getHeu(GridNodePtr node1, GridNodePtr node2);

    /* index-coord transformation */
	inline Eigen::Vector2d Index2Pos(const Eigen::Vector2i &index) const;
    inline Eigen::Vector2i Pos2Index(const Eigen::Vector2d &pos) const;
	inline bool Pos2Index(const Eigen::Vector2d &pos, Eigen::Vector2i &index) const;

    /* Adjust start/end point  */
	virtual bool ConvertToIndexAndAdjustStartEndPoints( Eigen::Vector2d start_pt,  Eigen::Vector2d end_pt, 
                                            Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx)=0;

    /* Occupancy Check */
	virtual bool isOccupied(const Eigen::Vector2d &pos)=0;// { return (bool)grid_map_->getFusedInflateOccupancy(pos); }

    /* Retrieve Path */
	virtual std::vector<GridNodePtr> retrievePath(GridNodePtr current)=0;

    
public:
	//typedef std::shared_ptr<AStar> Ptr;

	GraphSearch(){};
	~GraphSearch(){};
    
    enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };


    /* initilization */
	//virtual void initSearchMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size)=0;

    /* Search */
	//virtual bool Search( Eigen::Vector2d start_pt, Eigen::Vector2d end_pt)=0;

    /* result */
	virtual std::vector<Eigen::Vector2d> getPath()=0;
};

double GraphSearch::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

double GraphSearch::getDiagHeu(GridNodePtr node1, GridNodePtr node2){

    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    
    int diag = min(dx, dy);
    dx -= diag;
    dy -= diag;

    double h = 0.0;
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);

    return h;
}

double GraphSearch::getManhHeu(GridNodePtr node1, GridNodePtr node2){
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    return dx + dy ;
}

double GraphSearch::getEuclHeu(GridNodePtr node1, GridNodePtr node2){
    return (node2->index - node1->index).norm();
}


/* index-coord transformation */
inline Eigen::Vector2d GraphSearch::Index2Pos(const Eigen::Vector2i &index) const
{
    // Eigen::Vector2d pos;
    // pos(0) = ((double)index(0) + 0.5) * step_size_(0) + origin_(0);
    // pos(1) = ((double)index(1) + 0.5) * step_size_(1) + origin_(1);

    //Eigen::Vector2d pos=(index.cast<double> + Eigen::Vector2d(0.5, 0.5)) * step_size_+ origin_;

    return ((index - index_origin_).cast<double>() * step_size_) + pos_origin_;
  
}

inline Eigen::Vector2i GraphSearch::Pos2Index(const Eigen::Vector2d &pos) const
{   
    // Vector2i index;
    // index(0)= int(floor((pos(0)-origin_(0)) * inv_step_size_);
    // index(1)= int(floor((pos(1)-origin_(1)) * inv_step_size_);
    //Vector2i index=((pos-origin_)*inv_step_size_).array().floor().cast<int>();
    
    return ((pos - pos_origin_) * inv_step_size_ + Eigen::Vector2d(0.5, 0.5)).cast<int>() + index_origin_;
}

inline bool GraphSearch::Pos2Index(const Eigen::Vector2d &pos, Eigen::Vector2i &index) const
{
    index = Pos2Index(pos);

    if (index(0) < index_bound_min_(0) || index(0) >= index_bound_max_(0) || index(1) < index_bound_min_(1) || index(1) >= index_bound_max_(1))
	{
		ROS_ERROR("Ran out of pool, index=%d %d", index(0), index(1));
		return false;
	}

	return true;
}


























#endif