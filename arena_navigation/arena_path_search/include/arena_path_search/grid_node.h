#ifndef _GRIDNODE_H_
#define _GRIDNODE_H_

#include <Eigen/Eigen>

#define inf 1 >> 30
struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{   
    // state
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};
	enum enum_state state
	{UNDEFINED};
    
    // index
	Eigen::Vector2i index;

    // score
	double gScore{inf}, fScore{inf};

    // cameFrom
	GridNodePtr cameFrom{NULL};

    // direction of expanding( for jps only )
    Eigen::Vector2i dir; 

    int rounds{0}; // Distinguish every call
    
    GridNode(Eigen::Vector2i _index){  
		state = enum_state::UNDEFINED;
		index = _index;
		dir   = Eigen::Vector2i::Zero();
		gScore = inf;
		fScore = inf;
		cameFrom = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};


class GridNodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

#endif