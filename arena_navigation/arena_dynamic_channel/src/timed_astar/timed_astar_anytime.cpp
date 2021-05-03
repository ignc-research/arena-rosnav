
#include "arena_dynamic_channel/timed_astar/timed_astar_anytime.h"

namespace timed_astar
{
bool solveRoot(double a0, double a1, double a2, double &x1, double &x2){
    // calculate companion matrix of the polynomial
	Eigen::Matrix2d Compan;
    Compan<<-a1/a2	,	-a0/a2,
			1.0		,  	0.0;

	// solve the eigenvalue of companion matrix as root
	Eigen::EigenSolver<Eigen::MatrixXd> solver(Compan);
	auto T_vals=solver.eigenvalues();
	
    bool is_valid=false;
	// select root
	if(std::abs(T_vals(0).imag())>0.00001){
		// if has imag value, then never collide
		x1=x2=-1.0;
        
        
	}else{
		// have solution, between t1 and t2
		if(T_vals(0).real()>0){
			x1 = std::max(T_vals(1).real(),0.0);   // x_min
			x2 = std::max(T_vals(0).real(),0.0);   // x_max
            is_valid=true; 
            //std::cout<<"x_min:"<<x1<<std::endl;
            //std::cout<<"x_max:"<<x2<<std::endl;
		}else{
			// if t1<0,t2<0 or t1=t2=nan
			x1=x2=-1.0;
		}
		
	}
    return is_valid;
}

double getDistPointToLine(const Vec2d &a, const Vec2d &b1, const Vec2d &b2){
    Vec2d ab1=b1-a;
    Vec2d ab2=b2-a;
    double h=std::abs(0.5*cross(ab1,ab2)/(b1-b2).length());
    return h;
}

std::vector<double> getLineParam(const Vec2d & p1, const Vec2d &p2){
    //A = y2-y1, B = x1-x2, C = x2y1-x1y2
    double a = p2.y - p1.y;
    double b = p1.x - p2.x;
    double c = p2.x*p1.y-p1.x*p2.y;
    if(a<0){
        a=-a;
        b=-b;
        c=-c;
    }
    return std::vector<double>{a, b, c};
}

bool isPointOnSegment(const Vec2d & p1, const Vec2d &p2, const Vec2d & inter_pt){
    double d1=(p1-inter_pt).length();
    double d2=(p2-inter_pt).length();
    double d_sum=(p2-p1).length();
    const double EPS = 1e-5;
    if(abs(d1+d2-d_sum)<=EPS){return true;}else{return false;}

}

bool getTwoLineIntersection(const Vec2d & p1, const Vec2d &p2,const Vec2d &q1, const Vec2d &q2, Vec2d & inter_pt){
    // get line param
    std::vector<double> line1=getLineParam(p1,p2);
    std::vector<double> line2=getLineParam(q1,q2);

    // if parallel 
    if(line1[0]*line2[1]==line1[1]*line2[0]){
        return false;
    }

    // get intersection point
    double x = double(line2[2]*line1[1] - line1[2]*line2[1]) / double(line1[0]*line2[1] - line2[0]*line1[1]);
    double y = double(line1[2]*line2[0] - line2[2]*line1[0]) / double(line1[0]*line2[1] - line2[0]*line1[1]);
    inter_pt=Vec2d(x,y);

    // check if on both segement
    if(isPointOnSegment(p1,p2,inter_pt) && isPointOnSegment(q1,q2,inter_pt)){
        return true;
    }else{
        return false;
    }

}


void TimedAstar::init(GridMap::Ptr grid_map,TimedAstarParam param){
    // param
    param_=param;

    //init map 
    grid_map_ = grid_map;
    
    Eigen::Vector2d occ_map_origin,occ_map_size_2d;
	grid_map_->getRegion(occ_map_origin, occ_map_size_2d);
    occ_map_origin_=Vec2d(occ_map_origin(0),occ_map_origin(1));     
    occ_map_size_2d_=Vec2d(occ_map_size_2d(0),occ_map_size_2d(1));  
    
    // resolution
    resolution_= param.RESOLUTION;
    inv_resolution_=1.0/resolution_;
    
    // time resolution
    time_resolution_=param.TIME_RESOLUTION;
    inv_time_resolution_=1.0/time_resolution_;
    
    // init graph nodes pool
    ALLOCATE_NUM_=param.ALLOCATE_NUM;
    path_node_pool_.resize(ALLOCATE_NUM_);
    for (int i = 0; i < ALLOCATE_NUM_; i++)
  	{
    	path_node_pool_[i] = std::make_shared<PathNode>();
  	}
    
    // search param
    use_node_num_=0;
    iter_num_=0;
    
    MAX_SPEED_=param.MAX_SPEED;
    AVG_SPEED_=param.AVG_SPEED;
    MIN_SPEED_=param.MIN_SPEED;
    MAX_ROT_SPEED_=param.MAX_ROT_SPEED;
    action_v_set_={param.MAX_SPEED,param.AVG_SPEED,param.MIN_SPEED};
    action_w_set_={param.MAX_ROT_SPEED};
    
    SAFE_DIST_      = param.SAFE_DIST;
    ROBOT_RADIUS_   = param.ROBOT_RADIUS;
    OBSTACLE_RADIUS_= param.OBSTACLE_RADIUS;  
    TIME_HORIZON_   = param.TIME_HORIZON;
    SLICE_NUM_      = param.TIME_SLICE_NUM;
    GOAL_RADIUS_    = param.GOAL_RADIUS;
    NUM_SAMPLE_EDGE_= param.NUM_SAMPLE_EDGE;
    SAFE_TIME_      = param.SAFE_TIME;//   2.0;
    SENSOR_RANGE_   = param.SENSOR_RANGE;

}

void TimedAstar::reset()
{   
    // reset final_path_nodes
    final_path_nodes_.clear();

    // reset expanded list
    expanded_nodes_.clear();

    //reset open list
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_list_.swap(empty_queue);

    // init path_node_pool (for trace all expanded nodes)
    for (int i = 0; i < use_node_num_; i++)
    {
        PathNodePtr node = path_node_pool_[i];
        node->parent = nullptr;
        node->node_state =PathNode::UNDEFINED;
    }
    // reset timed_graph
    timed_graph_.clear();

    // reset counter
    use_node_num_ = 0;
    iter_num_ = 0;
    
}

void TimedAstar::getTimedGraph(const std::vector<double>& coords,
                                const std::vector<double>& speeds,
                                const std::vector<double>& angles,
                                std::vector<GraphPtr>& timed_graph) {

    // clear data
    timed_graph.clear();
    
    
    // decode the init and goal
    auto x_init = coords[INIT_INDEX*2];
    auto y_init = coords[INIT_INDEX*2+1];
    auto x_goal = coords[GOAL_INDEX*2];
    auto y_goal = coords[GOAL_INDEX*2+1];
    // decode the boundaries
    auto x_min = x_init -   SENSOR_RANGE_;
    auto y_min = y_init -   SENSOR_RANGE_;
    auto x_max = x_init +   SENSOR_RANGE_;
    auto y_max = y_init +   SENSOR_RANGE_;

    // decode initial boundray triangle
    auto x_b1 = coords[PHASE1_INDEX*2];
    auto y_b1 = coords[PHASE1_INDEX*2+1];
    auto x_b2 = coords[PHASE2_INDEX*2];
    auto y_b2 = coords[PHASE2_INDEX*2+1];
    auto x_b3 = coords[PHASE3_INDEX*2];
    auto y_b3 = coords[PHASE3_INDEX*2+1];
    auto x_b4 = coords[PHASE4_INDEX*2];
    auto y_b4 = coords[PHASE4_INDEX*2+1];
    // auto x_min = std::min(std::min(coords[PHASE1_INDEX*2],coords[PHASE2_INDEX*2]),std::min(coords[PHASE3_INDEX*2],coords[PHASE4_INDEX*2]));
    // auto y_min = std::min(std::min(coords[PHASE1_INDEX*2+1],coords[PHASE2_INDEX*2+1]),std::min(coords[PHASE3_INDEX*2+1],coords[PHASE4_INDEX*2+1]));
    // auto x_max = std::max(std::max(coords[PHASE1_INDEX*2],coords[PHASE2_INDEX*2]),std::max(coords[PHASE3_INDEX*2],coords[PHASE4_INDEX*2]));
    // auto y_max = std::max(std::max(coords[PHASE1_INDEX*2+1],coords[PHASE2_INDEX*2+1]),std::max(coords[PHASE3_INDEX*2+1],coords[PHASE4_INDEX*2+1]));
    
    // discretize the time space
    for (size_t t = 0; t < SLICE_NUM_; ++t) {
        std::vector<double> coord_t, speed_t, angle_t;

        // push init and goal
        //coord_t.push_back(x_init);
        //coord_t.push_back(y_init);
        coord_t.push_back(x_goal);
        coord_t.push_back(y_goal);

        coord_t.push_back(x_b1);
        coord_t.push_back(y_b1);

        coord_t.push_back(x_b2);
        coord_t.push_back(y_b2);

        coord_t.push_back(x_b3);
        coord_t.push_back(y_b3);

        //coord_t.push_back(x_b4);
        //coord_t.push_back(y_b4);
        for (size_t i = 0; i < coord_t.size(); ++i) {
            speed_t.push_back(0.0);
            angle_t.push_back(0.0);
        }

        // time evolution
        // std::cout << "new node:" << std::endl;
        for (size_t i = 12; i < coords.size(); i += 2) {
            auto xi = coords[i] + speeds[i] * time_resolution_ * t;
            auto yi = coords[i+1] + speeds[i+1] * time_resolution_ * t;
            if (xi < x_min || xi > x_max || yi < y_min || yi > y_max){
                continue;
            }
            // std::cout << xi << " " << yi << std::endl;
            // update state space
            coord_t.push_back(xi);
            coord_t.push_back(yi);
            speed_t.push_back(speeds[i]);
            speed_t.push_back(speeds[i+1]);
            angle_t.push_back(angles[i]);
            angle_t.push_back(angles[i+1]);
        }

        // generate timed graph and timed close list
        auto graph_t = std::make_shared<Graph>(coord_t, speed_t, angle_t);
        timed_graph.push_back(graph_t);
    }
}

bool TimedAstar::TimeAstarSearch(const std::vector<double>& coords,
                    const std::vector<double>& speeds,
                    const std::vector<double>& angles,
                    const Vec2d& robot,
                    const Vec2d& goal,
                    const double & dir_start,
                    const double & time_start){

             
    // reset
    reset();
    sample_num_=0;
    start_pos_  = robot;
    goal_pos_   = goal;
    time_origin_= time_start;
    reached_goal_=false;
    

    // init time_graphs
    getTimedGraph(coords, speeds, angles, timed_graph_);
     
    // locate robot in start graph
    auto init_eid = dl::locateCurrentFace(timed_graph_[0].get(), start_pos_.x, start_pos_.y);
    if (init_eid == NONE_INDEX) {
        std::cout << "Invalid start position!" << std::endl;
        return false;
    }
  
    // seed the start node
    PathNodePtr start_node = std::make_shared<PathNode>(); //path_node_pool_[0];
    start_node->parent     = nullptr;
    start_node->pos        =start_pos_;
    start_node->vel        = Vec2d(speeds[INIT_INDEX*2],speeds[INIT_INDEX*2+1]);
    start_node->dir        =dir_start;
    start_node->setTimedTriangle(start_pos_,0.0,timed_graph_,time_resolution_,SLICE_NUM_);
    start_node->G          = 0.0;
    checkStartNodeSafety(start_node,timed_graph_[0].get());
    start_node->H          =estimateHeuristic(start_node,goal_pos_,start_pos_);
    start_node->node_state = PathNode::OPENSET;

    
    
    
    // push_into openlist
    open_list_.push(start_node);
    expanded_nodes_.insert(posToIndex(start_node->pos),timeToIndex(start_node->time_elapsed),start_node);
    path_node_pool_[0]=start_node;
    use_node_num_ += 1;

    ros::Time t_start=ros::Time::now();

    // begin astar searching
    //std::cout<<"[time astar search]: begin while loop search---------------"<<std::endl;
    double nearest_dist = DBL_MAX;
    PathNodePtr goal_reached_node(nullptr);
    PathNodePtr goal_nearest_node(nullptr);
    while (!open_list_.empty()) {
        PathNodePtr curr_node = open_list_.top();
        
        
        // the goal or time horizon is reached
        double dist_to_goal = (curr_node->pos - goal_pos_).length();
        if (dist_to_goal < GOAL_RADIUS_) 
        {
            goal_reached_node = curr_node;
            reached_goal_=true;
            break;
        }
        
        // record the nearest node
        if (dist_to_goal < nearest_dist) {
            nearest_dist = dist_to_goal;
            goal_nearest_node = curr_node;
        }

        // // computation time limit reached
        // if((ros::Time::now()-t_start).toSec()<0.5){
        //     ROS_WARN("[Timed_astar] time limited reached, use goal nearest node ");
        //     break;
        // }

        

        // put it into closed set
        open_list_.pop();
        curr_node->node_state = PathNode::CLOSEDSET;
        iter_num_ += 1;
           
        // find neigbors
        // Plan in a future target time slice
        std::vector<PathNodePtr> neighbor_ptr_set;
        std::vector<double> edge_cost_set;
        getNeighborNodes(curr_node,neighbor_ptr_set,edge_cost_set);
       
        PathNodePtr neigbor_node;
        double tmp_g_score;

        // discover new node from neighbors
        for(size_t i=0;i<neighbor_ptr_set.size();++i)
        {   
            
            neigbor_node=neighbor_ptr_set[i];
            tmp_g_score =curr_node->G + edge_cost_set[i];
            
            auto pos_index  = posToIndex(neigbor_node->pos);
            auto time_index = timeToIndex(neigbor_node->time_elapsed);
            
            PathNodePtr pro_node = expanded_nodes_.find(pos_index,time_index);
            if(pro_node==nullptr)
            {   // neigbor_node is undefined
                
                neigbor_node->parent=curr_node;
                
                neigbor_node->G=tmp_g_score;
                neigbor_node->H=estimateHeuristic(neigbor_node, goal_pos_,start_pos_);
                
                neigbor_node->setTimedTriangle(neigbor_node->pos,neigbor_node->time_elapsed,timed_graph_,time_resolution_,SLICE_NUM_);
                
                neigbor_node->node_state=PathNode::OPENSET;
                // add to openlist
                open_list_.push(neigbor_node);
                
                // add to expanded nodes
                expanded_nodes_.insert(pos_index, time_index, neigbor_node);
                
                path_node_pool_[use_node_num_]=neigbor_node;
                use_node_num_ += 1;
                 
                if (use_node_num_ >= ALLOCATE_NUM_)
                {
                    std::cout << "run out of memory." << std::endl;
                }
               
                
            }else{
                
                if(pro_node->node_state==PathNode::CLOSEDSET){
                    // in closeset
                    
                    continue;
                }else
                {
                    // in openset
                    neigbor_node->node_state=PathNode::OPENSET;
                    if (tmp_g_score < pro_node->G){
                        
                        pro_node->parent = curr_node;
                        
                        pro_node->G = tmp_g_score;
                        pro_node->H = estimateHeuristic(neigbor_node, goal_pos_,start_pos_);
                        pro_node->pos=neigbor_node->pos;
                        pro_node->dir=neigbor_node->dir;
                        pro_node->time_elapsed=neigbor_node->time_elapsed;
                        pro_node->v_in=neigbor_node->v_in;
                        pro_node->w_in=neigbor_node->w_in;
                        pro_node->dur_v=neigbor_node->dur_v;
                        pro_node->dur_w=neigbor_node->dur_w;
                        pro_node->setTimedTriangle(neigbor_node->pos,neigbor_node->time_elapsed,timed_graph_,time_resolution_,SLICE_NUM_);
                        
                    }
                    
                } 
            }
        }
    }

    // check if find anynode
    if (goal_nearest_node == nullptr) {
        std::cout<<"[time astar search]: done[1] not found any thing---------------"<<std::endl;
        retrievePath(start_node);
        return false;
    }
    std::cout << "*************************************************" << std::endl;
    // if goal_reached_node is nullptr
    auto ancestor = goal_reached_node;
    if (ancestor == nullptr) {
        ancestor = goal_nearest_node;
        std::cout << "not reach goal" << std::endl;
    }else{
        std::cout << " goal reached " << std::endl;
    }
    // retrieve path
    retrievePath(ancestor);
    
    std::cout << "path size" << final_path_nodes_.size()<<std::endl;
    std::cout << "tried sample num: " << sample_num_ << std::endl;
    std::cout << "use node num: " << use_node_num_ << std::endl;
    std::cout << "iter num: " << iter_num_ << std::endl;
    
    return true;
}

double TimedAstar::computeCollisionTime(const Vec2d &p_ir, const Vec2d &v_ir){
    double a2,a1,a0;
    a2   = dot(v_ir,v_ir);
	a1   = 2*dot(p_ir,v_ir);
	a0   = dot(p_ir,p_ir)-SAFE_DIST_*SAFE_DIST_;
    double t_min,t_max;
    bool is_valid_solution=solveRoot(a0,a1,a2,t_min,t_max);

    double time_to_collide;
    double safe_time =SAFE_TIME_;                      // safe time sec parameter= 2.0 or TIME_HORIZON_
    
    if(!is_valid_solution){
        time_to_collide=DBL_MAX;                //  will never collide
    }else
    {   
        if(t_min > TIME_HORIZON_){
            time_to_collide=DBL_MAX;            // will not collide in TIME_HORIZON_
        }else{
            time_to_collide = t_min;
            
        }
    }
  
    return time_to_collide;
}

void TimedAstar::checkStartNodeSafety(const PathNodePtr &start_node, Graph *graph_t){
    Vec2d p_r =start_node->pos;
    Vec2d v_r =start_node->vel;
    
    double min_time_to_collide=DBL_MAX, min_dist_to_collide=DBL_MAX;
    for (size_t i = 0; i < graph_t->triangles.size(); ++i){
        size_t id = graph_t->triangles[i];
        Vec2d p_i = Vec2d(graph_t->coords[2*id], graph_t->coords[2*id+1]);
        Vec2d v_i = Vec2d(graph_t->speeds[2*id], graph_t->speeds[2*id+1]);
        
        if(v_i.x==0 && v_i.y==0){
            continue;           // if not an obstacle but start, goal, boundary
        }

        double time_to_collide;
        Vec2d p_ir, v_ir;
        p_ir = p_i - p_r;
        v_ir = v_i - v_r;
        time_to_collide=computeCollisionTime(p_ir,v_ir);
        if(time_to_collide<min_time_to_collide){
            min_time_to_collide=time_to_collide;
        }
    }
    if(min_time_to_collide<SAFE_TIME_){
        start_node->is_unsafe=true;
    }else{
         start_node->is_unsafe=false;
    }
    
}

void TimedAstar::setNeighborNodeActionDuration(const PathNodePtr &curr_node, PathNodePtr &next_node){
    // compute heading for next_node 
    next_node->dir   =  (next_node->pos-curr_node->pos).angle();

    // compute difference of heading & w direction
    double diff_dir = next_node->dir - curr_node->dir;
    if(diff_dir>PI || (diff_dir<0 && diff_dir>- PI)){
        next_node->w_in= -next_node->w_in;
    }

    // compute duration
    next_node->dur_w = std::abs((next_node->dir - curr_node->dir)/next_node->w_in);
    next_node->dur_v = std::abs((next_node->pos - curr_node->pos).length()/next_node->v_in);
    
}

bool TimedAstar::checkCollisionFree(const PathNodePtr &curr_node, PathNodePtr &next_node, Graph *graph_t, double & dist_to_collid, double & time_to_collid)
{   

    // test collision constraints
    Vec2d p_r =curr_node->pos;
    Vec2d v_r =Vec2d(next_node->v_in *cos(next_node->dir),curr_node->v_in*sin(next_node->dir));
    double min_time_to_collide=DBL_MAX, min_dist_to_collide=DBL_MAX;
    
    double dur_safe=SAFE_TIME_;
    
    for (size_t i = 0; i < graph_t->triangles.size(); ++i) {
        size_t id = graph_t->triangles[i];
        Vec2d p_i = Vec2d(graph_t->coords[2*id], graph_t->coords[2*id+1]);
        Vec2d v_i = Vec2d(graph_t->speeds[2*id], graph_t->speeds[2*id+1]);

        if(v_i.x==0 && v_i.y==0){
            // if not an obstacle but start, goal, boundary
            continue;
        }
        
        double time_to_collide_w,time_to_collide_v;
        Vec2d p_ir, v_ir;
        
        //check collision in w state
        p_ir = p_i - p_r;
        v_ir = v_i ;
        time_to_collide_w=computeCollisionTime(p_ir,v_ir);
        if(time_to_collide_w < next_node->dur_w){
            // if this direction meet collision, then the whole node is deleted
            time_to_collid=0;
            dist_to_collid=0;
            return false;
        }

	    // check collision in v state
        p_ir = (p_i + v_i *next_node->dur_w)- p_r;
        v_ir = v_i - v_r;
        time_to_collide_v=computeCollisionTime(p_ir,v_ir);

        if(time_to_collide_v <= next_node->dur_v){
            // if collision happens within action time
            if(time_to_collide_v - dur_safe<0){
                time_to_collid=0;
                dist_to_collid=0;
                return false;
            }else{
                
                //check safe distance at time (time_to_collide_v-dur_safe)
                double new_dur_1 = time_to_collide_v-dur_safe;
                double new_dur_2 = dur_safe;
                if(new_dur_1<0){
                    time_to_collid=0;
                    dist_to_collid=0;
                    return false;
                }
                double dist_1 =sqrt(dot((p_ir + (v_ir * new_dur_1)),(p_ir + (v_ir * new_dur_1))));
                double dist_2 =sqrt(dot((p_ir + (v_ir * new_dur_2)),(p_ir + (v_ir * new_dur_2))));
                if(dist_1>SAFE_DIST_ && next_node->v_in*new_dur_1>1.0){
                    next_node->pos = curr_node->pos + (next_node->pos-curr_node->pos)*new_dur_1/next_node->dur_v;
                    next_node->dur_v=new_dur_1;

                    if(dur_safe+time_to_collide_w<min_time_to_collide){
                        min_time_to_collide = dur_safe+time_to_collide_w;
                    }
                    continue;
                }
                
                if(new_dur_2>new_dur_1){
                    time_to_collid=0;
                    dist_to_collid=0;
                    return false;
                }

                if(dist_2>SAFE_DIST_ && next_node->v_in*new_dur_2>1.0){
                    next_node->pos = curr_node->pos + (next_node->pos-curr_node->pos)*new_dur_2/next_node->dur_v;
                    next_node->dur_v=new_dur_2;
                    if(time_to_collide_v-dur_safe+time_to_collide_w<min_time_to_collide){
                        min_time_to_collide = time_to_collide_v-dur_safe+time_to_collide_w;
                    }
                    continue;
                }else{
                    time_to_collid=0;
                    dist_to_collid=0;
                    return false;
                }
               
            }
        }else{
            // collision won't happen within action time, and safe distance is keeped
            if(time_to_collide_v+time_to_collide_w<min_time_to_collide){
                min_time_to_collide = time_to_collide_v+time_to_collide_w;
                min_dist_to_collide = time_to_collide_v*v_ir.length();
            }
        }
        
        
    }

    time_to_collid=min_time_to_collide;
    dist_to_collid=min_dist_to_collide;
    return true;
}

bool TimedAstar::getNeighborNodes(PathNodePtr &curr_node, std::vector<PathNodePtr> & neighbor_ptr_set,std::vector<double> & edge_cost_set){
    // init neighbor_ptr_set & edge_cost_set
    neighbor_ptr_set.clear();
    edge_cost_set.clear();
    std::vector<Vec2d> sample_set;
      
    // check if reached time horizon
    bool reached_time_horizon=false;
    if(curr_node->sid >= SLICE_NUM_-1){
        reached_time_horizon=true;
    }

    // get time_graph at time sid+1
    size_t curr_slice_id=std::min(curr_node->sid+1,SLICE_NUM_-1);
    auto graph_t = timed_graph_[curr_slice_id].get();

    // find triangle id on time_graph[t], where current node is in
    auto curr_eid = dl::locateCurrentFace(graph_t, curr_node->pos.x, curr_node->pos.y);
    auto goal_eid = dl::locateCurrentFace(graph_t, goal_pos_.x, goal_pos_.y);
    double curr_goal_diff = (curr_node->pos - goal_pos_).length();
    
    // add goal into sample_set
    //sample_set.push_back(goal_pos_);
    bool flag_goal_added=false;

    // search for other samples on timed_graph
    if(floor(curr_eid / 3.0)==floor(goal_eid / 3.0) || curr_goal_diff<SAFE_DIST_ )//
    {   // if in same triangle or near to the goal 
        std::cout<<"GOAL is in same triangle"<<std::endl;
        sample_set.push_back(goal_pos_);
    }else{
        // find 3 vertice of current triangle
        std::vector<NodePtr> vertices;
        vertices.reserve(3);
        vertices.push_back(std::make_shared<Node>(curr_eid));
        vertices.push_back(std::make_shared<Node>(dl::nextHalfedge(curr_eid)));
        vertices.push_back(std::make_shared<Node>(dl::prevHalfedge(curr_eid)));
        vertices[0]->fetch_states(graph_t);
        vertices[1]->fetch_states(graph_t);
        vertices[2]->fetch_states(graph_t);

        // sample points on triangle edge
        size_t k_sample=NUM_SAMPLE_EDGE_;;
        if(iter_num_==1){
            // first step need more sample_set to guarantee the solution
            k_sample=3*NUM_SAMPLE_EDGE_;
        }
        for(size_t i=0;i<vertices.size();++i){
            size_t j=i<2?i+1:0;
            //double height=getDistPointToLine(curr_node->pos,vertices[i]->position,vertices[j]->position);
            double length= (vertices[i]->position-vertices[j]->position).length();
            
            // if the edge(gate) not big enough to go through, pass it
            if(length< SAFE_DIST_) continue;
            
            //add goal directed point
            Vec2d inter_pt;
            bool have_inter_pt=getTwoLineIntersection(vertices[i]->position,vertices[j]->position,curr_node->pos,goal_pos_,inter_pt);
            if(have_inter_pt){
                sample_set.push_back(inter_pt);
                if(!flag_goal_added){
                    // add goal into sample_set
                    if(curr_slice_id==SLICE_NUM_-1){
                        sample_set.push_back(goal_pos_);
                        flag_goal_added=true;
                    }
                }
                
            }

            // add uniform sampled points
            for(size_t k=1;k<=k_sample;k++)
            {
                Vec2d sample;
                double a =double(k)/double(k_sample+1);
                sample=vertices[i]->position * (1-a)+ vertices[j]->position * a;
                sample_set.push_back(sample);
            }
        }
    }
    
    // check safety for the sample_set
    for(size_t i=0;i<sample_set.size();i++)
    {   sample_num_++;
        for(size_t action_id=0; action_id<action_v_set_.size();++action_id)
        {   
            
            // init candidate neighbor
            PathNodePtr next_node=std::make_shared<PathNode>();
            next_node->pos=sample_set[i];
            next_node->w_in=action_w_set_[0];
            next_node->v_in=action_v_set_[action_id];
            
            // set action duration
            setNeighborNodeActionDuration(curr_node,next_node );
            
            // check if collision free within action trajectory duration
            double dist_to_potential_collid, time_to_potential_collid;
            bool no_collision=checkCollisionFree(curr_node,next_node,graph_t,dist_to_potential_collid,time_to_potential_collid);
            
            //no_collision=true;
            if(no_collision || reached_time_horizon)
            {   
                //calculate edge cost
                double k =1.0;
                double weight= time_to_potential_collid > 2*SAFE_TIME_?1: 2*SAFE_TIME_/(time_to_potential_collid+0.0001);
                
                double edge_cost =next_node->dur_v+ next_node->dur_w;

             
                // double dir_goal_start   =(goal_pos_-start_pos_).angle(); //double dir_start_robot  =(curr_node->pos-start_pos).angle();
                // double dir_diff = std::abs(dir_goal_start-next_node->dir);
                // dir_diff=dir_diff>PI?2*PI-dir_diff:dir_diff;
                // double dur_dir = dir_diff/MAX_ROT_SPEED_;

                // edge_cost=edge_cost+dur_dir;
                //edge_cost=weight*edge_cost;

                

                // pushback node & edge cost
                neighbor_ptr_set.push_back(next_node);
                edge_cost_set.push_back(edge_cost);
            }
        }
    }
    
    if(neighbor_ptr_set.empty()) return false;

    return true;

}

double TimedAstar::estimateHeuristic(PathNodePtr &curr_node,Vec2d goal_pos, Vec2d start_pos){
    // arrving time with MAX_SPEED, so to make it admissable
    double curr_dir         = curr_node->dir;
    double dir_goal_robot   =(goal_pos-curr_node->pos).angle(); //double dir_start_robot  =(curr_node->pos-start_pos).angle();
    double dir_diff = std::abs(dir_goal_robot-curr_dir);
    dir_diff=dir_diff>PI?2*PI-dir_diff:dir_diff;
    double dur_dir = dir_diff/MAX_ROT_SPEED_;
	double dur_arrive = (curr_node->pos-goal_pos).length()/MIN_SPEED_;//curr_node->v_in;//MIN_SPEED_;
    double dur_to_avoid;
    if(curr_node->is_unsafe){
        dur_to_avoid = PI/MAX_ROT_SPEED_ + 2.0/MIN_SPEED_;//10*SAFE_TIME_;
    }else{
        dur_to_avoid =0.0;
    }
    
    return dur_arrive + dur_dir + dur_to_avoid;
}

void TimedAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  final_path_nodes_.push_back(cur_node);

  while (cur_node->parent != nullptr)
  {
    cur_node = cur_node->parent;
    final_path_nodes_.push_back(cur_node);
  }

  std::reverse(final_path_nodes_.begin(), final_path_nodes_.end());
}

std::vector<Eigen::Vector2d> TimedAstar::getVistedNodes(){
    std::vector<Eigen::Vector2d> visted_points;
    for(size_t i=0;i<path_node_pool_.size();++i){
        visted_points.push_back(Eigen::Vector2d(path_node_pool_[i]->pos.x,path_node_pool_[i]->pos.y));
    }
    return visted_points;
}

std::vector<Eigen::Vector2d> TimedAstar::getPath(){
    std::vector<Eigen::Vector2d> waypoints;
    for(size_t i=0;i<final_path_nodes_.size();++i){
        waypoints.push_back(Eigen::Vector2d(final_path_nodes_[i]->pos.x,final_path_nodes_[i]->pos.y));
    }
    return waypoints;
}

std::vector<Eigen::Vector2d> TimedAstar::getTrajectory(double ts, double local_time_horizon){
    std::vector<Eigen::Vector2d> point_set;
    double total_time=0.0;

    // add reachable part of the trajecotry
    for(size_t i=0;i<final_path_nodes_.size()-1;++i){
        PathNodePtr curr_node=final_path_nodes_[i];
        PathNodePtr next_node=final_path_nodes_[i+1];
        
        double t_curr=0;
        // get avg velocity of the robot (accounting for rotation & linear time )
        Vec2d v_robot =Vec2d(next_node->v_in *cos(next_node->dir),next_node->v_in*sin(next_node->dir));
        v_robot = v_robot * next_node->dur_v/(next_node->dur_v+next_node->dur_w);
        
        // add first pos of the state trajectory from curr_node to next_node
        Vec2d pos_curr  =curr_node->pos;
        point_set.push_back(Eigen::Vector2d(pos_curr.x,pos_curr.y));
        //still & rotate
        // while(t_curr < next_node->dur_w){
        //     pos_curr = pos_curr + v_robot*(0.01* ts);
        //     std::cout<<" ******curr time:"<<total_time<<std::endl;
        //     std::cout<<" ******node posw:"<<pos_curr.x <<"   "<<pos_curr.y<<std::endl;
        //     point_set.push_back(Eigen::Vector2d(pos_curr.x,pos_curr.y));
        //     t_curr=t_curr + 0.01*ts;
        //     //total_time=total_time+ts;
        //     if(total_time>local_time_horizon) return point_set;
        // }
        // run straight
        
        // add poses along the straight segement of the state trajectory from curr_node to next_node
        t_curr=0.0;
        while(t_curr < next_node->dur_v-0.00001*next_node->dur_w){
            pos_curr = pos_curr + v_robot * ts;
            //std::cout<<" ******curr time:"<<total_time<<std::endl;
            //std::cout<<" ******node posv:"<<pos_curr.x <<"   "<<pos_curr.y<<std::endl;
            point_set.push_back(Eigen::Vector2d(pos_curr.x,pos_curr.y));
            t_curr=t_curr + ts;
            total_time=total_time+ts;
            //if(total_time>local_time_horizon) return point_set;
        }
    }
    
    if(!reached_goal_){
        // if the path to the goal is blocked, because of obstacle or run out of time
        return point_set;
    }

    // add final part of trajectory
    PathNodePtr last_node=final_path_nodes_[final_path_nodes_.size()-1];
    double dist_to_goal=(last_node->pos - goal_pos_).length();
    if(dist_to_goal > GOAL_RADIUS_){
        double dir   =  (goal_pos_-last_node->pos).angle();
        Vec2d v_robot =Vec2d(param_.MIN_SPEED * cos(dir), param_.MIN_SPEED * sin(dir));
        double t = dist_to_goal / param_.MIN_SPEED;
        Vec2d pos_curr=last_node->pos;
        while(t>0){
            pos_curr = pos_curr + v_robot* ts;
            //std::cout<<" ******curr time:"<<total_time<<std::endl;
            //std::cout<<" ******node pos last:"<<pos_curr.x <<"   "<<pos_curr.y<<std::endl;
            point_set.push_back(Eigen::Vector2d(pos_curr.x,pos_curr.y));
            t=t-ts;
            total_time=total_time+ts;
            //if(total_time>local_time_horizon) return point_set;
        }
    }

    return point_set;
}

Vec2i TimedAstar::posToIndex(Vec2d pos){
    Vec2d temp=(pos-occ_map_origin_)*inv_resolution_;
    Vec2i idx=Vec2i(int(floor(temp.x)),int(floor(temp.y)));
    return idx;
}

int TimedAstar::timeToIndex(double time)
{
    int time_idx = floor((time - 0.0) *inv_time_resolution_);
    return time_idx;
}

void TimedAstar::getGraph(GraphPtr &graph_t, const double time=0.0){
    size_t t = timeToIndex(time);
    t= std::min(t,SLICE_NUM_);
    graph_t=timed_graph_[t];
}



}