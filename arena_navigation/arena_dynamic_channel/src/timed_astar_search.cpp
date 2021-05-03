#include "arena_dynamic_channel/timed_astar_search.h"

TimedAstarSearch::~TimedAstarSearch(){}

void TimedAstarSearch::init(ros::NodeHandle & nh,GridMap::Ptr grid_map, std::vector<DynamicObstacleInfo::Ptr> obs_info_provider){
    node_=nh;

    // init parameters
    node_.param("timed_astar/allocate_num",         tap_.ALLOCATE_NUM,      10000);
    node_.param("timed_astar/goal_tolerance",       tap_.GOAL_RADIUS,       0.2);

    node_.param("timed_astar/robot_radius",         tap_.ROBOT_RADIUS,      0.3);
    node_.param("timed_astar/obstacle_radius",      tap_.OBSTACLE_RADIUS,   0.3);

    node_.param("timed_astar/max_vel",              tap_.MAX_SPEED,         1.8);
    node_.param("timed_astar/avg_vel",              tap_.AVG_SPEED,         1.0);
    node_.param("timed_astar/min_vel",              tap_.MIN_SPEED,         0.5);
    node_.param("timed_astar/max_rot_vel",          tap_.MAX_ROT_SPEED,     0.52);

    node_.param("timed_astar/sensor_range",         tap_.SENSOR_RANGE,      5.0);
    node_.param("timed_astar/time_horizon",         tap_.TIME_HORIZON,      5.0);
    node_.param("timed_astar/time_resolution",      tap_.TIME_RESOLUTION,   1.2);
    node_.param("timed_astar/resolution",           tap_.RESOLUTION,        0.1);
    node_.param("timed_astar/num_sample_edge",      tap_.NUM_SAMPLE_EDGE,   5);
    node_.param("timed_astar/safe_time",            tap_.SAFE_TIME,         1.5);
    
    tap_.TIME_SLICE_NUM=static_cast<size_t>(round(tap_.TIME_HORIZON / tap_.TIME_RESOLUTION));
    tap_.SAFE_DIST =(tap_.ROBOT_RADIUS + tap_.OBSTACLE_RADIUS)*2;
    tap_.SENSOR_RANGE = tap_.SENSOR_RANGE;
    // init map
    this->grid_map_ = grid_map;
    grid_map_->getRegion(occ_map_origin_, occ_map_size_2d_);

    // obstalce info provider
    obs_info_provider_ = obs_info_provider;

    // init timed_astar_planner
    timed_astar_planner_.reset(new TimedAstar);
    timed_astar_planner_->init(grid_map_,tap_);

    visited_points_pub_ = node_.advertise<visualization_msgs::Marker>("vis_visted_samples", 1);
    
}

bool TimedAstarSearch::stateTimeAstarSearch(const Eigen::Vector2d & start_pos,
                                            const Eigen::Vector2d & start_vel,
                                            const double          & start_dir,
                                            const Eigen::Vector2d & end_pos,
                                            double                & ts,
                                            double                & local_time_horizon,
                                            std::vector<Eigen::Vector2d> &point_set,
                                            std::vector<Eigen::Vector2d> &start_end_derivatives,
                                            std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &line_sets){


    
    // init
    std::vector<double> coords;
    std::vector<double> speeds;
    std::vector<double> angles;
    coords.reserve(100*2);
    speeds.reserve(100*2);
    angles.reserve(100*2);

    Eigen::Vector2d corner_min = start_pos - Eigen::Vector2d(1.0,1.0) *tap_.SENSOR_RANGE;//Eigen::Vector2d(1.0,1.0) * tap_.AVG_SPEED * tap_.TIME_HORIZON;
    Eigen::Vector2d corner_max = start_pos + Eigen::Vector2d(1.0,1.0) *tap_.SENSOR_RANGE;//Eigen::Vector2d(1.0,1.0) * tap_.AVG_SPEED * tap_.TIME_HORIZON;

    Eigen::Vector2d vec_start2end=end_pos-start_pos;
    double dist_start2end=vec_start2end.norm();
    double dist_range=1.0;                  //tap_.SENSOR_RANGE;///1.8;//dist_start2end > tap_.SENSOR_RANGE?tap_.SENSOR_RANGE:dist_start2end;
    Eigen::Vector2d vec_norm=Eigen::Vector2d(vec_start2end(1),-vec_start2end(0))/dist_start2end;
    Eigen::Vector2d M = start_pos + dist_range/dist_start2end*(end_pos-start_pos);
    boundPosition(M);
    Eigen::Vector2d M0= start_pos - vec_start2end/dist_start2end *1.0;
    Eigen::Vector2d N1= M0 + vec_norm * 3.5;//tap_.SENSOR_RANGE;//M + vec_norm * tap_.SENSOR_RANGE;
    Eigen::Vector2d N2= M0 - vec_norm * 3.5;//tap_.SENSOR_RANGE;//M - vec_norm * tap_.SENSOR_RANGE;
    Eigen::Vector2d M1= M  + vec_norm * 2;//tap_.SENSOR_RANGE;//M + vec_norm * tap_.SENSOR_RANGE;
    Eigen::Vector2d M2= M  - vec_norm * 2;//tap_.SENSOR_RANGE;//M - vec_norm * tap_.SENSOR_RANGE;
    
    boundPosition(N1);
    boundPosition(N2);
    boundPosition(corner_min);
	boundPosition(corner_max);

    // INIT INDEX
    coords.push_back(start_pos(0));
    coords.push_back(start_pos(1));
    speeds.push_back(start_vel(0));
    speeds.push_back(start_vel(1));

    // GOAL_INDEX
    coords.push_back(end_pos(0));
    coords.push_back(end_pos(1));
    speeds.push_back(0.0);
    speeds.push_back(0.0);

    // PHASE1_INDEX
    coords.push_back(N1(0)); //corner_max
    coords.push_back(N1(1));
    speeds.push_back(0.0);
    speeds.push_back(0.0);

    // PHASE2_INDEX
    coords.push_back(N2(0));
    coords.push_back(N2(1));//corner_max
    speeds.push_back(0.0);
    speeds.push_back(0.0);

    // PHASE3_INDEX
    coords.push_back(M(0));//  M corner_min
    coords.push_back(M(1)); //M corner_min(1)
    speeds.push_back(0.0);
    speeds.push_back(0.0);

    // PHASE4_INDEX
    coords.push_back(M2(0));//M
    coords.push_back(M2(1));//M//corner_min
    speeds.push_back(0.0);
    speeds.push_back(0.0);
    
    // obstacles
	for (std::vector<DynamicObstacleInfo::Ptr>::iterator it = obs_info_provider_.begin() ; it != obs_info_provider_.end(); it++)
    {   
        const DynamicObstacleInfo::Ptr & obs_info = *it;
		Eigen::Vector2d obs_pos=obs_info->getPosition();
		Eigen::Vector2d obs_vel=obs_info->getVelocity();
        coords.push_back(obs_pos(0));
        coords.push_back(obs_pos(1));
        speeds.push_back(obs_vel(0));
        speeds.push_back(obs_vel(1));
    }
   
    // timed_astar_search
    bool success;
    success=timed_astar_planner_->TimeAstarSearch(coords,speeds,angles,Eigen2dToVec2d(start_pos),Eigen2dToVec2d(end_pos) ,start_dir,0.0);
    
    getTriangleEdges(line_sets); // for visualization
    //std::vector<Eigen::Vector2d> visted_points = timed_astar_planner_->getVistedNodes();
    //visualizePoints(visted_points,0.3,Eigen::Vector4d(1, 0, 0.5, 1.0),visited_points_pub_);
    
    std::cout<<"----------------timed astar finish 1"<<std::endl;
    if(success){
        double t_sample_step=ts;
        double local_time_horizon=2.0;
        //std::vector<Eigen::Vector2d> point_set, start_end_derivatives;
        Eigen::MatrixXd ctrl_pts;
        point_set=timed_astar_planner_->getTrajectory(t_sample_step,local_time_horizon);
   
        // if too few points, means near to goal, return false
        if(point_set.size()<3){
            return false;
        }
        //point_set=timed_astar_planner_->getPath();

        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(Eigen::Vector2d(0.0,0.0));
        start_end_derivatives.push_back(Eigen::Vector2d(0.0,0.0));
        start_end_derivatives.push_back(Eigen::Vector2d(0.0,0.0));
        
        //UniformBspline::parameterizeToBspline(t_sample_step, point_set, start_end_derivatives, ctrl_pts);
        std::cout<<"----------------timed astar finish 2"<<std::endl;
        return true;
    }else{
        std::cout<<"----------------timed astar finish 3"<<std::endl;
        return false;
    }
}




void TimedAstarSearch::getTriangleEdges(std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> &line_sets){
    using namespace delaunator;
    GraphPtr graph_t;
    timed_astar_planner_->getGraph(graph_t,0.0);
    line_sets.clear();
    for(size_t id=0;id<graph_t->triangles.size();id++){
        auto ai = 2 * graph_t->triangles[id];
        Eigen::Vector2d p1=Eigen::Vector2d( graph_t->coords[ai],graph_t->coords[ai+1]);

        ai = 2 * graph_t->triangles[nextHalfedge(id)];
        Eigen::Vector2d p2=Eigen::Vector2d( graph_t->coords[ai],graph_t->coords[ai+1]);

        ai = 2 * graph_t->triangles[prevHalfedge(id)];
        Eigen::Vector2d p3=Eigen::Vector2d( graph_t->coords[ai],graph_t->coords[ai+1]);

        std::pair<Eigen::Vector2d,Eigen::Vector2d> line1(p1,p2);
        std::pair<Eigen::Vector2d,Eigen::Vector2d> line2(p1,p3);
        std::pair<Eigen::Vector2d,Eigen::Vector2d> line3(p3,p2);
        line_sets.push_back(line1);
		line_sets.push_back(line2);
		line_sets.push_back(line3);
    }
}

// void TimedAstarSearch::getVisitedNodes(const std::vector<Eigen::Vector2d>& point_set){
//     timed_astar_planner_
// }

void TimedAstarSearch::visualizePoints(const std::vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub) {

//   for (auto &marker : target_marker_list_.markers) {
//         marker.action = visualization_msgs::Marker::DELETE;
//         marker.lifetime = ros::Duration(0.001); // just making sure! haha (probably overkill)
//   }
//   pub.publish(target_marker_list_);
//   target_marker_list_.markers.clear();

  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.lifetime = ros::Duration(0.001);
  mk.id              = 5;
  
  pub.publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = pt_size;
  mk.scale.y = pt_size;
  mk.scale.z = pt_size;
  mk.lifetime = ros::Duration();
  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < point_set.size(); i++) {
    pt.x = point_set[i](0);
    pt.y = point_set[i](1);
    pt.z = 0.0;
    mk.points.push_back(pt);
  }
  //target_marker_list_.markers.push_back(mk);
  pub.publish(mk);
   
  ros::Duration(0.001).sleep();

  
}

