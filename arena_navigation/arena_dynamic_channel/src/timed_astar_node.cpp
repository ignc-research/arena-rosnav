#include "arena_dynamic_channel/timed_astar_node.h"

using  std::cout; 
using  std::endl;

namespace timed_astar
{
TimeAstarSearch::~TimeAstarSearch()
{
	
    cout<<"DELETE"<<endl;

}

void TimeAstarSearch::init(ros::NodeHandle & nh){
	/* init obstacle provider nodes */
	node_=nh;
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    std::string str1="obs_dynamic";//"odom_walker";//

	// init vector size
	obs_info_provider_.reserve(100);

	// add obs_provider_node to vector
    for (ros::master::V_TopicInfo::iterator it = topic_infos.begin() ; it != topic_infos.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;
        
        if (info.name.find(str1) != std::string::npos) 
        {
            std::cout << "topic_" << it - topic_infos.begin() << ": " << info.name << std::endl;
			obs_info_provider_.emplace_back(std::make_shared<ObstacleInfo>(node_,info.name));
        }

    }

	/* init delaunay triangulation */
	//dt_graph_.reset(new Graph);

	/* init param */
	robot_avg_vel_=1.0; 		//avg_vel of robot m/s
	robot_max_vel_=3.0; 		//avg_vel of robot m/s
	radius_robot_=0.5;
	radius_obs_=0.5;
	have_odom_=false;

	/* map */
	grid_map_.reset(new GridMap);
  	grid_map_->initMap(node_);

    // bspline optimizer
    //bspline_optimizer_.reset(new BsplineOptimizer);
    //bspline_optimizer_->setParam(node_);
    //bspline_optimizer_->setEnvironment(grid_map_);
 
    Eigen::Vector2d occ_map_origin,occ_map_size_2d;
	grid_map_->getRegion(occ_map_origin, occ_map_size_2d);
    
    occ_map_origin_=Vec2d(occ_map_origin(0),occ_map_origin(1));//Vec2d(-6.0,-6.0);//
    occ_map_size_2d_=Vec2d(occ_map_size_2d(0),occ_map_size_2d(1));//Vec2d(23.0,26.0);//

	
	resolution_=0.1;								// 0.1 meter/cell
	inv_resolution_=1.0/resolution_;

	time_resolution_=0.1;  							// 0.1 sec
	inv_time_resolution_ = 1.0 / time_resolution_;

    /* init timed astar planner */

    timed_astar_planner_.reset(new TimedAstar);

    timed_astar_planner_->init(grid_map_);


	/* ros communication */
	ros::NodeHandle public_nh;

  	// subscriber
  	goal_sub_ =public_nh.subscribe("goal", 1, &TimeAstarSearch::goalCallback,this);
  	odom_sub_ = public_nh.subscribe("odom", 1, &TimeAstarSearch::odomCallback, this);

	// publisher
	vis_triangle_pub_= public_nh.advertise<visualization_msgs::Marker>("vis_triangle", 20);
	vis_goal_pub_ =	public_nh.advertise<visualization_msgs::Marker>("vis_goal", 20);
    vis_wp_pub_ =   public_nh.advertise<visualization_msgs::Marker>("vis_wps_timed_astar", 20);
    vis_path_pub_= public_nh.advertise<nav_msgs::Path>("vis_path_timed_astar", 20);

	/* init time event timer */
	update_timer_=node_.createTimer(ros::Duration(0.5), &TimeAstarSearch::UpdateCallback, this);  // shouldn't use different ros::NodeHandle inside the callback as timer's node handler

	
}

void TimeAstarSearch::odomCallback(const nav_msgs::OdometryConstPtr& msg){

    odom_pos_=Vec2d(msg->pose.pose.position.x,msg->pose.pose.position.y);
    odom_vel_=Vec2d(msg->twist.twist.linear.x,msg->twist.twist.linear.y);

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    auto euler = odom_orient_.toRotationMatrix().eulerAngles(0, 1, 2); // row,pitch,yaw
    odom_dir_=euler(2); 
    //cout<<"odom direction"<<odom_dir_*180/3.1415<<endl;
    have_odom_ = true;
}

void TimeAstarSearch::goalCallback(const geometry_msgs::PoseStampedPtr& msg){
  if(have_odom_==false) return;

  // end pt
  end_pt_=Vec2d(msg->pose.position.x,msg->pose.position.y);

  have_goal_=true;
  // vis goal
  std::cout << "Goal set!" << std::endl;
  std::vector<Vec2d> point_set;
  point_set.push_back(end_pt_);
  visualizePoints(point_set,0.5,Eigen::Vector4d(1, 1, 1, 1.0),vis_goal_pub_);

}

void TimeAstarSearch::resetGraph(){
	/* init a new Fade2D ptr */
    coords_.clear();
    speeds_.clear();
    angles_.clear();
    coords_.reserve(100*2);
    speeds_.reserve(100*2);
    angles_.reserve(100*2);

    // start & end
    start_pt_=odom_pos_;
	start_vel_=odom_vel_;
    start_dir_ =odom_dir_;

    // boundary
	//double local_bound_time=100.0; // 3 sec
	//Vec2d corner_min= occ_map_origin_;//start_pt_-Vec2d(1.0,1.0)*robot_avg_vel_*local_bound_time;
	//Vec2d corner_max=occ_map_size_2d_;//start_pt_+Vec2d(1.0,1.0)*robot_avg_vel_*local_bound_time;

    double local_bound_time=5.0; // 3 sec
	Vec2d corner_min=start_pt_-Vec2d(1.0,1.0)*robot_avg_vel_*local_bound_time;
	Vec2d corner_max=start_pt_+Vec2d(1.0,1.0)*robot_avg_vel_*local_bound_time;
    boundPosition(corner_min);
	boundPosition(corner_max);

    // INIT INDEX
    coords_.push_back(start_pt_.x);
    coords_.push_back(start_pt_.y);
    speeds_.push_back(start_vel_.x);
    speeds_.push_back(start_vel_.y);

    // GOAL_INDEX
    coords_.push_back(end_pt_.x);
    coords_.push_back(end_pt_.y);
    speeds_.push_back(0.0);
    speeds_.push_back(0.0);

    // PHASE1_INDEX
    coords_.push_back(corner_max.x);
    coords_.push_back(corner_max.y);
    speeds_.push_back(0.0);
    speeds_.push_back(0.0);

    // PHASE4_INDEX
    coords_.push_back(corner_min.x);
    coords_.push_back(corner_max.y);
    speeds_.push_back(0.0);
    speeds_.push_back(0.0);

    // PHASE3_INDEX
    coords_.push_back(corner_min.x);
    coords_.push_back(corner_min.y);
    speeds_.push_back(0.0);
    speeds_.push_back(0.0);

    // PHASE2_INDEX
    coords_.push_back(corner_max.x);
    coords_.push_back(corner_min.y);
    speeds_.push_back(0.0);
    speeds_.push_back(0.0);


	// obstacles
	for (std::vector<ObstacleInfo::Ptr>::iterator it = obs_info_provider_.begin() ; it != obs_info_provider_.end(); it++)
    {   
        const ObstacleInfo::Ptr & obs_info = *it;
		// calculate arriving time according to relative velocity
		Vec2d obs_pos=obs_info->getPosition();
		Vec2d obs_vel=obs_info->getVelocity();
		
        /* 
        // vec_Robot2Obstacle
        Vec2d dist=obs_pos-start_pt_;  	
		dist.x=dist.x==0.0?0.0001:dist.x;
		dist.y=dist.y==0.0?0.0001:dist.y;
		double obs_vel_project=-(dot(obs_vel,dist)/dist.length());

		double arriving_time=dist.length()/(robot_avg_vel_+obs_vel_project); 	// assume robot_vel target at the obstacle pos at t=t0
		arriving_time=std::max(arriving_time,0.0);							// if arriving_time<0, means can collid immediately

		if(arriving_time>3.0)
		{
			continue;
		} 
        */
       
        coords_.push_back(obs_pos.x);
        coords_.push_back(obs_pos.y);
        speeds_.push_back(obs_vel.x);
        speeds_.push_back(obs_vel.y);
        //angle_t.push_back(angles[i]);
        //angle_t.push_back(angles[i+1]);

	}

	//std::cout << "a---------------------------------" << std::endl;
	//std::cout<<"size="<<obs_state_set_.size()<<std::endl;
    //printf("%d\n", obs_state_set_.back().use_count());
	//std::cout<<"debug54---------------------"<<std::endl;

	/* reset dt_ */
    dt_graph_.reset(new Graph(coords_,speeds_,angles_));


	//std::cout<<"debug55---------------------"<<std::endl;
}

void TimeAstarSearch::StateTimeAstarSearch(){
    
    bool success;
    cout<<"bebug search1---------------"<<endl;
    if(have_goal_){
        cout<<"goal:"<<end_pt_.x<<" , "<<end_pt_.y<<endl;
        //success=StateTimeAstar(coords_,speeds_,angles_,start_pt_,end_pt_,waypoints_);

        success=timed_astar_planner_->TimeAstarSearch(coords_,speeds_,angles_,start_pt_,end_pt_ ,start_dir_,0.0);

        cout<<"bebug search2---------------"<<endl;
        if(success){
            //cout<<"waypoints---------------"<<endl;
            waypoints_.clear();
            waypoints_=timed_astar_planner_->getPath();
            cout<<"bebug search3---------------"<<endl;
            for(size_t i=0;i<waypoints_.size();++i)
            {
                //cout<<waypoints_[i].x<<", "<<waypoints_[i].y<<endl;

                
            }
            cout<<"num of wps:"<<waypoints_.size()<<endl;
            visualizePoints(waypoints_,0.5,Eigen::Vector4d(1, 0, 1, 1.0),vis_wp_pub_);
            visualizePath(waypoints_,vis_path_pub_);
        
        

        }
    }
    //cout<<"bebug search2---------------"<<endl;

}

void TimeAstarSearch::UpdateCallback(const ros::TimerEvent&){
    if(have_goal_)
    {
	    ros::WallTime t1, t2;
  	    t1 = ros::WallTime::now();
	
	
	    resetGraph();
        StateTimeAstarSearch();

	    t2 = ros::WallTime::now();
	    /* record time */
	    dc_time_ += (t2 - t1).toSec();
	    max_dc_time_ = std::max(max_dc_time_, (t2 - t1).toSec());
  	    dc_update_num_++;
	
	    bool show_time=true;
  	    if (show_time)
	    {
		    ROS_WARN("DC: cur t = %lf, avg t = %lf, max t = %lf", 
				(t2 - t1).toSec(),
             	dc_time_/ dc_update_num_,
				max_dc_time_);
	    }
    
    

        if((odom_pos_-end_pt_).length()<0.5){
            have_goal_=false;
        }

        /* publish Graph */
        publishVisGraph();
	
    }


}


/* --------------------visualization---------------------------- */
void TimeAstarSearch::visualizePoints(const std::vector<Vec2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub) {
  
  visualization_msgs::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = {};//ros::Time::now();
  mk.type            = visualization_msgs::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  //mk.id              = id;

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

  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < point_set.size(); i++) {
    pt.x = point_set[i].x;
    pt.y = point_set[i].y;
    pt.z = 0.0;
    mk.points.push_back(pt);
  }
  pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void TimeAstarSearch::visualizeLines(const std::vector<std::pair<Vec2d,Vec2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub){
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = pt_size;
    line_list.color.r = color(0);
    line_list.color.g = color(1);
    line_list.color.b = color(2);
    line_list.color.a = color(3);

    geometry_msgs::Point pt1,pt2;
    for (unsigned int i = 0; i < ptr_pair_sets.size(); i++) {
        pt1.x=ptr_pair_sets[i].first.x;
        pt1.y=ptr_pair_sets[i].first.y;
        pt1.z=0.0;

        pt2.x=ptr_pair_sets[i].second.x;
        pt2.y=ptr_pair_sets[i].second.y;
        pt2.z=0.0;

        line_list.points.push_back(pt1);
        line_list.points.push_back(pt2);
    }

    pub.publish(line_list);
    //ROS_INFO("vis once");
}

void TimeAstarSearch::visualizePath(const std::vector<Vec2d> path, const ros::Publisher & pub){

  //create a path message
  ros::Time plan_time = {};//ros::Time::now();
  std::string global_frame="/map";
  
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = global_frame;
  gui_path.header.stamp = plan_time;
  
  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for(unsigned int i=0; i < path.size(); i++){
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame;
      pose.pose.position.x = path[i].x;    //world_x;
      pose.pose.position.y = path[i].y;    //world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      gui_path.poses[i]=pose;               //plan.push_back(pose);
  }
  pub.publish(gui_path);
}

void TimeAstarSearch::publishVisGraph(){

	std::vector<std::pair<Vec2d,Vec2d>> line_sets;

    using namespace delaunator;
    GraphPtr graph_t;
    timed_astar_planner_->getGraph(graph_t,0.0);

    for(size_t id=0;id<graph_t->triangles.size();id++){
        auto ai = 2 * graph_t->triangles[id];
        Vec2d p1=Vec2d( graph_t->coords[ai],graph_t->coords[ai+1]);

        ai = 2 * graph_t->triangles[nextHalfedge(id)];
        Vec2d p2=Vec2d( graph_t->coords[ai],graph_t->coords[ai+1]);

        ai = 2 * graph_t->triangles[prevHalfedge(id)];
        Vec2d p3=Vec2d( graph_t->coords[ai],graph_t->coords[ai+1]);

        std::pair<Vec2d,Vec2d> line1(p1,p2);
        std::pair<Vec2d,Vec2d> line2(p1,p3);
        std::pair<Vec2d,Vec2d> line3(p3,p2);
        line_sets.push_back(line1);
		line_sets.push_back(line2);
		line_sets.push_back(line3);
    }
    visualizeLines(line_sets,0.2,Eigen::Vector4d(0, 1, 1, 1.0),vis_triangle_pub_);

    /* 
    for (let e = 0; e < delaunay.triangles.length; e++) {
        if (e > delaunay.halfedges[e]) {
            const p = points[delaunay.triangles[e]];
            const q = points[delaunay.triangles[nextHalfedge(e)]];
            callback(e, p, q);
        }
    }

	std::vector<Triangle2*> vAllDelaunayTriangles;
	dt_->getTrianglePointers(vAllDelaunayTriangles);
	for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it)
	{
		Triangle2* pT(*it);
		// An alternative (just to show how to access the vertices) would be:
		Point2* p0=pT->getCorner(0);
		Point2* p1=pT->getCorner(1);
		Point2* p2=pT->getCorner(2);

		std::pair<Eigen::Vector2d,Eigen::Vector2d> line1(Eigen::Vector2d(p0->x(),p0->y()),Eigen::Vector2d(p1->x(),p1->y()));
		std::pair<Eigen::Vector2d,Eigen::Vector2d> line2(Eigen::Vector2d(p0->x(),p0->y()),Eigen::Vector2d(p2->x(),p2->y()));
		std::pair<Eigen::Vector2d,Eigen::Vector2d> line3(Eigen::Vector2d(p2->x(),p2->y()),Eigen::Vector2d(p1->x(),p1->y()));
		line_sets.push_back(line1);
		line_sets.push_back(line2);
		line_sets.push_back(line3);
	} 

	visualizeLines(line_sets,0.2,Eigen::Vector4d(0, 1, 1, 1.0),vis_triangle_pub_); */
}

}

int main(int argc, char** argv)
{
    using namespace timed_astar;
	ros::init(argc, argv, "obstacle2");
    ros::NodeHandle nh("");
    TimeAstarSearch::Ptr timed_astar;

    timed_astar.reset(new TimeAstarSearch);
    timed_astar->init(nh);
	ros::spin();
}