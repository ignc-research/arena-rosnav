#include "arena_mapping/mapping.h"

/*init*/
void GridMap::initMap(ros::NodeHandle& nh){
    node_ = nh;

    /* get parameters*/
    std::string static_map_service_name = "/static_map";  
    ros::service::waitForService(static_map_service_name); // important
    static_map_client_= nh.serviceClient<nav_msgs::GetMap>(static_map_service_name);  
    bool is_static_map_avail=get_static_map();
    if (!is_static_map_avail){ROS_ERROR("No static map available, please check map_server.");}
    
    // check if use esdf
    node_.param("sdf_map/use_occ_esdf", mp_.use_occ_esdf_, true);
    
    // local map size
    node_.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), 5.5);
    node_.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), 5.5);

    // occupancy map
    node_.param("sdf_map/p_hit", mp_.p_hit_, 0.55);
    node_.param("sdf_map/p_miss", mp_.p_miss_, 0.2);
    node_.param("sdf_map/p_min", mp_.p_min_, 0.12);
    node_.param("sdf_map/p_max", mp_.p_max_, 0.80);
    node_.param("sdf_map/p_occ", mp_.p_occ_, 0.70);

    // laser ray length
    node_.param("sdf_map/min_ray_length", mp_.min_ray_length_, 0.5);
    node_.param("sdf_map/max_ray_length", mp_.max_ray_length_, 3.5);

    //show time
    node_.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);
    node_.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);

    // local map
    node_.param("sdf_map/frame_id", mp_.frame_id_, std::string("map"));
    node_.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, 0.01);
    node_.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 0.0);
    node_.param("sdf_map/local_map_margin", mp_.local_map_margin_, 50);

    /* map size*/
    // resolution
    mp_.resolution_=static_map_.info.resolution;
    mp_.resolution_inv_ = 1 / mp_.resolution_;

    // size in meter
    double x_size, y_size, x_origin, y_origin;
    x_size=static_map_.info.width*static_map_.info.resolution;                // width in meter
    y_size=static_map_.info.height*static_map_.info.resolution;               // height in meter
    mp_.map_size_ = Eigen::Vector2d(x_size, y_size);                          // in meter

    x_origin=static_map_.info.origin.position.x;                              // coordinate of left-bottom corner of the map, in meter
    y_origin=static_map_.info.origin.position.y;                              // coordinate of left-bottom corner of the map, in meter
    mp_.map_origin_ = Eigen::Vector2d(x_origin, y_origin);                    // left-bottom corner  w.r.t coordinate origin
    
   
    // size in pixel of global map
    //for (int i = 0; i < 2; ++i) mp_.map_pixel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
    mp_.map_pixel_num_(0)=static_map_.info.width;                             // pixel num: width
    mp_.map_pixel_num_(1)=static_map_.info.height;                            // pixel num: height

    // global map boundary in pixel/index
    mp_.map_min_idx_ = Eigen::Vector2i::Zero();                               // min pixel idx  (x_pixel_min,y_pixel_min)
    mp_.map_max_idx_ = mp_.map_pixel_num_ - Eigen::Vector2i::Ones();          // max pixel idx  (x_pixel_max,y_pixel_max)

    // global map boundary in meter
    mp_.map_min_boundary_ = mp_.map_origin_;                                  // map boundary (x_min,y_min) in meter
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;                  // map boundary (x_max,y_max) in meter

    // local bound inflate
    mp_.local_bound_inflate_ = std::max(mp_.resolution_, mp_.local_bound_inflate_);  // esdf inflation 
    
    // occupancy map probability param                    // odd(s+)=odd(s-)+prob_hit_log_ or odd(s+)=odd(s-)+likelihood
    mp_.prob_hit_log_ = logit(mp_.p_hit_);                // log likelihood log[p(z=hit|s=occ)/p(m=hit|s=free)]
    mp_.prob_miss_log_ =logit(mp_.p_miss_);               // log likelihood log[p(z=miss|s=occ)/p(m=miss|s=free)]
    mp_.clamp_min_log_ = logit(mp_.p_min_);               // log min state prob  log[p(s=occ)/p(s=free)]
    mp_.clamp_max_log_ = logit(mp_.p_max_);               // log max state prob  log[p(s=occ)/p(s=free)]
    mp_.min_occupancy_log_ = logit(mp_.p_occ_);           // log of occupancy determined prob
    mp_.unknown_flag_ = 0.01;

    /* map data param */
    md_.occ_need_update_ = false;           // scan_odom_callback
    md_.local_updated_ = false;             // raycast process
    md_.esdf_need_update_ = false;          // scan callback, occupancy updated 
    //md_.has_first_depth_ = false;
    md_.has_odom_ = false;                  // odom callback [not much use]
    md_.has_cloud_ = false;                 // scan callback [no use]
    md_.depth_cloud_cnt_ = 0;               // no use
    md_.has_static_map_=false;
    
    md_.esdf_time_ = 0.0;
    md_.fuse_time_ = 0.0;
    md_.update_num_ = 0;
    md_.max_esdf_time_ = 0.0;
    md_.max_fuse_time_ = 0.0;

    /* initialize data buffers*/
    // global map buffer size
    md_.buffer_size_ = mp_.map_pixel_num_(0) * mp_.map_pixel_num_(1) ;           // buffer size

    // global occupancy map buffer
    md_.occupancy_buffer_ = std::vector<double>(md_.buffer_size_, mp_.clamp_min_log_ - mp_.unknown_flag_); // save state_occu probability [mp_.clamp_min_log_,mp_.clamp_max_log_]
    md_.occupancy_buffer_neg_ = std::vector<char>(md_.buffer_size_, 0);
    md_.occupancy_buffer_inflate_ = std::vector<char>(md_.buffer_size_, 0);                                // save is_occ {0,1}, 0 free, 1 occ 
    md_.occupancy_buffer_static_inflate_=std::vector<char>(md_.buffer_size_, 0); // static map buffer
    md_.occupancy_buffer_dynamic_inflate_=std::vector<char>(md_.buffer_size_, 0);
    

    // global distance map buffer
    md_.distance_buffer_      = std::vector<double>(md_.buffer_size_, 10000);
    md_.distance_buffer_neg_  = std::vector<double>(md_.buffer_size_, 10000);
    md_.distance_buffer_all_  = std::vector<double>(md_.buffer_size_, 10000);
    md_.tmp_buffer1_          = std::vector<double>(md_.buffer_size_, 0);
    md_.distance_buffer_static_all_=std::vector<double>(md_.buffer_size_, 10000);

    // init static occ & ESDF buffers
    get_static_buffer(md_.occupancy_buffer_static_inflate_);
    updateESDF2d_static(md_.occupancy_buffer_static_inflate_,md_.distance_buffer_static_all_);

    // global occ log probabilty buffer
    md_.count_hit_and_miss_ = std::vector<short>(md_.buffer_size_, 0);
    md_.count_hit_          = std::vector<short>(md_.buffer_size_, 0);
    md_.flag_rayend_        = std::vector<char>(md_.buffer_size_, -1);
    md_.flag_traverse_      = std::vector<char>(md_.buffer_size_, -1);
    
    
    /* some counter */
    md_.raycast_num_ = 0;                   // just count how many raycast is done [not much use]
    md_.proj_points_cnt = 0;                // raycast is triggered only proj_points_cnt>0


    
    /* show map param */
    std::cout << "use_occ_esdf: " << mp_.use_occ_esdf_<<std::endl; 
    std::cout << "x_size: " <<  x_size << std::endl;
    std::cout << "y_size: " <<  y_size << std::endl;
    std::cout << "x_origin: " << x_origin << std::endl;
    std::cout << "y_origin: " << y_origin << std::endl;
    
    std::cout << "x_size_pix: " << mp_.map_pixel_num_(0) << std::endl;
    std::cout << "y_size_pix: " << mp_.map_pixel_num_(1) << std::endl;
    std::cout << "x_map_max_idx: " << mp_.map_max_idx_(0) << std::endl;
    std::cout << "y_map_max_idx: " << mp_.map_max_idx_(1) << std::endl;

    std::cout << "x_local_update_range_: " << mp_.local_update_range_(0) << std::endl;
    std::cout << "y_local_update_range_: " << mp_.local_update_range_(1) << std::endl;
    std::cout << "hit: " << mp_.prob_hit_log_ << std::endl;
    std::cout << "miss: " << mp_.prob_miss_log_ << std::endl;
    std::cout << "min log: " << mp_.clamp_min_log_ << std::endl;
    std::cout << "max: " << mp_.clamp_max_log_ << std::endl;
    std::cout << "thresh log: " << mp_.min_occupancy_log_ << std::endl;
    /*
    rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
    rand_noise2_ = normal_distribution<double>(0, 0.2);
    random_device rd;
    eng_ = default_random_engine(rd());
    */

    ros::NodeHandle public_nh("");
    // sensor sub: syn message filter
    scan_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(public_nh, "scan", 50));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(public_nh, "odometry/ground_truth", 100));
    sync_scan_odom_.reset(new message_filters::Synchronizer<SyncPolicyScanOdom>(SyncPolicyScanOdom(100), *scan_sub_, *odom_sub_));
    sync_scan_odom_->registerCallback(boost::bind(&GridMap::scanOdomCallback, this, _1, _2));

    // sensor sub
    indep_scan_sub_ =public_nh.subscribe<sensor_msgs::LaserScan>("scan", 10, &GridMap::scanCallback, this);
    indep_odom_sub_ =public_nh.subscribe<nav_msgs::Odometry>("odometry/ground_truth", 10, &GridMap::odomCallback, this);

    // timer callbacks
    occ_timer_ = public_nh.createTimer(ros::Duration(0.05),   &GridMap::updateOccupancyCallback, this); // raycasting & setCacheOccupancy is the key
    if (mp_.use_occ_esdf_){
      esdf_timer_ = public_nh.createTimer(ros::Duration(0.05), &GridMap::updateESDFCallback, this);
    }
    vis_timer_ = public_nh.createTimer(ros::Duration(0.05), &GridMap::visCallback, this);
    
    // publishers 
    map_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("sdf_map/occupancy", 10);
    static_map_pub_= public_nh.advertise<sensor_msgs::PointCloud2>("sdf_map/occupancy_static", 10);
    dynamic_map_pub_= public_nh.advertise<sensor_msgs::PointCloud2>("sdf_map/occupancy_dynamic", 10);
    
    //map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
    
    esdf_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("sdf_map/esdf", 10);
    esdf_static_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("sdf_map/esdf_static", 10);
    update_range_pub_ = public_nh.advertise<visualization_msgs::Marker>("sdf_map/update_range", 10);
    unknown_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("sdf_map/unknown", 10);
    depth_pub_ = public_nh.advertise<sensor_msgs::PointCloud2>("sdf_map/depth_cloud", 10);

}

/* static map */
bool GridMap::get_static_map(){
    nav_msgs::GetMap srv;
    srv.request={}; //std_srvs::Empty::Request& request
    
    if (static_map_client_.call(srv))
    {   
      ROS_INFO_STREAM("Is stacic map available: "<<"True");
      // set global_plan
      static_map_=srv.response.map;

      
      return true;
    }
    else{
      ROS_INFO_STREAM("Is stacic map available: "<<"False");

      return false;
    }
}

void GridMap::get_static_buffer(std::vector<char> & static_buffer_inflate){
 
  int idx_nav_occ;
  int data;
  double value;

  md_.has_static_map_=true;    
  /* inflate the point */
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  
  for (int id_x = 0; id_x < mp_.map_pixel_num_(0); id_x++)
      for (int id_y = 0; id_y < mp_.map_pixel_num_(1); id_y++){
        // [*] addr in nav_msg::OccupancyGrid.data
        idx_nav_occ=id_x+id_y*mp_.map_pixel_num_(0);// cell_col_num+ cell_row_num*rows
        
        // [*] cast data from int8 to int[important]
        data=(int)static_map_.data[idx_nav_occ];

        // Check data not be 0 or 100
        if(data!=0 && data!=100){
          //std::cout<<"data probability="<<value<<std::endl;
        }
        value=double(data)/100.0;

        // save data to buffer
        if(value>mp_.p_occ_){
          // [*] addr in buffer
          Eigen::Vector2i idx;
          Eigen::Vector2d idx_pos;

          idx(0)=id_x;idx(1)=id_y;
          indexToPos(idx, idx_pos);
          //idx_inf=toAddress(idx);
          //md_.occupancy_buffer_static_inflate_[idx_inf]=1;

          /* inflate the point */
          Eigen::Vector2i inf_pt;
          Eigen::Vector2d p2d_inf;
          
          // Determine local occupandcy boundary for current location
          double max_x, max_y, min_x, min_y;

          min_x = mp_.map_max_boundary_(0);
          min_y = mp_.map_max_boundary_(1);

          max_x = mp_.map_min_boundary_(0);
          max_y = mp_.map_min_boundary_(1);

          for (int x = -inf_step; x <= inf_step; ++x)
            for (int y = -inf_step; y <= inf_step; ++y){
              
              p2d_inf(0) = idx_pos(0) + x * mp_.resolution_;
              p2d_inf(1) = idx_pos(1) + y * mp_.resolution_;
              
              max_x = std::max(max_x, p2d_inf(0));
              max_y = std::max(max_y, p2d_inf(1));
              
              min_x = std::min(min_x, p2d_inf(0));
              min_y = std::min(min_y, p2d_inf(1));
              
              posToIndex(p2d_inf, inf_pt);

              if (!isInMap(inf_pt)) continue;

              int idx_inf = toAddress(inf_pt);
              
              static_buffer_inflate[idx_inf] = 1;
            }
        }
      
      }   

     
}

void GridMap::updateDynamicOccupancyMap(const std::vector<Eigen::Vector2d> &pos_set){
  // reset dynamic buffer
  for (int id_x = 0; id_x < mp_.map_pixel_num_(0); id_x++)
      for (int id_y = 0; id_y < mp_.map_pixel_num_(1); id_y++)
      {
        md_.occupancy_buffer_dynamic_inflate_[toAddress(id_x, id_y)] = 0;
      }
  // set occupancy
  for(size_t i=0;i<pos_set.size();i++)
  {
    setDynamicOccupancy(pos_set[i],0);
  }
}

void GridMap::updateESDF2d_static(std::vector<char> & occ_buffer_inflate, std::vector<double> &dist_buffer_all ) {
  Eigen::Vector2i min_esdf=mp_.map_min_idx_;
  Eigen::Vector2i max_esdf=mp_.map_max_idx_;

  std::vector<char>   occ_buffer_neg=std::vector<char>(md_.buffer_size_, 0);
  std::vector<double>   tmp_buffer=std::vector<double>(md_.buffer_size_, 0);
  std::vector<double> dist_buffer=std::vector<double>(md_.buffer_size_, 10000);
  std::vector<double> dist_buffer_neg=std::vector<double>(md_.buffer_size_, 10000);


  /* ========== compute positive DT ========== */
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    fillESDF( [&](int y) { return occ_buffer_inflate[toAddress(x, y)] == 1 ?0 :std::numeric_limits<double>::max(); },
              [&](int y, double val) { tmp_buffer[toAddress(x, y)] = val; }, 
              min_esdf[1],
              max_esdf[1], 
              1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF([&](int x) { return tmp_buffer[toAddress(x, y)]; },
               [&](int x, double val) {  dist_buffer[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);},
               min_esdf[0], 
               max_esdf[0], 
               0);
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
        int idx = toAddress(x, y);
        if (occ_buffer_inflate[idx] == 0) {
          occ_buffer_neg[idx] = 1;

        } else if (occ_buffer_inflate[idx] == 1) {
          occ_buffer_neg[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
    }


  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      fillESDF([&](int y) { return occ_buffer_neg[x * mp_.map_pixel_num_(1) +y] == 1 ?0 :std::numeric_limits<double>::max(); },
               [&](int y, double val) { tmp_buffer[toAddress(x, y)] = val; }, 
               min_esdf[1],
               max_esdf[1], 
               1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF([&](int x) { return tmp_buffer[toAddress(x, y)]; },
               [&](int x, double val) {dist_buffer_neg[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);},
               min_esdf[0], 
               max_esdf[0], 
               0);
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y){

        int idx = toAddress(x, y);
        dist_buffer_all[idx] = dist_buffer[idx];

        if (dist_buffer_neg[idx] > 0.0)
          dist_buffer_all[idx] += (-dist_buffer_neg[idx] + mp_.resolution_);
    }
}

/* sensor callback */
void GridMap::scanOdomCallback(const sensor_msgs::LaserScanConstPtr& scan, const nav_msgs::OdometryConstPtr& odom) {
  /* get pose */
  md_.laser_pos_(0)=odom->pose.pose.position.x;
  md_.laser_pos_(1) = odom->pose.pose.position.y;
  md_.laser_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  
  /* preprocess laser scan */
  sensor_msgs::LaserScan laser_scan=*scan;
  //change range=nan to range=range_max, otherwise projection will negelect directions with range=nan
  for(unsigned int j=0;j< laser_scan.ranges.size();j++){
      if(std::isnan(laser_scan.ranges[j])){
          laser_scan.ranges[j]=laser_scan.range_max;
      }
  } 
  
  /* get depth pointCloud */
  sensor_msgs::PointCloud2 cloud;
  // wait for transform from robot_base to laser scan
  if(!tfListener_.waitForTransform(
        laser_scan.header.frame_id,
        mp_.frame_id_,
        laser_scan.header.stamp + ros::Duration().fromSec(laser_scan.ranges.size()*laser_scan.time_increment),
        ros::Duration(1.0))){return;}
     //"/base_footprint",

  // High fidelity projection: suitable for laser that is moving
  projector_.transformLaserScanToPointCloud(mp_.frame_id_, laser_scan, cloud, tfListener_);  //projector_.projectLaser(laser_scan, cloud);

  // ros_cloud to pcl_cloud
  pcl::fromROSMsg(cloud, md_.depth_cloud);//pcl::PointCloud<pcl::PointXYZ> rawCloud;
  
  /* 
  if(md_.has_cloud_==false){
      ROS_INFO_STREAM("scan:");
    for(unsigned int i=0; i<md_.depth_cloud.points.size();i++){
      ROS_INFO_STREAM("scan:"<<i);
      ROS_INFO_STREAM("PLC"<<md_.depth_cloud.points[i]);
      ROS_INFO_STREAM("laser:"<<laser_scan.ranges[i]);
    }
    int cnt=0;
    for(int j=0;j< laser_scan.ranges.size();j++){
      if(std::isnan(laser_scan.ranges[j])){
          cnt++;
      }
    }
    ROS_INFO_STREAM("laser ranges size:"<<laser_scan.ranges.size());
    ROS_INFO_STREAM("laser ranges non size:"<<cnt);
    md_.has_cloud_=true;
  }
  */
  
  // occ flag
  md_.occ_need_update_ = true;  // occ only update when both odom(or tf) & point cloud is ready
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {

  md_.laser_pos_(0)=odom->pose.pose.position.x;
  md_.laser_pos_(1) = odom->pose.pose.position.y;
  md_.laser_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  md_.has_odom_ = true;
}

void GridMap::scanCallback(const sensor_msgs::LaserScanConstPtr& scan) {
  
  /* scan to pcl cloud -----------------------------------*/
  // read scan to pcl cloud
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  sensor_msgs::PointCloud2 scan_cloud;

  /* preprocess laser scan */
  sensor_msgs::LaserScan laser_scan=*scan;
  //change range=nan to range=range_max, otherwise projection will negelect directions with range=nan
  for(unsigned int j=0;j< laser_scan.ranges.size();j++){
      if(std::isnan(laser_scan.ranges[j])){
          laser_scan.ranges[j]=laser_scan.range_max;
      }
  } 

  // wait for transform from robot_base to laser scan
  if(!tfListener_.waitForTransform(
        laser_scan.header.frame_id,
        "/map",
        laser_scan.header.stamp + ros::Duration().fromSec(laser_scan.ranges.size()*laser_scan.time_increment),
        ros::Duration(1.0))){return;}


  // High fidelity projection: suitable for laser that is moving
  projector_.transformLaserScanToPointCloud(mp_.frame_id_, laser_scan, scan_cloud, tfListener_);

  
  //projector_.projectLaser(*scan, scan_cloud);
  pcl::fromROSMsg(scan_cloud, latest_cloud);
  md_.has_cloud_ = true;
  
  /* reset buffer: occupancy_buffer_inflate_, distance_buffer_ -----------------------------------*/
  // check odom available
  if (!md_.has_odom_) {
    // std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0) return;

  if (isnan(md_.laser_pos_(0)) || isnan(md_.laser_pos_(1))) return;

  // reset the occupancy buffer
  this->resetBuffer(md_.laser_pos_ - mp_.local_update_range_,
                    md_.laser_pos_ + mp_.local_update_range_);

  // inflation step
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);

  // Determine local occupandcy boundary for current location
  double max_x, max_y, min_x, min_y;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);


  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  

  pcl::PointXY pt;
  Eigen::Vector2d p2d, p2d_inf;

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    pt.x = latest_cloud.points[i].x;
    pt.y = latest_cloud.points[i].y;
    p2d(0) = pt.x, p2d(1) = pt.y;

    /* point inside update range */
    Eigen::Vector2d devi = p2d - md_.laser_pos_;
    Eigen::Vector2i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1)) {

      /* inflate the point */
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y){

            p2d_inf(0) = pt.x + x * mp_.resolution_;
            p2d_inf(1) = pt.y + y * mp_.resolution_;
            
            max_x = std::max(max_x, p2d_inf(0));
            max_y = std::max(max_y, p2d_inf(1));
            
            min_x = std::min(min_x, p2d_inf(0));
            min_y = std::min(min_y, p2d_inf(1));
            
            posToIndex(p2d_inf, inf_pt);

            if (!isInMap(inf_pt)) continue;

            int idx_inf = toAddress(inf_pt);

            // add point_inf into occupancy_buffer_inflate_
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  // make sure min_xy < laser_pos_, max_xy>laser_pos_
  min_x = std::min(min_x, md_.laser_pos_(0));
  min_y = std::min(min_y, md_.laser_pos_(1));
  
  max_x = std::max(max_x, md_.laser_pos_(0));
  max_y = std::max(max_y, md_.laser_pos_(1));
  

  posToIndex(Eigen::Vector2d(max_x, max_y), md_.local_bound_max_);
  posToIndex(Eigen::Vector2d(min_x, min_y), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.esdf_need_update_ = true;
  
}

void GridMap::resetBuffer() {
  // reset all map grid occ value in occupancy_buffer_inflate_ & distance_buffer_
  Eigen::Vector2d min_pos = mp_.map_min_boundary_;
  Eigen::Vector2d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector2i::Zero();
  md_.local_bound_max_ = mp_.map_pixel_num_ - Eigen::Vector2i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos) {

  Eigen::Vector2i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y){
        md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
        md_.distance_buffer_[toAddress(x, y)] = 10000;
      }
}

/*Time event callback: occupancy update*/
void GridMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {

  if (!md_.occ_need_update_) {
    //std::cout<<"return"<<std::endl; 
    return;
  }

  /* update occupancy */
  ros::WallTime t1, t2;
  t1 = ros::WallTime::now();

  projectDepthCloud();
  raycastProcess();
  
  // raycast is done, local_updated_ will be setted true
  if (md_.local_updated_) clearAndInflateLocalMap();
  
  t2 = ros::WallTime::now();

  md_.fuse_time_ += (t2 - t1).toSec();
  md_.max_fuse_time_ =std:: max(md_.max_fuse_time_, (t2 - t1).toSec());
  
  if (mp_.show_occ_time_)
    ROS_WARN("Occupancy update: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  if (md_.local_updated_) md_.esdf_need_update_ = true;
  md_.local_updated_ = false;

}

void GridMap::projectDepthCloud() {
  md_.proj_points_cnt = 0;
  int num_pts=md_.depth_cloud.points.size();
  md_.proj_points_=std::vector<Eigen::Vector2d>(num_pts);
  
  //Eigen::Matrix3d laser_r = md_.laser_q_.toRotationMatrix();


  for(int i=0;i<num_pts;i++){

    Eigen::Vector2d proj_pt;
    proj_pt(0)=md_.depth_cloud.points[i].x;
    proj_pt(1)=md_.depth_cloud.points[i].y;

    md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
  }
  
  /* maintain laser pose for consistency check */
  md_.last_laser_pos_ = md_.laser_pos_;
  md_.last_laser_q_ = md_.laser_q_;
  md_.last_depth_cloud = md_.depth_cloud;
}

void GridMap::raycastProcess() {
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0) return;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int pix_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  
  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  

  RayCaster raycaster;
  Eigen::Vector2d half = Eigen::Vector2d(0.5, 0.5);
  Eigen::Vector2d ray_pt, pt_w;

  
  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];
    

    // set flag for projected point
    if (!isInMap(pt_w)) {// if not in map, select the nearest point in map & set occ=0
      pt_w = closetPointInMap(pt_w, md_.laser_pos_);

      length = (pt_w - md_.laser_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w = (pt_w - md_.laser_pos_) / length * mp_.max_ray_length_ + md_.laser_pos_;
      }
      pix_idx = setCacheOccupancy(pt_w, 0);

    } else {// if in map
      length = (pt_w - md_.laser_pos_).norm();

      if (length > mp_.max_ray_length_) {// if in map, but exceed max_ray_length, set occ=0
        pt_w = (pt_w - md_.laser_pos_) / length * mp_.max_ray_length_ + md_.laser_pos_;
        pix_idx = setCacheOccupancy(pt_w, 0);
      } else {                            // if in map, and inside max_ray_length, set occ=1
        pix_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = std::max(max_x, pt_w(0));
    max_y = std::max(max_y, pt_w(1));

    min_x = std::min(min_x, pt_w(0));
    min_y = std::min(min_y, pt_w(1));


    // raycasting between camera center and point

    if (pix_idx != INVALID_IDX) {
      if (md_.flag_rayend_[pix_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[pix_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.laser_pos_ / mp_.resolution_);


    while (raycaster.step(ray_pt)) {
      Eigen::Vector2d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.laser_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      pix_idx = setCacheOccupancy(tmp, 0);

      if (pix_idx != INVALID_IDX) {
        if (md_.flag_traverse_[pix_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[pix_idx] = md_.raycast_num_;
        }
      }
    }
  }

  // determine the local bounding box for updating ESDF
  min_x = std::min(min_x, md_.laser_pos_(0));
  min_y = std::min(min_y, md_.laser_pos_(1));
  

  max_x = std::max(max_x, md_.laser_pos_(0));
  max_y = std::max(max_y, md_.laser_pos_(1));


  posToIndex(Eigen::Vector2d(max_x, max_y), md_.local_bound_max_);
  posToIndex(Eigen::Vector2d(min_x, min_y), md_.local_bound_min_);

  int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  md_.local_bound_max_ += esdf_inf * Eigen::Vector2i(1, 1);
  md_.local_bound_min_ -= esdf_inf * Eigen::Vector2i(1, 1);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector2d local_range_min = md_.laser_pos_ - mp_.local_update_range_;
  Eigen::Vector2d local_range_max = md_.laser_pos_ + mp_.local_update_range_;

  Eigen::Vector2i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;
  /*update cells in md_.cache_pixel's probability in occupancy_buffer */
  while (!md_.cache_pixel_.empty()) {
    Eigen::Vector2i idx = md_.cache_pixel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_pixel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
        mp_.prob_hit_log_ :
        mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;
 

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) && idx(1) <= max_id(1);
    if (!in_local) {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector2d GridMap::closetPointInMap(const Eigen::Vector2d& pt, const Eigen::Vector2d& laser_pos) {
  Eigen::Vector2d diff = pt - laser_pos;
  Eigen::Vector2d max_tc = mp_.map_max_boundary_ - laser_pos;
  Eigen::Vector2d min_tc = mp_.map_min_boundary_ - laser_pos;

  double min_t = 1000000;

  for (int i = 0; i < 2; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }

  return laser_pos + (min_t - 1e-3) * diff;
}

int GridMap::setCacheOccupancy(Eigen::Vector2d pos, int occ) {
  if (occ != 1 && occ != 0) return INVALID_IDX;

  Eigen::Vector2i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  if(occ==0){
    md_.count_hit_and_miss_[idx_ctns] += 1;
  }else{
    md_.count_hit_and_miss_[idx_ctns] += 1;
  }
  

  if (md_.count_hit_and_miss_[idx_ctns] ==2) {// 1, // occ probability update for hit less often 
    md_.cache_pixel_.push(id);
  }else if(occ==0){                           // occ probability update for every miss
    md_.cache_pixel_.push(id);
  }



  if (occ == 1) {
    md_.count_hit_[idx_ctns] += 1;
  }
  
  return idx_ctns;
}

void GridMap::clearAndInflateLocalMap() {
  /*clear outside local*/
  const int vec_margin = 500;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector2i min_cut = md_.local_bound_min_ - Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector2i max_cut = md_.local_bound_max_ + Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector2i min_cut_m = min_cut - Eigen::Vector2i(vec_margin, vec_margin);
  Eigen::Vector2i max_cut_m = max_cut + Eigen::Vector2i(vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);


  /* clear data outside the local range ( with margin vec_margin) */
  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {
    for (int y = min_cut_m(1); y < min_cut(1); ++y) {
      int idx = toAddress(x, y);
      md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      md_.distance_buffer_all_[idx] = 10000;
    }

    for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
      int idx = toAddress(x, y);
      md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      md_.distance_buffer_all_[idx] = 10000;
    }
  }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
  {
      for (int x = min_cut_m(0); x < min_cut(0); ++x) {
        int idx = toAddress(x, y);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
        int idx = toAddress(x, y);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
  }

  /* inflated occupied pixels inside the local range */
  // inflate occupied voxels to compensate robot size
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  std::vector<Eigen::Vector2i> inf_pts(pow(2 * inf_step + 1, 2));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector2i inf_pt;

  // clear outdated data inside local bound (indexs)
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y){
        md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y){
        if (md_.occupancy_buffer_[toAddress(x, y)] > mp_.min_occupancy_log_) {
          inflatePoint(Eigen::Vector2i(x, y), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k) {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_pixel_num_(0) * mp_.map_pixel_num_(1)) {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
    }

}

/* Fuse Occuancy buffer
void GridMap::fuseOccupancyBuffer(){
  if(!md_.has_static_map_) return;

  Eigen::Vector2i min_cut = md_.local_bound_min_;
  Eigen::Vector2i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector2i(lmm, lmm);
  max_cut += Eigen::Vector2i(lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y){
      if(md_.occupancy_buffer_static_inflate_[toAddress(x, y)]==1){
        md_.occupancy_buffer_inflate_[toAddress(x, y)] = 1;
      }
    }

  // Eigen::Vector2d p2d;
  // Eigen::Vector2i pt_idx;

  // Eigen::Vector2d min_local =md_.laser_pos_ - mp_.local_update_range_;
  // Eigen::Vector2d max_local = md_.laser_pos_ + mp_.local_update_range_;

  // for (int x = min_local(0); x <= max_local(0); ++x)
  //   for (int y = min_local(1); y <= max_local(1); ++y){ 
  //       p2d(0)=x;p2d(1)=y;
  //       posToIndex(p2d, pt_idx);

  //       if (!isInMap(pt_idx)) continue;

  //       int pt_addr = toAddress(pt_idx);

  //       // add point_inf into occupancy_buffer_inflate_
  //       if(md_.occupancy_buffer_static_inflate_[pt_addr]==1){
  //         md_.occupancy_buffer_inflate_[pt_addr] = 1;
  //       }
  //   }
}
*/

/*Time event callback: ESDF update*/
void GridMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  // this is important otherwise will be neicun error
  if (!md_.esdf_need_update_) return;  // because of dynamic env, we need to update more often

  /* esdf */
  ros::WallTime t1, t2;
  t1 = ros::WallTime::now();

  updateESDF2d();

  t2 = ros::WallTime::now();

  md_.esdf_time_ += (t2 - t1).toSec();
  md_.max_esdf_time_ = std::max(md_.max_esdf_time_, (t2 - t1).toSec());
  md_.update_num_++;
  if (mp_.show_esdf_time_)
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);
  
  md_.esdf_need_update_ = false;
  /*test*/
  
  //int8_t a=static_map_.data[10];
  //ROS_INFO_STREAM("vec="<<(int)a);
  
  

}

template <typename F_get_val, typename F_set_val>
void GridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_pixel_num_(dim)];
  double z[mp_.map_pixel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void GridMap::updateESDF2d() {
  Eigen::Vector2i min_esdf = md_.local_bound_min_;
  Eigen::Vector2i max_esdf = md_.local_bound_max_;

  /* ========== compute positive DT ========== */
  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
    fillESDF( [&](int y) { return md_.occupancy_buffer_inflate_[toAddress(x, y)] == 1 ?0 :std::numeric_limits<double>::max(); },
              [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; }, 
              min_esdf[1],
              max_esdf[1], 
              1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF([&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
               [&](int x, double val) {  md_.distance_buffer_[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);
                 //  min(mp_.resolution_ * std::sqrt(val), md_.distance_buffer_[toAddress(x, y, z)]);
               },
               min_esdf[0], 
               max_esdf[0], 
               0);
  }

  /* ========== compute negative distance ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
        int idx = toAddress(x, y);
        if (md_.occupancy_buffer_inflate_[idx] == 0) {
          md_.occupancy_buffer_neg_[idx] = 1;

        } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
          md_.occupancy_buffer_neg_[idx] = 0;
        } else {
          ROS_ERROR("what?");
        }
    }

  ros::Time t1, t2;

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      fillESDF([&](int y) { return md_.occupancy_buffer_neg_[x * mp_.map_pixel_num_(1) +y] == 1 ?0 :std::numeric_limits<double>::max(); },
               [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; }, 
               min_esdf[1],
               max_esdf[1], 
               1);
  }

  for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      fillESDF([&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
               [&](int x, double val) {md_.distance_buffer_neg_[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);},
               min_esdf[0], 
               max_esdf[0], 
               0);
  }

  /* ========== combine pos and neg DT ========== */
  for (int x = min_esdf(0); x <= max_esdf(0); ++x)
    for (int y = min_esdf(1); y <= max_esdf(1); ++y){

        int idx = toAddress(x, y);
        md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

        if (md_.distance_buffer_neg_[idx] > 0.0)
          md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
    }
}

/* DISTANCE FIELD MANAGEMENT*/
double GridMap::evaluateCoarseEDT(Eigen::Vector2d& pos) {
  double d1 = getDistance(pos);
  return d1;
}

void GridMap::getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2],
                            Eigen::Vector2d& diff) {
  if (!isInMap(pos)) {
    // cout << "pos invalid for interpolation." << endl;
  }

  /* interpolation position */
  Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
  Eigen::Vector2i idx;
  Eigen::Vector2d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_; // (p-p0)/ (p1-p0) 

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
        Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
        Eigen::Vector2d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y] = current_pos;
    }
  }
}

void GridMap::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
        dists[x][y] = getDistance(pts[x][y]);
    }
  }
}

void GridMap::interpolateBilinear(double values[2][2],const Eigen::Vector2d& diff,double& value,Eigen::Vector2d& grad) {
  
  // bilinear interpolation
  double v00=values[0][0];  //f(x0,y0)
  double v10=values[1][0];  //f(x1,y0)
  double v01=values[0][1];  //f(x0,y1)
  double v11=values[1][1];  //f(x1,y1)

  double v0=(1-diff(0))*v00+diff(0)*v10;  
  double v1=(1-diff(0))*v01+diff(0)*v11;
  value = (1-diff(1))*v0 + diff(1)*v1;
  
  // calculate gradient
  grad[1]= (v1-v0) * mp_.resolution_inv_;
  grad[0]= ((1 - diff[1]) * (v10 - v00) + diff[1] * (v11 - v01)) * mp_.resolution_inv_;

}

void GridMap::evaluateEDTWithGrad(  const Eigen::Vector2d& pos,double& dist,Eigen::Vector2d& grad) {
    // get diff & surround pts
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getSurroundPts(pos, sur_pts, diff);

    // get distances of the surround pts
    double dists[2][2];
    getSurroundDistance(sur_pts, dists);

    // do interpolate to get distance gradient
    interpolateBilinear(dists, diff, dist, grad);
    
}


/* Map utils */
void GridMap::getRegion(Eigen::Vector2d& ori, Eigen::Vector2d& size) {
  ori = mp_.map_origin_, size = mp_.map_size_;
}

double GridMap::getResolution() { return mp_.resolution_; }


/* Visualization Publishers*/
void GridMap::publishMap() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector2i min_cut = md_.local_bound_min_;
  Eigen::Vector2i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector2i(lmm, lmm);
  max_cut += Eigen::Vector2i(lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y){ 
        //if (md_.occupancy_buffer_[toAddress(x, y)] == 0) continue;
        if (md_.occupancy_buffer_inflate_[toAddress(x, y)] == 0) continue;

        Eigen::Vector2d pos;
        indexToPos(Eigen::Vector2i(x, y), pos);
        
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = 1; // z=0
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;

  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
  
}

void GridMap::publishStaticMap(){
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int x = 0; x <= mp_.map_pixel_num_(0); ++x)
      for (int y = 0; y <= mp_.map_pixel_num_(1); ++y){ 
        //if (md_.occupancy_buffer_[toAddress(x, y)] == 0) continue;
        int idx_inf=toAddress(x,y);
        if (md_.occupancy_buffer_static_inflate_[idx_inf] == 0) continue;

        Eigen::Vector2d pos;
        indexToPos(Eigen::Vector2i(x, y), pos);
        
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = 1; // z=0
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;

  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  static_map_pub_.publish(cloud_msg);
}

void GridMap::publishDynamicMap(){
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int x = 0; x <= mp_.map_pixel_num_(0); ++x)
      for (int y = 0; y <= mp_.map_pixel_num_(1); ++y){ 
        //if (md_.occupancy_buffer_[toAddress(x, y)] == 0) continue;
        int idx_inf=toAddress(x,y);
        if (md_.occupancy_buffer_dynamic_inflate_[idx_inf] == 0) continue;

        Eigen::Vector2d pos;
        indexToPos(Eigen::Vector2i(x, y), pos);
        
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = 1; // z=0
        cloud.push_back(pt);
      }

    cloud.width = cloud.points.size();
    cloud.height = 1;

    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    dynamic_map_pub_.publish(cloud_msg);
}

void GridMap::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector2i min_cut = md_.local_bound_min_ - Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector2i max_cut = md_.local_bound_max_ + Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector2d pos;
      indexToPos(Eigen::Vector2i(x, y), pos);
      //pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = std::min(dist, max_dist);
      dist = std::max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.0;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void GridMap::publishStaticESDF(){
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector2i min_cut = mp_.map_min_idx_; //md_.local_bound_min_ - Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector2i max_cut = mp_.map_max_idx_;//md_.local_bound_max_ + Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {

      Eigen::Vector2d pos;
      indexToPos(Eigen::Vector2i(x, y), pos);
      //pos(2) = mp_.esdf_slice_height_;

      dist = getDistanceStatic(pos);
      dist = std::min(dist, max_dist);
      dist = std::max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.0;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_static_pub_.publish(cloud_msg);

}

void GridMap::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i][0];
    pt.y = md_.proj_points_[i][1];
    pt.z = 0;
    cloud.push_back(pt);
  }

  cloud.width = md_.last_depth_cloud.width;
  cloud.height = md_.last_depth_cloud.height;
  
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
  
}

void GridMap::publishUpdateRange() {
  Eigen::Vector2d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  indexToPos(md_.local_bound_min_, esdf_min_pos);
  indexToPos(md_.local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = mp_.frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = 0.0;

  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = 1.0;

  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void GridMap::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector2i min_cut = md_.local_bound_min_;
  Eigen::Vector2i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y){
        if (md_.occupancy_buffer_[toAddress(x, y)] < mp_.clamp_min_log_ - 1e-3) {
          Eigen::Vector2d pos;
          indexToPos(Eigen::Vector2i(x, y), pos);
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = 0.0;
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  // auto sz = max_cut - min_cut;
  // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
  //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void GridMap::visCallback(const ros::TimerEvent& /*event*/) {
  publishMap();
  publishStaticMap();
  publishDynamicMap();
  //publishMapInflate(false);
  if(mp_.use_occ_esdf_){
    publishESDF();
    publishStaticESDF();
  }
  //publishDepth();
  //publishUnknown();
  //publishUpdateRange();
  //std::cout<<"VisualCallback:"<<std::endl;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "sdf_map");
    std::cout<<"start"<<std::endl;
    ros::NodeHandle nh("~");
    //GridMap filter();

    GridMap::Ptr grid_map_;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);
    ros::spin();
    return 0;
}




























