#ifndef MAPPING_H
#define MAPPING_H

//
#include <Eigen/Eigen>
#include <iostream>
#include <ros/ros.h>

//data structure
#include <queue>
#include <vector>
#include <unordered_map>

//#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>

// tf
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// Laser & PCL
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>

// Odom & Pose
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

// Map
#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>

// message filter
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

// visual
#include <visualization_msgs/Marker.h>

// raycaster
#include <arena_mapping/raycast.h>

#define logit(x) (log((x) / (1 - (x))))  

struct MappingParameters {

  /* map properties */
  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector2i map_pixel_num_;                        // map range in index
  Eigen::Vector2i map_min_idx_, map_max_idx_;
  Eigen::Vector2d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  std::string frame_id_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;     // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                             // logit of occupancy probability
  double min_ray_length_, max_ray_length_;            // range of doing raycasting

  /* local map update and clear */
  double local_bound_inflate_;
  int local_map_margin_;

  /* visualization and computation time display */
  bool show_esdf_time_, show_occ_time_;

  /* active mapping */
  double unknown_flag_;

  /* flag for using esdf */
  bool use_occ_esdf_;
};

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance
  int buffer_size_;
  std::vector<double> occupancy_buffer_;
  std::vector<char>   occupancy_buffer_inflate_;
  std::vector<char>   occupancy_buffer_static_inflate_;
  std::vector<char>   occupancy_buffer_dynamic_inflate_;

  std::vector<char>   occupancy_buffer_neg_;             // for esdf
  std::vector<double> distance_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;
  std::vector<double> tmp_buffer1_;                     // for esdf
  std::vector<double> distance_buffer_static_all_;
  
  // laser  position and pose data
  Eigen::Vector2d laser_pos_, last_laser_pos_;
  Eigen::Quaterniond laser_q_, last_laser_q_;

  // depth pointCloud2 data
  pcl::PointCloud<pcl::PointXYZ> depth_cloud, last_depth_cloud;
  int depth_cloud_cnt_;

  // laser scan projected point cloud
  std::vector<Eigen::Vector2d> proj_points_;
  int proj_points_cnt;

  // flags of map state
  bool occ_need_update_, local_updated_, esdf_need_update_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;
  bool has_static_map_;

  //[new] odom_depth_timeout_
  ros::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_use_depth_fusion;

  // flag buffers for speeding up raycasting
  std::vector<short> count_hit_, count_hit_and_miss_;
  std::vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  std::queue<Eigen::Vector2i> cache_pixel_;

  // range of updating ESDF
  Eigen::Vector2i local_bound_min_, local_bound_max_;

  // computation time
  double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap{
    public:
        GridMap() {}
        ~GridMap() {}
        typedef std::shared_ptr<GridMap> Ptr;

        enum { INVALID_IDX = -10000 };

        void initMap(ros::NodeHandle& nh);

        /* occupancy map management */
        // static map 
        void get_static_buffer(std::vector<char> & static_buffer_inflate); 

        // reset buffer be used in scan_callback: occupancy_buffer_inflate_, distance_buffer_
        void resetBuffer();
        void resetBuffer(Eigen::Vector2d min, Eigen::Vector2d max);
        
        // pos[meter] to index [pixel cell]
        inline void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
        inline void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
        
        // index [pixel cell] to buffer array id [num]
        inline int toAddress(const Eigen::Vector2i& id);
        inline int toAddress(int& x, int& y);
        
        // is in map
        inline bool isInMap(const Eigen::Vector2d& pos);
        inline bool isInMap(const Eigen::Vector2i& idx);

        // occupancy manage
        inline void setOccupancy(Eigen::Vector2d pos, double occ = 1);
        inline void setOccupied(Eigen::Vector2d pos);
        inline int getOccupancy(Eigen::Vector2d pos);
        inline int getOccupancy(Eigen::Vector2i id);
        inline int getInflateOccupancy(Eigen::Vector2d pos);
        inline int getFusedInflateOccupancy(Eigen::Vector2d pos);         // fuse local and static map 
        
        inline int getFusedDynamicInflateOccupancy(Eigen::Vector2d pos);  // fuse local and static map and dynamic obs
        inline void setDynamicOccupancy(Eigen::Vector2d pos, int occ);
        void updateDynamicOccupancyMap(const std::vector<Eigen::Vector2d> &pos_set);


        // utils: bound index, known, unknown, free, occupied
        inline void boundIndex(Eigen::Vector2i& id);
        inline bool isUnknown(const Eigen::Vector2i& id);
        inline bool isUnknown(const Eigen::Vector2d& pos);
        inline bool isKnownFree(const Eigen::Vector2i& id);
        inline bool isKnownOccupied(const Eigen::Vector2i& id);

        /* distance field management */
        // get distance
        inline double getDistance(const Eigen::Vector2d& pos);
        inline double getDistance(const Eigen::Vector2i& id);
        inline double getDistanceStatic(const Eigen::Vector2d& pos);
        
        double evaluateCoarseEDT(Eigen::Vector2d& pos);
        
        // get distance gradient
        void evaluateEDTWithGrad(const Eigen::Vector2d& pos,double& dist, Eigen::Vector2d& grad);
        void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff);
        void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
        void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff,
                                                     double& value, Eigen::Vector2d& grad);
        
        /* utils map */
        void getRegion(Eigen::Vector2d& ori, Eigen::Vector2d& size);
        double getResolution();
        
        /* visualization publish */
        void publishMap();
        void publishStaticMap();
        void publishDynamicMap();
        void publishESDF();
        void publishStaticESDF();
        void publishDepth();
        void publishUpdateRange();
        void publishUnknown();


    private:
        ros::NodeHandle node_;
        MappingParameters mp_;
        MappingData md_;

        // scan to pointCloud2 projector
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        // sensor: message filter
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicyScanOdom;
        typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyScanOdom>> SynchronizerScanOdom;
        
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
        std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
        SynchronizerScanOdom sync_scan_odom_;

        // sensor: subscriber
        ros::Subscriber indep_scan_sub_, indep_odom_sub_;

        // map server service
        ros::ServiceClient static_map_client_;
        nav_msgs::OccupancyGrid static_map_;

        // publiser
        ros::Publisher map_pub_,static_map_pub_,dynamic_map_pub_;
        ros::Publisher esdf_pub_,esdf_static_pub_;
        ros::Publisher depth_pub_; //laser pointcloud2
        ros::Publisher update_range_pub_;
        ros::Publisher unknown_pub_;


        // timer
        ros::Timer occ_timer_;
        ros::Timer esdf_timer_;
        ros::Timer vis_timer_;

        
        /* Sensor Callbacks */
        void scanOdomCallback(const sensor_msgs::LaserScanConstPtr& scan, const nav_msgs::OdometryConstPtr& odom);
        void scanCallback(const sensor_msgs::LaserScanConstPtr& scan);
        void odomCallback(const nav_msgs::OdometryConstPtr& odom);

        /* Time event callback: update occupancy by raycasting, and update ESDF*/
        void updateOccupancyCallback(const ros::TimerEvent& /*event*/);
        void updateESDFCallback(const ros::TimerEvent& /*event*/);
        void visCallback(const ros::TimerEvent& /*event*/);  
        
        // main update process
        /* occupancy map update */
        void projectDepthCloud();
        void raycastProcess();
        void clearAndInflateLocalMap();

        inline void inflatePoint(const Eigen::Vector2i& pt, int step, std::vector<Eigen::Vector2i>& pts);
        int setCacheOccupancy(Eigen::Vector2d pos, int occ);
        Eigen::Vector2d closetPointInMap(const Eigen::Vector2d& pt, const Eigen::Vector2d& laser_pos);
        //void fuseOccupancyBuffer();
        // service static callback
        bool get_static_map();


        /* ESDF map update */
        void updateESDF2d();
        void updateESDF2d_static(std::vector<char> & occ_buffer_inflate, std::vector<double> &dist_buffer_all );

        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

        

};




inline void GridMap::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id) {
  for (int i = 0; i < 2; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos) {
  for (int i = 0; i < 2; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline int GridMap::toAddress(const Eigen::Vector2i& id) {
  return id(0) * mp_.map_pixel_num_(1) + id(1);
}

inline int GridMap::toAddress(int& x, int& y) {
  return x * mp_.map_pixel_num_(1)  + y;
}

inline bool GridMap::isInMap(const Eigen::Vector2d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ) {
    // cout << "less than min range!" << endl;
    return false;
  }

  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector2i& idx) {
  if (idx(0) < 0 || idx(1) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_pixel_num_(0) - 1 || idx(1) > mp_.map_pixel_num_(1) - 1 ) {
    return false;
  }
  return true;
}

inline void GridMap::setOccupied(Eigen::Vector2d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_pixel_num_(1) + id(1) ] = 1;
}

inline void GridMap::setOccupancy(Eigen::Vector2d pos, double occ) {
  if (occ != 1 && occ != 0) {
    std::cout << "occ value error!" << std::endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline void GridMap::setDynamicOccupancy(Eigen::Vector2d pos,int occ){
  if (occ != 1 && occ != 0) {
    std::cout << "occ value error!" << std::endl;
    return;
  }
  
  if (!isInMap(pos)) return;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_dynamic_inflate_[toAddress(id)] = occ;
}

inline int GridMap::getOccupancy(Eigen::Vector2d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector2i id;
  posToIndex(pos, id);
  
  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getOccupancy(Eigen::Vector2i id) {
  if (id(0) < 0 || id(0) >= mp_.map_pixel_num_(0) || id(1) < 0 || id(1) >= mp_.map_pixel_num_(1))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector2d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector2i id;
  posToIndex(pos, id);
  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int GridMap::getFusedInflateOccupancy(Eigen::Vector2d pos){
  if (!isInMap(pos)) return -1;

  Eigen::Vector2i id;
  posToIndex(pos, id);

  if(md_.has_static_map_ && md_.occupancy_buffer_static_inflate_[toAddress(id)]==1){
    return 1;
  }

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);

}

inline int GridMap::getFusedDynamicInflateOccupancy(Eigen::Vector2d pos){
    if (!isInMap(pos)) return -1;
    Eigen::Vector2i id;
    posToIndex(pos, id);

    if(md_.has_static_map_ && md_.occupancy_buffer_static_inflate_[toAddress(id)]==1){
      return 1;
    }

    if(md_.occupancy_buffer_dynamic_inflate_[toAddress(id)]==1){
      return 1;
    }

    return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}




inline void GridMap::boundIndex(Eigen::Vector2i& id) {
  Eigen::Vector2i id1;
  id1(0) = std::max(std::min(id(0), mp_.map_pixel_num_(0) - 1), 0);
  id1(1) = std::max(std::min(id(1), mp_.map_pixel_num_(1) - 1), 0);
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector2i& id) {
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector2d& pos) {
  Eigen::Vector2i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector2i& id) {
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector2i& id) {
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::inflatePoint(const Eigen::Vector2i& pt, int step, std::vector<Eigen::Vector2i>& pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y){
        pts[num++] = Eigen::Vector2i(pt(0) + x, pt(1) + y);
    }
}

/* DISTANCE FIELD*/
inline double GridMap::getDistance(const Eigen::Vector2d& pos) {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  boundIndex(id);

  return std::min(md_.distance_buffer_all_[toAddress(id)],md_.distance_buffer_static_all_[toAddress(id)]);
}

inline double GridMap::getDistance(const Eigen::Vector2i& id) {
  Eigen::Vector2i id1 = id;
  boundIndex(id1);
  return std::min(md_.distance_buffer_all_[toAddress(id)],md_.distance_buffer_static_all_[toAddress(id)]);
}

inline double GridMap::getDistanceStatic(const Eigen::Vector2d& pos) {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_static_all_[toAddress(id)];
}


#endif