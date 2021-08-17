//
// Created by johannes on 14.06.21.
//

#ifndef ALL_IN_ONE_TEB_INTERFACE_LOCAL_COSTMAP_NODE_H
#define ALL_IN_ONE_TEB_INTERFACE_LOCAL_COSTMAP_NODE_H

#include <costmap_2d/costmap_2d_ros.h>
#include <string>
#include <all_in_one_teb_interface/ResetCostmap.h>
#include <costmap_2d/costmap_layer.h>

class LocalCostmapNode {
public:
    LocalCostmapNode();

    void initialize(tf2_ros::Buffer &tf, std::string &name);

    costmap_2d::Costmap2DROS *getCostmapRos();

    void clearCostmap(bool force_map_update);

    bool clearCostmap_service(all_in_one_teb_interface::ResetCostmap::Request &req,
                              all_in_one_teb_interface::ResetCostmap::Response &rep);

private:
    costmap_2d::Costmap2DROS* costmap_ros_;

    void clearMap(const boost::shared_ptr<costmap_2d::CostmapLayer>& costmap, double robot_pose_x, double robot_pose_y);
};


#endif //ALL_IN_ONE_TEB_INTERFACE_LOCAL_COSTMAP_NODE_H
