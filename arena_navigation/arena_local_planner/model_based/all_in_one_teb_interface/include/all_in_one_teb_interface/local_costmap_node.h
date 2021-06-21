//
// Created by johannes on 14.06.21.
//

#ifndef ALL_IN_ONE_TEB_INTERFACE_LOCAL_COSTMAP_NODE_H
#define ALL_IN_ONE_TEB_INTERFACE_LOCAL_COSTMAP_NODE_H

#include <costmap_2d/costmap_2d_ros.h>
#include <string>

class LocalCostmapNode {
public:
    LocalCostmapNode(tf2_ros::Buffer &tf, std::string &name);

    costmap_2d::Costmap2DROS *getCostmapRos() const;

    void clearCostmap();

private:
    costmap_2d::Costmap2DROS *costmap_ros_;
};


#endif //ALL_IN_ONE_TEB_INTERFACE_LOCAL_COSTMAP_NODE_H
