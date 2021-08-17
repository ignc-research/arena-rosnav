//
// Created by johannes on 10.06.21.
//

#ifndef TEB_LOCAL_PLANNER_ALL_IN_ONE_TEB_NODE_H
#define TEB_LOCAL_PLANNER_ALL_IN_ONE_TEB_NODE_H


#include <teb_local_planner/teb_config.h>
#include "local_costmap_node.h"
#include "all_in_one_teb_interface.h"

class AllInOneTebNode {
public:
    AllInOneTebNode(ros::NodeHandle &nh, tf2_ros::Buffer &tfBuffer);

private:
    teb_local_planner::TebConfig cfg_;
    AllInOneInterface *teb_interface;
    LocalCostmapNode *local_costmap;
    ros::ServiceServer getVelSrv_;
    ros::ServiceServer resetCostmapSrv_;
};


#endif //TEB_LOCAL_PLANNER_ALL_IN_ONE_TEB_NODE_H
