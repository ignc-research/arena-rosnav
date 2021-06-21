//
// Created by johannes on 26.05.21.
//


#include <all_in_one_teb_interface/all_in_one_teb_interface.h>
#include <teb_local_planner/homotopy_class_planner.h>


AllInOneInterface::AllInOneInterface(teb_local_planner::TebConfig *config, LocalCostmapNode *local_costmap) {
    AllInOneInterface::local_costmap = local_costmap;

    //load robot footprint model
    teb_local_planner::RobotFootprintModelPtr robot_model = AllInOneInterface::getRobotModel();

    // reserve some memory for obstacles
    obst_vector.reserve(500);

    AllInOneInterface::config = config;

    //create planner
    if (config->hcp.enable_homotopy_class_planning)
        planner = teb_local_planner::PlannerInterfacePtr(
                new teb_local_planner::HomotopyClassPlanner(*config, &obst_vector, robot_model));
    else
        planner = teb_local_planner::PlannerInterfacePtr(
                new teb_local_planner::TebOptimalPlanner(*config, &obst_vector, robot_model));
    ROS_INFO("Setup teb interface node done!");
}

bool AllInOneInterface::service_callback(all_in_one_teb_interface::TebService::Request &req,
                                         all_in_one_teb_interface::TebService::Response &rep) {
    //ROS_INFO("Calculate new teb plan...");
    geometry_msgs::PoseStamped robot_pose;
    AllInOneInterface::local_costmap->getCostmapRos()->getRobotPose(robot_pose);
    teb_local_planner::PoseSE2 robot_pose_ = teb_local_planner::PoseSE2(robot_pose.pose);
    teb_local_planner::PoseSE2 goal = teb_local_planner::PoseSE2(req.goal_pose.x,
                                                                 req.goal_pose.y,
                                                                 req.goal_pose.theta);
    //TODO convert global plan to correct frame (?)
    return AllInOneInterface::calculateNewPlan(robot_pose_, goal, &req.robot_velocity, rep);
}

bool
AllInOneInterface::calculateNewPlan(const teb_local_planner::PoseSE2 &start, const teb_local_planner::PoseSE2 &goal,
                                    const geometry_msgs::Twist *start_vel,
                                    all_in_one_teb_interface::TebService::Response &rep) {
    // set new obstacles
    obst_vector.clear();
    // AllInOneInterface::saveObstaclesFromOccupancyGrid(occ_grid);
    AllInOneInterface::updateObstacleContainerWithCostmap();
    //ROS_INFO("Extracted obstacles from costmap.");

    bool successful = planner->plan(start, goal, start_vel, true);

    //ROS_INFO("New plan calculated!");

    // reset costmap if no feasible plan was found
    if (!successful) {
        AllInOneInterface::local_costmap->clearCostmap();
    }
    rep.costmaps_resetted = successful;

    double vx, vy, omega;

    if (successful) {
        planner->getVelocityCommand(vx, vy, omega, AllInOneInterface::config->trajectory.control_look_ahead_poses);
    } else {
        vx = 0;
        vy = 0;
        omega = 0;
    }
    rep.vel = geometry_msgs::Twist();
    rep.vel.angular.z = omega;
    rep.vel.linear.x = vx;
    rep.vel.linear.y = vy;
    ROS_INFO("Teb planning done!");
    return successful;
}

teb_local_planner::RobotFootprintModelPtr AllInOneInterface::getRobotModel() {
    // TODO load from .yaml file
    //hardcode myrobot parameters
    double radius = 0.3;
    return boost::make_shared<teb_local_planner::CircularRobotFootprint>(radius);
}

void AllInOneInterface::updateObstacleContainerWithCostmap() {
    // Note: Function is taken from teb_local_planner
    // (https://github.com/rst-tu-dortmund/teb_local_planner/blob/melodic-devel/src/teb_local_planner_ros.cpp#L474)

    // Add costmap obstacles if desired
    if (AllInOneInterface::config->obstacles.include_costmap_obstacles) {
        costmap_2d::Costmap2D *costmap_ = AllInOneInterface::local_costmap->getCostmapRos()->getCostmap();


        for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i) {
            for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j) {
                if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
                    Eigen::Vector2d obs;
                    costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                    AllInOneInterface::obst_vector.push_back(
                            teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(obs)));
                }
            }
        }
    }
}

void AllInOneInterface::saveObstaclesFromOccupancyGrid(nav_msgs::OccupancyGrid &occ_grid) {
    //TODO transform to correct frame (?)
    double origin_x = occ_grid.info.origin.position.x;
    double origin_y = occ_grid.info.origin.position.y;
    double resolution = occ_grid.info.resolution;

    for (unsigned int i = 0; i < occ_grid.info.width - 1; ++i) {
        for (unsigned int j = 0; j < occ_grid.info.height - 1; ++j) {
            // lethal obstacles have value of 100
            // (https://github.com/ros-planning/navigation/blob/jade-devel/costmap_2d/src/costmap_2d_publisher.cpp#L63)
            if (occ_grid.data[j * occ_grid.info.width + i] > 99) {
                Eigen::Vector2d obs;
                //convert from map to world coordinates
                obs.coeffRef(0) = origin_x + (i + 0.5) * resolution;
                obs.coeffRef(1) = origin_y + (j + 0.5) * resolution;

                AllInOneInterface::obst_vector.push_back(
                        teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(obs)));
            }
        }
    }
}

void AllInOneInterface::getCustomObstacles(costmap_converter::ObstacleArrayMsg &custom_obstacle_msg_) {
    /*
    if (!custom_obstacle_msg_.obstacles.empty())
    {
        // We only use the global header to specify the obstacle coordinate system instead of individual ones
        Eigen::Affine3d obstacle_to_map_eig;
        try
        {
            geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
                                                                                    custom_obstacle_msg_.header.frame_id, ros::Time(0),
                                                                                    custom_obstacle_msg_.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));
            obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            obstacle_to_map_eig.setIdentity();
        }

        for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
        {


            // Set velocity, if obstacle is moving
            if(!obstacles_.empty())
                obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
        }
    }
    */
}
