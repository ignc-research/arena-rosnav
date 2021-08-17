//
// Created by johannes on 26.05.21.
//


#include <all_in_one_teb_interface/all_in_one_teb_interface.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <all_in_one_teb_interface/all_in_one_teb_node.h>
#include <base_local_planner/odometry_helper_ros.h>

AllInOneInterface::AllInOneInterface() :
        costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
        local_costmap_node_(),
        config_() {
}

void AllInOneInterface::initialize(teb_local_planner::TebConfig *config, LocalCostmapNode *local_costmap,
                                   ros::NodeHandle &nh) {
    AllInOneInterface::local_costmap_node_ = local_costmap;

    //load robot footprint model
    teb_local_planner::RobotFootprintModelPtr robot_model = AllInOneInterface::getRobotModel();

    //create visualizer
    visualizer = teb_local_planner::TebVisualizationPtr(new teb_local_planner::TebVisualization(nh, *config));

    // reserve some memory for obstacles
    obst_vector.reserve(500);

    AllInOneInterface::config_ = config;

    // create costmap (to polygon) converter - Note based on teb_local_planner_ros
    // https://github.com/rst-tu-dortmund/teb_local_planner/blob/melodic-devel/src/teb_local_planner_ros.cpp#L134
    if (!config->obstacles.costmap_converter_plugin.empty()) {
        try {
            costmap_converter_ = costmap_converter_loader_.createInstance(config->obstacles.costmap_converter_plugin);
            costmap_converter_->setOdomTopic(config->odom_topic);
            costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter"));
            costmap_converter_->setCostmap2D(local_costmap->getCostmapRos()->getCostmap());
            costmap_converter_->startWorker(ros::Rate(config->obstacles.costmap_converter_rate),
                                            local_costmap->getCostmapRos()->getCostmap(),
                                            config->obstacles.costmap_converter_spin_thread);
            ROS_INFO_STREAM("Costmap conversion plugin " << config->obstacles.costmap_converter_plugin << " loaded.");
        }
        catch (pluginlib::PluginlibException &ex) {
            ROS_WARN(
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s",
                    ex.what());
            costmap_converter_.reset();
        }
    } else
        ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treated as point obstacles.");

    //create planner
    if (config->hcp.enable_homotopy_class_planning)
        planner = teb_local_planner::PlannerInterfacePtr(
                new teb_local_planner::HomotopyClassPlanner(*config, &obst_vector, robot_model, visualizer));
    else
        planner = teb_local_planner::PlannerInterfacePtr(
                new teb_local_planner::TebOptimalPlanner(*config, &obst_vector, robot_model, visualizer));
    ROS_INFO("Setup teb interface node done!");
}

bool AllInOneInterface::service_callback(all_in_one_teb_interface::TebService::Request &req,
                                         all_in_one_teb_interface::TebService::Response &rep) {
    //ROS_INFO("Calculate new teb plan...");
    geometry_msgs::PoseStamped robot_pose;
    local_costmap_node_->getCostmapRos()->getRobotPose(robot_pose);
    robot_pose_ = teb_local_planner::PoseSE2(robot_pose.pose);
    teb_local_planner::PoseSE2 goal = teb_local_planner::PoseSE2(req.goal_pose.x,
                                                                 req.goal_pose.y,
                                                                 req.goal_pose.theta);

    return AllInOneInterface::calculateNewPlan(goal, &req.robot_velocity, rep);
}

bool
AllInOneInterface::calculateNewPlan(const teb_local_planner::PoseSE2 &goal,
                                    const geometry_msgs::Twist *start_vel,
                                    all_in_one_teb_interface::TebService::Response &rep) {
    // set new obstacles
    obst_vector.clear();
    if (costmap_converter_)
        updateObstacleContainerWithCostmapConverter();
    else
        updateObstacleContainerWithCostmap();
    //ROS_INFO("Extracted obstacles from costmap.");

    bool successful = planner->plan(robot_pose_, goal, start_vel, true);

    //ROS_INFO("New plan calculated!");

    // reset costmap if no feasible plan was found
    if (!successful) {
        AllInOneInterface::local_costmap_node_->clearCostmap(true);
        rep.costmaps_resetted = true;
    }


    double vx, vy, omega;

    if (successful) {
        planner->getVelocityCommand(vx, vy, omega, AllInOneInterface::config_->trajectory.control_look_ahead_poses);
    } else {
        planner->clearPlanner();
        vx = 0;
        vy = 0;
        omega = 0;
    }
    rep.vel = geometry_msgs::Twist();
    rep.vel.angular.z = omega;
    rep.vel.linear.x = vx;
    rep.vel.linear.y = vy;
    // ROS_INFO("Teb planning done!");
    planner->visualize();
    visualizer->publishObstacles(obst_vector);

    if (vx < 0.1 && successful) {
        local_costmap_node_->clearCostmap(true);
        rep.costmaps_resetted = true;
    } else if (successful) {
        rep.costmaps_resetted = false;
    }

    rep.successful = successful;

    return successful;
}

teb_local_planner::RobotFootprintModelPtr AllInOneInterface::getRobotModel() {
    // TODO load from .yaml file
    //hardcode myrobot parameters
    double radius = 0.3;
    return boost::make_shared<teb_local_planner::CircularRobotFootprint>(radius);
}

void AllInOneInterface::updateObstacleContainerWithCostmap() {
    // Note: Function is based on teb_local_planner_ros
    // (https://github.com/rst-tu-dortmund/teb_local_planner/blob/melodic-devel/src/teb_local_planner_ros.cpp#L474)

    // Add costmap obstacles if desired
    if (AllInOneInterface::config_->obstacles.include_costmap_obstacles) {
        costmap_2d::Costmap2D *costmap_ = AllInOneInterface::local_costmap_node_->getCostmapRos()->getCostmap();
        Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

        for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i) {
            for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j) {
                if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
                    Eigen::Vector2d obs;
                    costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                    // check if obstacle is interesting (e.g. not far behind the robot)
                    Eigen::Vector2d obs_dir = obs - robot_pose_.position();
                    if (obs_dir.dot(robot_orient) < 0 &&
                        obs_dir.norm() > config_->obstacles.costmap_obstacles_behind_robot_dist)
                        continue;

                    AllInOneInterface::obst_vector.push_back(
                            teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(obs)));
                }
            }
        }
    }
}

// Note: Based on teb_local_planner_ros
// https://github.com/rst-tu-dortmund/teb_local_planner/blob/melodic-devel/src/teb_local_planner_ros.cpp#L502
void AllInOneInterface::updateObstacleContainerWithCostmapConverter() {

    //Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
    if (!obstacles)
        return;

    for (const auto &i : obstacles->obstacles) {
        const costmap_converter::ObstacleMsg *obstacle = &i;
        const geometry_msgs::Polygon *polygon = &obstacle->polygon;

        if (polygon->points.size() == 1 && obstacle->radius > 0) // Circle
        {
            obst_vector.push_back(
                    teb_local_planner::ObstaclePtr(
                            new teb_local_planner::CircularObstacle(polygon->points[0].x, polygon->points[0].y,
                                                                    obstacle->radius)));
        } else if (polygon->points.size() == 1) // Point
        {
            obst_vector.push_back(teb_local_planner::ObstaclePtr(
                    new teb_local_planner::PointObstacle(polygon->points[0].x, polygon->points[0].y)));
        } else if (polygon->points.size() == 2) // Line
        {
            obst_vector.push_back(teb_local_planner::ObstaclePtr(
                    new teb_local_planner::LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y)));
        } else if (polygon->points.size() > 2) // Real polygon
        {
            auto *polyobst = new teb_local_planner::PolygonObstacle;
            for (auto point : polygon->points) {
                polyobst->pushBackVertex(point.x, point.y);
            }
            polyobst->finalizePolygon();
            obst_vector.emplace_back(polyobst);
        }

        // Set velocity, if obstacle is moving
        if (!obst_vector.empty())
            obst_vector.back()->setCentroidVelocity(i.velocities,
                                                    i.orientation);
    }
}
