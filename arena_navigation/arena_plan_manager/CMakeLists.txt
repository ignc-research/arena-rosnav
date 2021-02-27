cmake_minimum_required(VERSION 3.0.2)
project(arena_plan_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros

  arena_plan_msgs
  arena_fake_localization
  arena_intermediate_planner
  arena_mapping
  arena_path_search
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arena_plan_manager
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs arena_plan_msgs arena_intermediate_planner arena_mapping arena_path_search
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  #include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# this is important to find eigen path
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


add_executable(plan_manager_node
    #src/plan_manager_node.cpp 
    src/plan_manager.cpp 
  )





add_dependencies(plan_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(plan_manager_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

