cmake_minimum_required(VERSION 3.0.2)
project(arena_intermediate_planner)

## Compile as C++14, supported in ROS Noetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  
  visualization_msgs
  plan_visualization

  arena_mapping
  arena_path_search
  arena_traj_planner
  arena_plan_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arena_intermediate_planner
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs plan_visualization arena_mapping arena_path_search arena_traj_planner arena_plan_msgs
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(arena_intermediate_planner 
              src/intermediate_planner.cpp
)

target_link_libraries(arena_intermediate_planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
