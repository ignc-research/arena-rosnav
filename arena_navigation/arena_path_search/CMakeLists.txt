cmake_minimum_required(VERSION 3.0.2)
project(arena_path_search)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  arena_mapping
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


# this is important to find eigen path
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES arena_path_search
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs  arena_mapping
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
)


## Declare a C++ library
add_library(arena_path_search
  #src/astar.cpp
  src/kinodynamic_astar.cpp
  src/dyn_astar.cpp
  src/jps.cpp
  src/jps_utils.cpp
)

target_link_libraries(arena_path_search
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


# add_executable(triangle 
#               src/delaunator_triangle.cpp
# )

# target_link_libraries(triangle
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )


