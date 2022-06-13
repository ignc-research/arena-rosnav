cmake_minimum_required(VERSION 3.0.2)
project(observations)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
			roscpp
			roslib
			rospy
			std_msgs
			sensor_msgs
			message_generation)

# find_package(SDL2 REQUIRED)
# find_package(OpenGL REQUIRED)
# find_package(OpenCV REQUIRED)

add_message_files(
	FILES
	Observation.msg
)

generate_messages(
	DEPENDENCIES
	sensor_msgs
	geometry_msgs
	std_msgs
)

catkin_package(
#  LIBRARIES arena_spacial_horizon
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
./
${SDL2_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${OPENGL_INCLUDE_DIRS}
)

# adding observation_packer
add_executable(observation_packer observation_packer/observation_packer.cpp common/zVector2d.cpp common/zVector2d.h common/f_math.cpp common/f_math.h)
add_dependencies(observation_packer observations_generate_messages_cpp)
target_link_libraries(observation_packer ${catkin_LIBRARIES})

# # adding ros agent
# catkin_install_python(PROGRAMS
# agents/agent_play.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

