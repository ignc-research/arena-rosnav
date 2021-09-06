# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "arena_mapping;arena_path_search;arena_traj_planner;arena_dynamic_channel;visualization_msgs;geometry_msgs;nav_msgs;roscpp;rospy;sensor_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-larena_timespace_planner".split(';') if "-larena_timespace_planner" != "" else []
PROJECT_NAME = "arena_timespace_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
