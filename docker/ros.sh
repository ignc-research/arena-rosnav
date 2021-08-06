#! /bin/sh
params = $1
echo $params
roslaunch arena_bringup start_arena_flatland.launch $params
