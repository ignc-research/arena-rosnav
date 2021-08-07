#! /bin/bash
params="$1"
echo "$params"

activate () {
  /root/.python_env/rosnav/bin/activate
}

activate
roslaunch arena_bringup start_arena_flatland.launch "$params"
