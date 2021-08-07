#! /bin/bash
params="$1"
echo "$params"
python --verson
activate () {
  /root/.python_env/rosnav/bin/activate
}

activate
python --version
roslaunch arena_bringup start_arena_flatland.launch "$params"
