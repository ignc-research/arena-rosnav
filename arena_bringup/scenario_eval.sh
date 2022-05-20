export PATH="$HOME/.poetry/bin:$PATH"
export PYTHONPATH=""
source /opt/ros/noetic/setup.zsh

#export PYTHONPATH=/home/ducanor/catkin_ws/src/arena-rosnav:/home/ducanor/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl:$PYTHONPATH
#source /home/ducanor/catkin_ws/devel/setup.sh

source /home/ducanor/arena_ws/devel/setup.sh
export PYTHONPATH=/home/ducanor/arena_ws/src/arena-rosnav:/home/ducanor/arena_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
#source /home/ducanor/arena_ws/devel/setup.sh

export WORKON_HOME=/home/ducanor/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh
export GAZEBO_PLUGIN_PATH=/home/ducanor/ActorCollisionsPlugin/build 
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

workon rosnav
# rep015
roslaunch arena_bringup start_arena_flatland.launch map_file:="map5" local_planner:="rosnav" scenario_file:="eval/map5_obs10.json" use_recorder:="true"
roslaunch arena_bringup start_arena_flatland.launch map_file:="map5" local_planner:="mpc" scenario_file:="eval/map5_obs10.json" use_recorder:="true"
roslaunch arena_bringup start_arena_flatland.launch map_file:="map5" local_planner:="dwa" scenario_file:="eval/map5_obs10.json" use_recorder:="true"
roslaunch arena_bringup start_arena_flatland.launch map_file:="map5" local_planner:="teb" scenario_file:="eval/map5_obs10.json" use_recorder:="true"
