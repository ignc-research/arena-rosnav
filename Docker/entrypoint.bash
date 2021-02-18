#!/bin/bash

# In the ~/.bashrc
export WORKON_HOME=/root/.python_env
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /opt/ros/melodic/setup.bash
source /usr/local/bin/virtualenvwrapper.sh
source /root/catkin_ws/devel/setup.bash
export PYTHONPATH=//geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
export PYTHONPATH=/root/catkin_ws/src/arena-rosnav/:${PYTHONPATH}

JQ_EXEC=`which jq`
SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)

AGENT_NAME=$1
NUM_SIM_ENVS$2
CONFIG_FILE=$3
MAP_FOLDER=$4

echo "=======Here are the parameters you have given========"
echo "AGENT_NAME is : $AGENT_NAME"
echo "NUM_SIM_ENVS is : $NUM_SIM_ENVS"
echo "CONFIG_FILE is : $CONFIG_FILE"
echo "map_folder_name is : $MAP_FOLDER"
echo "===================================================="

(
    source "/root/.python_env/rosnav/bin/activate"
    roslaunch arena_bringup start_training.launch num_envs:=${NUM_SIM_ENVS} map_folder_name:=${MAP_FOLDER}
)&
# (
#     roslaunch arena_bringup visualization_training.launch ns:=sim_1
# )&
(
    sleep 2
    source "/root/.python_env/rosnav/bin/activate"
    roscd arena_local_planner_drl
    python scripts/training/train_agent.py --agent ${AGENT_NAME} --config ${CONFIG_FILE}
)