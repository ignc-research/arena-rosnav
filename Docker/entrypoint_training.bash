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

CONFIG_FILE=$1
# CONFIG_FILE="config1.json"

AGENT_NAME=$(cat $SHELL_FOLDER/configs/${CONFIG_FILE} | ${JQ_EXEC} .input.AGENT_NAME | sed 's/\"//g')
NUM_SIM_ENVS=$(cat $SHELL_FOLDER/configs/${CONFIG_FILE} | ${JQ_EXEC} .input.NUM_SIM_ENVS)

robot=$(cat $SHELL_FOLDER/configs/${CONFIG_FILE} | ${JQ_EXEC} .parameters.robot)
gamma=$(cat $SHELL_FOLDER/configs/${CONFIG_FILE} | ${JQ_EXEC} .parameters.gamma)

echo "========================="
echo "AGENT_NAME is : $AGENT_NAME"
echo "NUM_SIM_ENVS is : $NUM_SIM_ENVS"
echo "========================="
echo "robot is : $robot"
echo "gamma is : $gamma"


(
    source "/root/.python_env/rosnav/bin/activate"
    roslaunch arena_bringup start_training.launch num_envs:=${NUM_SIM_ENVS}
)&
(
    sleep 2
    source "/root/.python_env/rosnav/bin/activate"
    roscd arena_local_planner_drl
    python scripts/training/train_agent.py --agent ${AGENT_NAME}
)