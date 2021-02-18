#!/bin/bash

# In the ~/.bashrc
export WORKON_HOME=/root/.python_env
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh
source /root/catkin_ws/devel/setup.bash
export PYTHONPATH=//geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
export PYTHONPATH=/root/catkin_ws/src/arena-rosnav/:${PYTHONPATH}

# AGENT_NAME=$1
# NUM_SIM_ENVS=$2
# CONFIGFILE=$1
# CONFIGFILE=config1.json

AGENT_NAME=DRL_LOCAL_PLANNER
NUM_SIM_ENVS=1
# CONFIGS= $(cat ./${CONFIGFILE})


(
    source "/root/.python_env/rosnav/bin/activate"
    roslaunch arena_bringup start_training.launch num_envs:=${NUM_SIM_ENVS}
)&
(
    sleep 1
    source "/root/.python_env/rosnav/bin/activate"
    roscd arena_local_planner_drl
    python scripts/training/train_agent.py --agent ${AGENT_NAME}
)