#!/bin/bash
AGENT_NAME = $1
NUM_SIM_ENVS $2
echo "get info of $1"
echo "get info of $2"
source `which virtualenvwrapper.sh`

(
    workon rosnav
    roslaunch arena_bringup start_training.launch num_envs:=${NUM_SIM_ENVS}
)&
(
    workon rosnav
    roscd arena_local_planner_drl
    python scripts/training/train_agent.py --agent ${AGENT_NAME}
)