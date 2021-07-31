#!/bin/bash
#get params
#MLP_ARENA2D
#column

trainmode=$1
agent_mode=$2

echo $1
echo $2
#roscd arena_local_planner_drl
python /root/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training/train_agent.py --agent $agent_mode

#recover the old IFS fully
#deactivate
#IFS=$OLDIFS
