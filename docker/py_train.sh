 #!/bin/bash
params="$1"

echo ${params}|xargs python /root/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training/train_agent.py
