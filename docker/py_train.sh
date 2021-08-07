 #!/bin/bash

params="$1"

activate () {
  /root/.python_env/rosnav/bin/activate
}

activate

echo ${params}|xargs python /root/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training/train_agent.py
