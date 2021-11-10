 #!/bin/bash

params="$1"

source /opt/ros/melodic/setup.sh

export WORKON_HOME=/root/.python_env
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
export PYTHONPATH=/root/arena_ws/src/arena-rosnav:${PYTHONPATH}
source /usr/local/bin/virtualenvwrapper.sh
source /root/arena_ws/devel/setup.bash
export PYTHONPATH=//geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
source /root/.python_env/rosnav/bin/activate

echo ${params}|xargs python /root/arena_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training/train_agent.py
