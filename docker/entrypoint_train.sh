#! /bin/bash
# roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode:=random
roslaunch="$1"
python_train="$2"

echo "$roslaunch"
echo "$python_train"

source /opt/ros/melodic/setup.sh

export WORKON_HOME=/root/.python_env
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
export PYTHONPATH=/root/catkin_ws/src/arena-rosnav:${PYTHONPATH}
source /usr/local/bin/virtualenvwrapper.sh
source /root/catkin_ws/devel/setup.bash
export PYTHONPATH=//geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
source /root/.python_env/rosnav/bin/activate

screen -dmS roslaunch bash -c "source ./docker/ros.sh "$roslaunch""
screen -S roslaunch -X logfile screenlog_roslaunch.log
screen -S roslaunch -X log
sleep 10
screen -ls

# python scripts/training/train_agent.py
bash ./docker/py_train.sh "$python_train"
