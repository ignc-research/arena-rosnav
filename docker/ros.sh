#! /bin/bash
params="$1"
echo "$params"
source /opt/ros/melodic/setup.sh

export WORKON_HOME=/root/.python_env
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
export PYTHONPATH=/root/catkin_ws/src/arena-rosnav:${PYTHONPATH}
source /usr/local/bin/virtualenvwrapper.sh
source /root/catkin_ws/devel/setup.bash
export PYTHONPATH=//geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}

echo python --version
activate () {
  /root/.python_env/rosnav/bin/activate
}

activate
echo python --version

roslaunch arena_bringup start_arena_flatland.launch "$params"
