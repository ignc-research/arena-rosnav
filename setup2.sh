#!/bin/bash

if test -n "$ZSH_VERSION"; then
  CURSHELL=zsh
elif test -n "$BASH_VERSION"; then
  CURSHELL=bash
else
  echo "Currently only Bash and ZSH are supported for an automatic install. Please refer to the manual installation if you use any other shell."
  exit 1
fi

# NOTE: This script requires a pyten env called 'rosnav'
pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip3 install pyyaml catkin_pkg netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml numpy scikit-image Pillow rospkg
pip install PyQt5 --upgrade

# install the arena-rosnav-repo
cd $HOME && mkdir -p arena_ws/src && cd arena_ws/src
git clone https://github.com/ignc-research/arena-rosnav -b noetic-devel
cd arena-rosnav && rosws update
source $HOME/.${CURSHELL}rc 
source `which virtualenvwrapper.sh` && workon rosnav 
cd $HOME/arena_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_CXX_STANDARD=14

echo "source $HOME/arena_ws/devel/setup.bash
export PYTHONPATH=$HOME/arena_ws/src/arena-rosnav:${PYTHONPATH}:/opt/ros/noetic/lib/python3/dist-packages" >> ~/.${CURSHELL}rc
