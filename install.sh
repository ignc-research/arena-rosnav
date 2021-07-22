#!/bin/bash

mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --depth 1 https://github.com/wittenator/arena-rosnav.git
cd arena-rosnav

sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update

sudo apt-get install aptitude

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo aptitude update
sudo aptitude -y install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.$(echo $0)rc
source ~/.$(echo $0)rc

sudo aptitude -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

sudo aptitude update && sudo aptitude -y install \
libopencv-dev \
liblua5.2-dev \
screen \
python3-rospkg-modules \
ros-noetic-navigation \
ros-noetic-teb-local-planner \
ros-noetic-mpc-local-planner \
libarmadillo-dev \
ros-noetic-nlopt \
ros-noetic-geometry2

poetry install

rosws update

echo "export PYTHONPATH=${PWD}:\$PYTHONPATH" >> ~/.$(echo $0)rc

source ~/.$(echo $0)rc
poetry run catkin_make -C ../.. -DCMAKE_BUILD_TYPE=Release

echo "source ~/.profile" >> ~/.$(echo $0)rc

echo "source $(readlink -f ${PWD}/../../devel/setup.sh)" >> ~/.$(echo $0)rc
source ~/.$(echo $0)rc
