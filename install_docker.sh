#!/bin/bash

case $(lsb_release -sc) in
  focal)
    ROS_NAME_VERSION=noetic
    ;;
  
  bionic)
    ROS_NAME_VERSION=melodic
    ;;

  *)
    echo "Currently only Ubuntu Bionic Beaver and Focal Fossa are supported for an automatic install. Please refer to the manual installation if you use any Linux release or version."
    exit 1
    ;;
esac

if test -n "$ZSH_VERSION"; then
  CURSHELL=zsh
elif test -n "$BASH_VERSION"; then
  CURSHELL=bash
else
  echo "Currently only Bash and ZSH are supported for an automatic install. Please refer to the manual installation if you use any other shell."
  exit 1
fi

apt-get install aptitude -y
aptitude install curl git -y

mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --depth 1 --branch local_planner_subgoalmode https://github.com/ignc-research/arena-rosnav.git
cd arena-rosnav

if [ $ROS_NAME_VERSION = "noetic" ]; then
  mv .rosinstall.noetic .rosinstall
else
  mv .rosinstall.melodic .rosinstall
fi

add-apt-repository universe
add-apt-repository multiverse
add-apt-repository restricted
apt update

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

aptitude update
aptitude -y install ros-${ROS_NAME_VERSION}-desktop-full

echo "source /opt/ros/${ROS_NAME_VERSION}/setup.${CURSHELL}" >> ~/.${CURSHELL}rc
source ~/.${CURSHELL}rc

if [ $ROS_NAME_VERSION = "noetic" ]; then
  aptitude -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
else
  aptitude -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
fi


rosdep init
rosdep update

aptitude update && aptitude -y install \
libopencv-dev \
liblua5.2-dev \
screen \
python3-rospkg-modules \
ros-${ROS_NAME_VERSION}-navigation \
ros-${ROS_NAME_VERSION}-teb-local-planner \
ros-${ROS_NAME_VERSION}-mpc-local-planner \
libarmadillo-dev \
ros-${ROS_NAME_VERSION}-nlopt

rosws update

poetry install

if [ $ROS_NAME_VERSION = "noetic" ]; then
  aptitude -y install ros-noetic-geometry2
else
PYTHON3_EXEC=$(poetry env info -p)/bin/python3
PYTHON3_INCLUDE="$(poetry run ls -d /usr/include/* | grep  python | sort -r| head -1)"
PYTHON3_DLIB="$(poetry run ls -d /usr/lib/x86_64-linux-gnu/* | grep -P  "libpython3\S*.so"| sort | head -1)"
  if [ -z $PYTHON3_DLIB ] || [ -z $PYTHON3_INCLUDE ] || [ -z $PYTHON3_EXEC ] ; then
      echo "Can't find python library please install it with \" apt-get python3-dev \" !" >&2
  fi

  # compile geometry2 with python3 
  echo -n "compiling geometry2 with python3 ..."
  poetry run catkin_make -C ../forks/geometry2_ws/ -D  -DPYTHON_EXECUTABLE=${PYTHON3_EXEC} -DPYTHON_INCLUDE_DIR=${PYTHON3_INCLUDE} -DPYTHON_LIBRARY=${PYTHON3_DLIB}
  echo $PWD

  # add the lib path to the python environment 
  if [ $? -eq 0 ] ; then
      echo " done!"
      rc_info="export PYTHONPATH=$(readlink -f ${PWD}/../../devel/lib/python3/dist-packages):\${PYTHONPATH}\n"
      echo -e "$rc_info" >> ~/.${CURSHELL}rc
  else
      echo "Fail to compile geometry2"
  fi
fi

echo "export PYTHONPATH=${PWD}:${PWD}/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl:\$PYTHONPATH" >> ~/.${CURSHELL}rc

source ~/.${CURSHELL}rc
poetry run catkin_make -C ../.. -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=$(poetry env info -p)/bin/python3

echo "source $(readlink -f ${PWD}/../../devel/setup.sh)" >> ~/.${CURSHELL}rc
source ~/.${CURSHELL}rc
