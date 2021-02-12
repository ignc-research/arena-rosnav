#################################################################
# Dockerfile for Arena-Rosnav

# URL: https://github.com/ignc-research/arena-rosnav

# Based on Ubuntu 18.04 + ROS-Melodic-Desktop-Full Version

# Author: Yi Sun

#################################################################

# This image includes additional meta-packages such for desktop installations than offical image
FROM osrf/ros:melodic-desktop-full

# Change the default shell from /bin/sh to /bin/bash
SHELL ["/bin/bash","-c"]

# 1. Install additinal pkgs
RUN apt-get -y update && apt-get install -y \
    libqt4-dev \
    libopencv-dev \
    liblua5.2-dev \
    screen \
    python3.6 \
    python3.6-dev \
    libpython3.6-dev \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-empy \
    python3-setuptools \
    python3-pip \
    ros-melodic-navigation \
    ros-melodic-teb-local-planner \
    ros-melodic-mpc-local-planner \
&& echo $'\n\
source /opt/ros/melodic/setup.sh' >> /root/.bashrc

# 2. Prepare virtual environment
RUN pip3 install --upgrade pip \
&& pip3 install virtualenv  virtualenvwrapper\
&& cd /root \
&& mkdir .python_env \
&& echo $'\n\
export WORKON_HOME=/root/.python_env \n\
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \n\
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv \n\
source /usr/local/bin/virtualenvwrapper.sh' >> /root/.bashrc

ENV WORKON_HOME=/root/.python_env \
    VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \
    VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
    
# 3. Create python virtualenv and install dependencies
RUN . /usr/local/bin/virtualenvwrapper.sh \
&& mkvirtualenv --python=python3.6 rosnav \ 
&& workon rosnav \
&& pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed \
&& pip install pyyaml catkin_pkg netifaces pathlib \
&& pip --no-cache-dir install stable-baselines3

# 4. Install Arena-Rosnav repo and compile
RUN git clone --depth 1 https://github.com/ignc-research/arena-rosnav /root/catkin_ws/src/arena-rosnav \
&& . /usr/local/bin/virtualenvwrapper.sh \
&& . /opt/ros/melodic/setup.sh \
&& workon rosnav \
&& pip install -r /root/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/cadrl_ros/requirements_cadrl.txt \
&& pip install gym \
&&  cd /root/catkin_ws/src/arena-rosnav \
&& rosws update \
&& . /opt/ros/melodic/setup.sh \
&& cd /root/catkin_ws \
&& catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 \
&& echo $'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

# 5. Install geometry2 pkg
RUN . /root/.bashrc \
&& . /opt/ros/melodic/setup.sh \
&& cd /root/catkin_ws/src/arena-rosnav/ \
&& . geometry2_install.sh