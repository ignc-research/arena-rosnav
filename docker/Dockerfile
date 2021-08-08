################################################################i
# Dockerfile for Arena-Rosnav

# URL: https://github.com/ignc-research/arena-rosnav

# Based on Ubuntu 18.04 + ROS-Melodic-Desktop-Full Version

#################################################################

# This image includes additional meta-packages such for desktop installations than offical image
FROM osrf/ros:melodic-desktop-full

# Change the default shell from /bin/sh to /bin/bash
SHELL ["/bin/bash","-c"]

# 1. Install additinal pkgs
RUN apt-get -y update && apt-get install -y \
    gnutls-bin \
    vim \
    git \
    original-awk \
    python3-pip \
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
    ros-melodic-navigation \
    ros-melodic-teb-local-planner \
    ros-melodic-mpc-local-planner \
    ros-melodic-nlopt \
    libarmadillo-dev \
&& echo $'\n\
source /opt/ros/melodic/setup.sh' >> /root/.bashrc
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
&& DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration
RUN apt-get install -y software-properties-common
RUN apt-get install wget
# 2. Prepare virtual environment
RUN pip3 install --upgrade pip \
&& pip3 install virtualenv  virtualenvwrapper\
&& cd /root \
&& mkdir .python_env \
&& echo $'\n\
export WORKON_HOME=/root/.python_env \n\
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \n\
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv \n\
export PYTHONPATH=/root/catkin_ws/src/arena-rosnav:${PYTHONPATH} \n\
source /usr/local/bin/virtualenvwrapper.sh' >> /root/.bashrc

ENV WORKON_HOME=/root/.python_env \
    VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3 \
    VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
    
# 3. Create python virtualenv and install dependencies
RUN . /usr/local/bin/virtualenvwrapper.sh \
&& mkvirtualenv --python=python3.6 rosnav \ 
&& workon rosnav \
&& pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed \
&& pip3 install pyyaml catkin_pkg netifaces pathlib \
&& pip3 install filelock
#&& pip3 --no-cache-dir install stable-baselines3

# 4. Install Arena-Rosnav repo and compile
RUN git clone https://github.com/ignc-research/arena-rosnav /root/catkin_ws/src/arena-rosnav
RUN wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker-compose.yml
RUN cp docker-compose.yml /root/catkin_ws/src/arena-rosnav
RUN mkdir /root/catkin_ws/src/arena-rosnav/docker/
RUN wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker/entrypoint_train.sh \
&& wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker/Dockerfile \
&& wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker/novnc.env \
&& wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker/py_train.sh \
&& wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker/ros.sh \
&& wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker/roslaunch.env \
&& wget https://raw.githubusercontent.com/Ann-Rachel/arena-rosnav/docker_new/docker/train_params.csv 
RUN cp entrypoint_train.sh /root/catkin_ws/src/arena-rosnav/docker \
&& cp Dockerfile /root/catkin_ws/src/arena-rosnav/docker \
&& cp novnc.env /root/catkin_ws/src/arena-rosnav/docker \
&& cp py_train.sh /root/catkin_ws/src/arena-rosnav/docker \
&& cp ros.sh /root/catkin_ws/src/arena-rosnav/docker \
&& cp roslaunch.env /root/catkin_ws/src/arena-rosnav/docker \
&& cp train_params.csv /root/catkin_ws/src/arena-rosnav/docker

RUN . /usr/local/bin/virtualenvwrapper.sh \
&& . /opt/ros/melodic/setup.sh \
&& workon rosnav \
&& pip install gym \
&& pip install PyQt5 \
&& cd /root/catkin_ws/src/arena-rosnav \
&& git checkout local_planner_subgoalmode \
&& rosws update \
&& cd /root/catkin_ws/src/forks/stable-baselines3 \
&& git pull \
&& pip install -e .   

#RUN apt-get -y update && apt-get install -y libvtk7.1-qt
RUN . /opt/ros/melodic/setup.sh \
&& cd /root/catkin_ws \
&& catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
&& echo $'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

# 5. Install geometry2 pkg
RUN . /root/.bashrc \
&& . /opt/ros/melodic/setup.sh \
&& cd /root/catkin_ws/src/arena-rosnav/ \
&& . geometry2_install.sh

# 6. start workon rosnav
RUN echo $'source /root/.python_env/rosnav/bin/activate' >> /root/.bashrc

# 7. Entrypoint
RUN ["chmod", "+x", "/root/catkin_ws/src/arena-rosnav/docker/entrypoint_train.sh"]
WORKDIR /root/catkin_ws/src/arena-rosnav/
ARG ROS_START="train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4"
ARG PYTHON_START="--custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn relu"
ENV roslaunchParams="$ROS_START"
ENV pythonParams="$PYTHON_START"
ENTRYPOINT docker/entrypoint_train.sh "$roslaunchParams" "$pythonParams"
