FROM ubuntu:18.04

RUN DEBIAN_FRONTEND="noninteractive" apt-get update && apt-get install -y aptitude git software-properties-common

WORKDIR /root/catkin_ws/src/arena-rosnav/

RUN  add-apt-repository universe
RUN  add-apt-repository multiverse
RUN  add-apt-repository restricted
RUN  apt update

RUN  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN  aptitude update
RUN DEBIAN_FRONTEND="noninteractive"  aptitude -y install ros-melodic-desktop-full

#RUN touch ~/.bashrc && echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN . /opt/ros/melodic/setup.sh

RUN  aptitude -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
RUN  rosdep init
RUN rosdep update

RUN  aptitude update && sudo aptitude -y install \
libopencv-dev \
liblua5.2-dev \
screen \
python3-rospkg-modules \
ros-melodic-navigation \
ros-melodic-teb-local-planner \
ros-melodic-mpc-local-planner \
libarmadillo-dev \
ros-melodic-nlopt \
ros-melodic-geometry2

RUN aptitude install -y curl
RUN curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | POETRY_HOME=/opt/poetry python && \
    cd /usr/local/bin && \
    ln -s /opt/poetry/bin/poetry

RUN poetry install

RUN rosws update
RUN . /opt/ros/melodic/setup.bash

RUN poetry run catkin_make -C ../.. -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=$(poetry env info -p)

RUN . ../../devel/setup.sh
