FROM ubuntu:18.04

RUN apt-get update && apt-get install -y git

RUN mkdir -p catkin_ws/src && cd catkin_ws/src
RUN git clone https://github.com/wittenator/arena-rosnav.git
RUN cd arena-rosnav

RUN  add-apt-repository universe
RUN  add-apt-repository multiverse
RUN  add-apt-repository restricted
RUN  apt update

RUN  apt-get install -y aptitude

RUN  sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN  apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN  aptitude update
RUN  aptitude -y install ros-melodic-desktop-full

RUN  "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN . ~/.bashrc

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

RUN poetry install

RUN rosws update
RUN . $HOME/.bashrc

RUN poetry run catkin_make -C ../.. -DCMAKE_BUILD_TYPE=Release

RUN . ../../devel/setup.sh
