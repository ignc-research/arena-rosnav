FROM osrf/ros:noetic-desktop-full


RUN apt-get update && apt-get -y install curl git python3-pip

RUN pip3 install --user poetry
ENV PATH="${PATH}:/root/.local/bin"

RUN apt-get -y install \
python3-rosdep \
python3-rosinstall \
python3-rosinstall-generator \
python3-wstool build-essential \
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

WORKDIR /catkin_ws/src/arena-rosnav

COPY ./pyproject.toml pyproject.toml
COPY ./poetry.lock poetry.lock

RUN poetry install

COPY . .

RUN rosdep update
RUN rosws update

ENV PYTHONPATH="${PWD}:\$PYTHONPATH"

RUN . /opt/ros/noetic/setup.sh && poetry run catkin_make -C ../.. -DCMAKE_BUILD_TYPE=Release

RUN echo ". $(readlink -f ${PWD}/../../devel/setup.sh)" >> ~/.bashrc
