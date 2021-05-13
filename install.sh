mkdir -p catkin_ws/src && 
cd catkin_ws/src
git clone git@github.com:wittenator/arena-rosnav.git
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

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

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

source $(poetry env info --path)/bin/activate

PYTHONPATH=$(which python)

rosws update
source $HOME/.bashrc
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.sh
