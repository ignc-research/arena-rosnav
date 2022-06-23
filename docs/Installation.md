## 1. Installation
#### 1.1. Standard ROS setup
(Code has been tested with ROS-noetic on Ubuntu 20.04 and Python 3.8)

* Configure your Ubuntu repositories
```
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update
```

* Setup your scources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

*	Set up your keys
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

*	Installation
```
sudo apt update
sudo apt install ros-noetic-desktop-full
```

* Environment Setup
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

*	Dependencies for building packages
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

* Initialize rosdep
```
sudo rosdep init
rosdep update
```

* Install additional pkgs 
```bash
sudo apt-get update && sudo apt-get install -y \
libopencv-dev \
liblua5.2-dev \
screen \
python3-rosdep \
python3-rosinstall \
python3-rosinstall-generator \
build-essential \
python3-rospkg-modules \
ros-noetic-navigation \
ros-noetic-teb-local-planner \
ros-noetic-mpc-local-planner \
libarmadillo-dev \
ros-noetic-nlopt \
ros-noetic-turtlebot3-description \
ros-noetic-turtlebot3-navigation \
ros-noetic-lms1xx \
ros-noetic-velodyne-description 
```

#### 1.2. Prepare virtual environment & install python packages
To be able to use python3 with ROS, you need an virtual environment. We recommend using virtualenv & virtualenvwrapper. 

* Install virtual environment and wrapper (as root or admin! with sudo) on your local pc (without conda activated. Deactivate conda env, if you have one active)
```
sudo pip3 install --upgrade pip
sudo pip3 install virtualenv
sudo pip3 install virtualenvwrapper
which virtualenv   # should output /usr/local/bin/virtualenv  
```

* Create venv folder inside your home directory
```
cd $HOME
mkdir python_env   # create a venv folder in your home directory 
```

* Add exports into your .bashrc (if you use zsh change the last line to bashrc instead of bashrc):
```
echo "export WORKON_HOME=$HOME/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3 
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
source ~/.bashrc
```

* Create a new venv

```
mkvirtualenv --python=python3.8 rosnav
workon rosnav
```

* Install packages inside your venv (venv always activated!):
```
pip3 install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip3 install pyyaml catkin_pkg netifaces pathlib filelock pyqt5 mpi4py torch lxml scipy defusedxml aliyun-fc2
```     


#### 1.3. Install arena-rosnav repo
* Create a arena_ws and clone this repo into your arena_ws 
````
cd $HOME
mkdir -p arena_ws/src && cd arena_ws/src
git clone https://github.com/flameryx/arena-rosnav-noetic-devel-branch

cd arena-rosnav-noetic-devel-branch && rosws update
source $HOME/.bashrc
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_CXX_STANDARD=14
````
Note: if you use bash replace bash with bash in the commands

# Training with GPU RTX 3090
in order to train with an NVIDIA GPU RTX3090 you need the latest version of pytorch. Inside your venv, do:
```
pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
```
