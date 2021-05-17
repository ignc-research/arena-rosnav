## 1. Installation

#### Installation on Ubuntu 20 using the installation script

##### Prerequisites

In order to use the install script you need to have (Poetry)[https://python-poetry.org/docs/] installed and available in your shell.

#### Installation

 Navigate to the directory, where you want your code to reside and execute our install script which sets everything up:

'curl -sSL https://raw.githubusercontent.com/wittenator/arena-rosnav/local_planner_subgoalmode/install.sh | bash'


If you have troubles with the installation we recommend to follow the manual installation procedure below and replacing `melodic` by `noetic` in the respective places. Pay attention to changing the ROS dependency branches in the `.rosinstall` as well.

#### Manual Installation

##### 1.1. Standard ROS setup
(Code has been tested with ROS-melodic on Ubuntu 18.04 and Python 3.6)

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
sudo apt install ros-melodic-desktop-full
```

* Environment Setup
```
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
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
```
sudo apt-get update && sudo apt-get install -y \
libqt4-dev \
libopencv-dev \
liblua5.2-dev \
screen \
python3-rospkg-modules \
ros-melodic-navigation \
ros-melodic-teb-local-planner \
ros-melodic-mpc-local-planner \
libarmadillo-dev \
ros-melodic-nlopt \
```

##### 1.3. Install arena-rosnav repo
* Create a catkin_ws and clone this repo into your catkin_ws 
````
cd $HOME
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/ignc-research/arena-rosnav

cd arena-rosnav 
````
To be able to use python3 with ROS, you need an virtual environment. We recommend using poetry. 

* Install the virtual environment and libraries by navigating to the root of the repository.
```
poetry install
```

* Activate the virtual environment
```
poetry shell
```

* Build the project

```
rosws update
source $HOME/.zshrc
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
```

Note: if you use bash replace zsh with bash in the commands

* Install ros geometry2 from source(compiled with python3) 

The official ros only support tf2 with python2. In order to make the *tf* work in python3, its necessary to compile it with python3. We provided a script to automately install this
and do some additional configurations for the convenience . You can simply run it with 
```bash
./geometry2_install.sh
```

If the command failed to compile:
```
cd $HOME/geometry2_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

* Set python path in .zshrc (or .bashrc if you use that)
```
nano ~/.zshrc
```
Add these lines below "source/opt/ros/melodic/setup.zsh"
```
source /$HOME/catkin_ws/devel/setup.zsh
export PYTHONPATH=$HOME/catkin_ws/src/arena-rosnav:${PYTHONPATH}
export PYTHONPATH=$HOME/geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
```
Add this line above "source/opt/ros/melodic/setup.zsh"
```
export PYTHONPATH=""
```

* Install CADRL dependencies (venv always activated!) 
```
workon rosnav
cd $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/cadrl_ros
pip install -r requirements_cadrl.txt
```
If you encounter errors, e.g. specific versions not found, please manually install the packages with an available version.
You only need this to run our cadrl node, if you dont plan to use it, skip this step.


* Inside forks/stable_baselines3
```
pip install -e .

```
* inside catkin_ws:
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Update after developing flatland code
After changes inside the forks/flatland folder you should do the following steps to fetch the latest version:
```
cd $HOME/catkin_ws/src/arena-rosnav
rosws update
```
pull latest ignc-flatland version 
```
cd $HOME/catkin_ws/src/forks/flatland
git pull
```
# Error Handling 
if you encounter the error "world path not given", it is probably because you havent updated the forks repository or working on an old branch.
In that case go to the arena-rosnav folder and do
```
rosws update
```
Subsequently, go to the forks/stable_baselines3 folder and do:
```
pip install -e .
```
