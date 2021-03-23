## 1. Installation
#### 1.1. Standard ROS setup
(Code has been tested with ROS-melodic on Ubuntu 18.04 and Python 3.6)

* Install ROS Melodic following the steps from ros wiki:
```
http://wiki.ros.org/melodic/Installation/Ubuntu
```

* Install additional pkgs 
```
sudo apt-get update && sudo apt-get install -y \
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

* Add exports into your .zshrc (if you use bash change the last line to bashrc instead of zshrc):
```
echo "export WORKON_HOME=/home/linh/python_env   #path to your venv folder
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3   #path to your python3 
export VIRTUALENVWRAPPER_VIRTUALENV=/usr/local/bin/virtualenv
source /usr/local/bin/virtualenvwrapper.sh" >> ~/.zshrc
```

* Create a new venv
```
mkvirtualenv --python=python3.6 rosnav
workon rosnav
```

* Install packages inside your venv (venv always activated!):
```
pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf tf2_ros --ignore-installed
pip install pyyaml catkin_pkg netifaces pathlib
```     

* Install stable_baselines3 for training DRL into your venv (venv always activated!)
```
pip install stable-baselines3
```
* (optional) Install CADRL dependencies (venv always activated!) 
```
cd /arena-rosnav/arena_navigation/arena_local_planner/model_based/cadrl_ros
pip install -r requirements_cadrl.txt
```
If you encounter errors, e.g. sopecific versions not found, please manually install the packages with an available version.
  You only need this to run our cadrl node, if you dont plan to use it, skip this step.

#### 1.3. Install arena-rosnav repo
* Create a catkin_ws and clone this repo into your catkin_ws 
````
cd $HOME
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/ignc-research/arena-rosnav

cd arena-rosnav && rosws update
source $HOME/.zshrc
cd ../.. 
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.zsh
````

* Install ros geometry2 from source(compiled with python3) 

The official ros only support tf2 with python2. In order to make the *tf* work in python3, its necessary to compile it with python3. We provided a script to automately install this
and do some additional configurations for the convenience . You can simply run it with 
```bash
./geometry2_install.sh
```
After that you can try to import tf in python3 and no error is supposed to be shown up.

* Install MPC-Planner

```
cd $HOME/catkin_ws/src/forks/navigation/mpc_local_planner
rosdep install mpc_local_planner
```

## Update after developing flatland code
After changes inside the forks/flatland folder you should do the following steps to fetch the latest version:
```
cd arena-rosnav
rosws update
```
pull latest ignc-flatland version 
```
cd src/forks/flatland
git pull
```
## Notes
If you develop on the branch drl_multiprocessing, go inside the forks/flatland folder and checkout to the branch dev_multi_lei, afterwards catkin_make.

