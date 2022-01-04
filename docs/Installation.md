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
```
sudo apt-get update && sudo apt-get install -y \
libqt4-dev \
libopencv-dev \
liblua5.2-dev \
screen \
python3.8 \
python3.8-dev \
libpython3.8-dev \
python3-catkin-pkg-modules \
python3-rospkg-modules \
python3-empy \
python3-setuptools \
ros-noetic-navigation \
ros-noetic-teb-local-planner \
ros-noetic-mpc-local-planner \
libarmadillo-dev \
ros-noetic-nlopt \
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
```

* Create a new venv

Note: You might need to restart your terminal at this point.
```
mkvirtualenv --python=python3.8 rosnav
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

#### 1.3. Install arena-rosnav repo
* Create a arena_ws and clone this repo into your arena_ws 
````
cd $HOME
mkdir -p arena_ws/src && cd arena_ws/src
git clone https://github.com/ignc-research/arena-rosnav

cd arena-rosnav && rosws update
source $HOME/.bashrc
cd ../.. 
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
````
Note: if you use bash replace bash with bash in the commands

* Install ros geometry2 from source(compiled with python3) 

The official ros only support tf2 with python2. In order to make the *tf* work in python3, its necessary to compile it with python3. We provided a script to automately install this
and do some additional configurations for the convenience . You can simply run it with 
```
cd $HOME/arena_ws/src/arena-rosnav
./geometry2_install.sh
```

If the command failed to compile:
```
cd $HOME/geometry2_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

* Set python path in .bashrc (or .zshrc if you use that)
```
nano ~/.bashrc
```
Add these lines below "source/opt/ros/noetic/setup.bash"
```
source /$HOME/arena_ws/devel/setup.bash
export PYTHONPATH=$HOME/arena_ws/src/arena-rosnav:${PYTHONPATH}
export PYTHONPATH=$HOME/geometry2_ws/devel/lib/python3/dist-packages:${PYTHONPATH}
```
Add this line above "source/opt/ros/noetic/setup.bash"
```
export PYTHONPATH=""
```

* Install CADRL dependencies (venv always activated!) 
```
workon rosnav
cd $HOME/arena_ws/src/arena-rosnav/arena_navigation/arena_local_planner/model_based/cadrl_ros
pip install -r requirements_cadrl.txt
```
If you encounter errors, e.g. specific versions not found, please manually install the packages with an available version.
You only need this to run our cadrl node, if you dont plan to use it, skip this step.


* Inside forks/stable-baselines3
```
pip install -e .

```
* inside arena_ws:
```
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Update after developing flatland code
After changes inside the forks/flatland folder you should do the following steps to fetch the latest version:
```
cd $HOME/arena_ws/src/arena-rosnav
rosws update
```
pull latest ignc-flatland version 
```
cd $HOME/arena_ws/src/forks/flatland
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

# Training with GPU RTX 3090
in order to train with an NVIDIA GPU RTX3090 you need the latest version of pytorch. Inside your venv, do:
```
pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
```
