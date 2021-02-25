# sarl_star
ROS implementation of the paper [SARL*: Deep Reinforcement Learning based Human-Aware Navigation for Mobile Robot in Indoor Environments](https://ieeexplore.ieee.org/abstract/document/8961764) presented in ROBIO 2019. This mobile robot navigation framework is implemented on a Turtlebot2 robot platform with lidar sensors (Hokuyo or RPlidar), integrating SLAM, path planning, pedestrian detection and deep reinforcement learning algorithms.

**Video demonstration can be found on**   [Youtube](https://youtu.be/izrrctuUd-g) or [bilibili](https://www.bilibili.com/video/av74520445).

## Introduction
We present an advanced version of the Socially Attentive Reinforcement Learning (SARL) algorithm, namely SARL*, to achieve human-aware navigation in indoor environments. Recently, deep RL has achieved great success in generating human-aware navigation policies. However, there exist some limitations in the real-world implementations: the learned navigation policies are limited to certain distances associated with the training process, and the simplification of the environment neglects obstacles other than humans. In this work, we improve the SARL algorithm by introducing a dynamic local goal setting mechanism and a map-based safe action space to tackle the above problems. 

## Method Overview
![For more details, please refer to the paper.](https://github.com/LeeKeyu/sarl_star/blob/master/imgs/overview.png)


## System Setup

We use the laser scanner **Hokuyo UTM-30LX** or **RPLIDAR-A2** as the sensor and [**TurtleBot 2**](http://wiki.ros.org/turtlebot/Tutorials/indigo)  as the robot platform.

![](https://github.com/LeeKeyu/sarl_star/blob/master/imgs/system.png)



## Some Experiments
![For more details, please refer to the paper.](https://github.com/LeeKeyu/sarl_star/blob/master/imgs/experiments.png)

## Code Structure
- **[Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2)**: Crowd simulator using Optimal Reciprocal Collision Avoidance algorithm.
- [**laser_filters**](http://wiki.ros.org/laser_filters): ROS package to filter out unwanted laser scans. (**optional**)
 - **[navigation](http://wiki.ros.org/navigation/Tutorials)**: ROS stack to provide AMCL localization, costmaps and basic path planners.
 - **[people](http://wiki.ros.org/people)**: ROS stack to detect and track humans using sensor information.
 - [**rplidar_ros**](http://wiki.ros.org/rplidar_ros): ROS package to use ROS with rplidar sensor.
 - **sarl_star_ros** : Core ROS package to run the SARL* navigation algorithm.
 - **[turtlebot_apps](http://wiki.ros.org/turtlebot_apps)**: ROS stack to use ROS with TurtleBot.

## Build & Install
Our codes have been tested in Ubuntu 16.04 with Python 2.7. 
1. Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
2. Create and build a catkin workspace and download the codes into src/:
```
mkdir -p ~/sarl_ws/src
cd ~/sarl_ws/
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/LeeKeyu/sarl_star.git
```
3. Install other dependencies:

```
sudo apt-get install libbullet-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
sudo apt-get install ros-kinetic-bfl
sudo apt-get install ros-kinetic-tf2-sensor-msgs
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs
pip install empy
pip install configparser
```
4. Install [Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2):

```
cd sarl_star/Python-RVO2/
pip install -r requirements.txt
python setup.py build
python setup.py install
```
5. Install CrowdNav (Note that the CrowdNav in this repository are modified from [the original SARL implementation](https://github.com/vita-epfl/CrowdNav)):

```
cd sarl_star/sarl_star_ros/CrowdNav/
pip install -e .
```

6. Build the catkin workspace:

```
cd ~/sarl_ws/
catkin_make
source devel/setup.bash
```

## Start the Navigation

0. Before starting the navigation, make sure your PC is connected with Turtlebot2 and the lidar sensor (either Hokuyo or RPlidar).
1. Bringup the turtlebot

```
roslaunch turtlebot_bringup minimal.launch
```
2. Build a map of your environment using [gmapping](http://wiki.ros.org/gmapping) package:


If you're using Hokuyo, run
```
roslaunch turtlebot_navigation hokuyo_gmapping_movebase.launch 
```
If you're using RPlidar, run
```
roslaunch turtlebot_navigation rplidar_gmapping_movebase.launch 
```
Then push or tele-operate the robot to explore the environment and build a map. You will be able to view the real-time navigation in [RVIZ](http://wiki.ros.org/rviz). 
To save the map, open a new terminal and run:

```
mkdir -p ~/sarl_ws/src/sarl_star/sarl_star_ros/map
rosrun map_server map_saver -f ~/sarl_ws/src/sarl_star/sarl_star_ros/map/new_map
```

3. Start navigation using the SARL* policy:

  The "sarl_star_navigation.launch" includes the following parts:

 - **Laser type**: To specify the laser type: "hokuyo" or "rplidar"
 - **Laser driver**: To start the laser.
 - [**Laser filter**](http://wiki.ros.org/laser_filters): To filter the laser scans (in our implementation, the laser scans in 0.3m range of the robot or behind the robot are filtered out to avoid misidentifying the robot's structure as human legs) (**optional**)
 - **[Map server](http://wiki.ros.org/map_server)**: To provide the map. (change the map name to your map)
 - [**AMCL**](http://wiki.ros.org/amcl): To track the pose of the robot in the map.
 - **[**People Detector**](http://wiki.ros.org/people)**: To detect and track humans using laser scan information.
 - [**Move base**](http://wiki.ros.org/move_base): To provide the global planner and costmap during navigation.
 - **SARL_star Planner** : To run the SARL* algorithm and send motion commands to the robot.
 - **[RVIZ](http://wiki.ros.org/rviz)**: To visualize the navigation.

To start the SARL* navigation, run
```
roslaunch sarl_star_ros sarl_star_navigation.launch
```
You will see the rviz window which shows the robot navigation in real time. Draw a goal pose in the map (following the instructions in [ROS RVIZ tutorials](http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack)), the robot will navigate using the SARL* policy, which will avoid humans (blue ballls) and static obstacles in the map, and dynamically update its local goal (red cubes) according to the global plan. The action commands given to the robot can be seen in rviz as green arrows.

To navigate using the original SARL policy, run:

```
roslaunch sarl_star_ros sarl_original_navigation.launch
```
Note that we also add the implementation of map-based safe action space to the original SARL policy to ensure the safety of the robot in the real indoor environment.

## Citation
If you find our work useful in your research, please consider citing our paper:
```
@INPROCEEDINGS{8961764,  
author={K. {Li} and Y. {Xu} and J. {Wang} and M. Q. -. {Meng}},  
booktitle={2019 IEEE International Conference on Robotics and Biomimetics (ROBIO)},   
title={SARL∗: Deep Reinforcement Learning based Human-Aware Navigation for Mobile Robot in Indoor Environments},   
year={2019},  
volume={},  
number={},  
pages={688-694},}
```
## Questions

If you have any questions, please contact "kyli@link.cuhk.edu.hk".


