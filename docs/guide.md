This documentation will help you set up Arena-Rosnav and provide an overview on how to start a simulation and how to use Arena-Rosnav for training and evaluation of robot navigation. Please also check out the glossary on the bottom of this documentation for better understanding.

# Installation
## Set up Ubuntu in Windows (For Windows users only)
Note: If you already have a Linux distribution, you can continue with **Installation of Arena-Rosnav**.

### 1. Installation of WSL2
Please follow the steps in this [WSL installation guide for Windows 10](https://docs.microsoft.com/en-us/windows/wsl/install-win10) to install WSL2 on your computer.

Note: If you already have a Linux distribution, skip this step.

#### Troubleshooting
You might encounter this problem during installation:
```
Installing, this may take a few minutes...
WslRegisterDistribution failed with error: 0x80370102
Error: 0x80370102 The virtual machine could not be started because a required feature is not installed.
```
This problem can be resolved by enabling CPU virtualization in your BIOS. How you can achieve this depends on your hardware.
[This guide from bleepingcomputer](https://www.bleepingcomputer.com/tutorials/how-to-enable-cpu-virtualization-in-your-computer-bios/) might help you with that.

### 2. Installation of Windows-X-Server
To use WSL with graphical programs, an X-server will need to be installed on the Windows 10 system and the DISPLAY variable will need to be set in Bash/Zsh.
One possible program to use is [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

#### Set up DISPLAY variable
After installing the X-server you need to set the DISPLAY variable in your Bash/Zsh.
Use ```nano ~/.bashrc``` or ```nano ~/.zshrc``` and insert the following code on the bottom of the file.

```export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0```

Exit via Ctrl+X. Agree with Y. Press Enter to save the file name (don't change it).

#### Xlaunch Settings
Start Xlaunch and configure it the following way. In the end the configuration can be saved.
##### Display Settings
- Choose Option: Multiple Windows
- Set Display Number to 0
##### Client Settings
- Choose Option: Start no Client
##### Extra Settings
- Choose Option: Disable access control
#### Trouble Shooting
If you encounter problems, you might go to Windows Defender Firewall -> Communication between Applications and Windows Firewall.
Look for VcXsrv and change the settings to both private and public checked.

### 3. Installation of Visual Studio Code and 
We recommend you use Visual Studio Code as your programming environment. Please follow the instructions in this [VS Code with WSL tutorial](https://docs.microsoft.com/en-us/windows/wsl/tutorials/wsl-vscode).

## Installation of Arena-Rosnav
Please install Arena-Rosnav according to the step-by-step instruction in [Installation.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md).

# Usage
## Start a simulation
The most basic simulation can be started by using the following command. Please make sure you are working in your virtual environment by running ```workon rosnav``` beforehand.
```
roslaunch arena_bringup start_arena_flatland.launch  train_mode:=false
```
Now you can click on the 2D Nav Goal button in RViz to generate a goal towards which the agent will move automatically and stop after reaching it.

You can specify the following parameters:
- train_mode:=<true, false>
- use_viz:=<true, false> (default true)
- local_planner:=<teb,dwa,mpc,cadrl,arena2d> (default dwa)
- ~~task_mode:=<random, manual, scenario> (default random)~~ (redundant and does not need to be specified anymore)
- obs_vel:= # maximum velocity of dynamic obstacles [m/s]. It is recommended to set a max velocity within [0.1,0.7] (default 0.3)
- map_file:= # e.g. map1 (default map_empty)

## Training the agent
Please refer to [DRL-Training.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/DRL-Training.md) for detailed explanations about agent, policy and training setups.

During training the agent will at first behave randomly and over time learns to navigate to the goal and avoid obstacles.

#### Trouble Shooting
While trying the Quickstart you might encouter the following error in the second terminal:
```
Traceback (most recent call last):
  File "scripts/training/train_agent.py", line 229, in <module>
    treshhold_type="succ", threshold=0.9, verbose=1)
TypeError: __init__() got an unexpected keyword argument 'treshhold_type'
```
This error can be resolved by updating your stable baselines and your workspace. Therefore run the following commands:
```
cd $HOME/catkin_ws/src/forks/stable-baselines3
pip install -e.
```
```
cd $HOME/catkin_ws/src/arena-rosnav
rosws update
```
```
cd $HOME/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Evaluation of agent performance
After training agents, their performance can be evaluated following [Evaluation.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Evaluations.md).

Note: The evaluation results will be recorded in .rosbag files in the directory in which the rosbag command was run.

#### Troubleshooting
There are different errors that can occur depending on the circumstances. Running the following commands in respective order usually solves most errors:
- Activate virtual environment
```
workon rosnav
```
- Update git repository
```
cd $HOME/catkin_ws/src/arena-rosnav
git pull
```
- Update ROS workspace
```
cd $HOME/catkin_ws/src/arena-rosnav
rosws update
```
- Build your workspace
```
cd $HOME/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Another source of error could be your ```PYTHONPATH```. Please check it with ```echo $PYTHONPATH```. The output should look like this:
```
/home/user/catkin_ws/devel/lib/python3/dist-packages:
/home/user/catkin_ws/src/arena-rosnav:
/home/user/geometry2_ws/devel/lib/python3/dist-packages:
/opt/ros/melodic/lib/python2.7/dist-packages
```
```/opt/ros/melodic/lib/python2.7/dist-packages``` should be listed last. The order of the first 3 paths is not relevant.

## Plotting evaluation results
### 1. Quantitative plots
There is a script for plotting quantitative results called [sim_evaluation_v3.py](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/quantitative). The version number might change over time. How to use this script is explained in the [readme.md for quantitative plotting](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/readme.md).
### 2. Qualitative plots
Please refer to the [readme.md for qualitative plotting](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/readme.md) on qualitative plots for instructions.

# Glossary
- global planner: calculates collision free global path from start to goal only considering the map layout
- local planner: navigates the robot along the global path and reacts to local objects not considered by global path
- waypoint generator: generates sub goals (waypoints) along global path which will each be targeted by the local planner
