
---
---
Major differences to `local_planner_subgoalmode` branch are:
+ Usage of `ros-noetic`
+ Full support of the following robots with realistic parameters: `turtlebot3_burger`, `ridgeback`, `jackal`, `agv-ota` _(see [here](docs/Simulation.md#Robots) for detailed description of their usage)_

    THIS INCLUDES:
    - Full support of the planers `teb`, `dwa`, `mpc`, `cadrl`, `rlca`, `arena`
    - Realistic flatland models for each robot
    - Defined action space for each Robot (for _rl-training_)
+ Support of the planer `arena`
+ Setting of navigation goal automatically for all planers in `testing` mode

---
---

# Arena-Rosnav (IROS 21)
A flexible, high-performance 2D simulator with configurable agents, multiple sensors, and benchmark scenarios for testing robotic navigation. 

Arena-Rosnav uses Flatland as the core simulator and is a modular high-level library for end-to-end experiments in embodied AI -- defining embodied AI tasks (e.g. navigation, obstacle avoidance, behavior cloning), training agents (via imitation or reinforcement learning, or no learning at all using conventional approaches like DWA, TEB or MPC), and benchmarking their performance on the defined tasks using standard metrics.


| <img width="400" height="400" src="/img/rosnav1.gif"> | <img width="400" height="400" src="/img/rosnav2.gif"> |
|:--:| :--:| 
| *Training Stage* | *Deployment Stage* |


## What is this repository for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is inspired by Ronja Gueldenring's work: [drl_local_planner_ros_stable_baselines](https://github.com/RGring/drl_local_planner_ros_stable_baselines.git). Test state of the art local and global planners in ROS environments both in simulation and on real hardware. Following features are included:

* Setup to train a local planner with reinforcement learning approaches from [stable baselines3](https://github.com/DLR-RM/stable-baselines3.git)
* Training in simulator [Flatland](https://github.com/avidbots/flatland) in train mode
* Include realistic behavior patterns and semantic states of obstacles (speaking, running, etc.)
* Include different obstacles classes (other robots, vehicles, types of persons, etc.)
* Implementation of intermediate planner classes to combine local DRL planner with global map-based planning of ROS Navigation stack
* Testing a variety of planners (learning based and model based) within specific scenarios in test mode
* Modular structure for extension of new functionalities and approaches

# Start Guide
We recommend starting with the [start guide](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/guide.md) which contains all information you need to know to start off with this project including installation on **Linux and Windows** as well as tutorials to start with. 

* For Mac, please refer to our [Docker](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Docker.md).


## 1. Installation
Please refer to [Installation.md](docs/Installation.md) for detailed explanations about the installation process.

## 1.1. Docker
We provide a Docker file to run our code on other operating systems. Please refer to [Docker.md](docs/Docker.md) for more information.

## 2. Usage

### DRL Training

Please refer to [DRL-Training.md](docs/DRL-Training.md) for detailed explanations about agent, policy and training setups.

### Scenario Creation with the [arena-scenario-gui](https://github.com/ignc-research/arena-scenario-gui/)
To create complex, collaborative scenarios for training and/or evaluation purposes, please refer to the repo [arena-scenario-gui](https://github.com/ignc-research/arena-scenario-gui/). This application provides you with an user interface to easily create complex scenarios with multiple dynamic and static obstacles by drawing and other simple UI elements like dragging and dropping. This will save you a lot of time in creating complex scenarios for you individual use cases.

# IROS21 information
To test the code and reproduce the experiments, follow the installation steps in [Installation.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Installation.md). Afterwards, follow the steps in [Evaluations.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/Evaluations.md).

To test the different **Waypoint Generators**, follow the steps in [waypoint_eval.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/eval_28032021.md)

**DRL agents** are located in the [agents folder](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/agents).


# Used third party repos:
* Flatland: http://flatland-simulator.readthedocs.io
* ROS navigation stack: http://wiki.ros.org/navigation
* Pedsim: https://github.com/srl-freiburg/pedsim_ros
