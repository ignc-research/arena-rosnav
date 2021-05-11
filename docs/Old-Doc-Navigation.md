## Arena-Navigation Launch files

### 2.1 Test the simulation environment and task generator

* In one terminal, start simulation. You can specify the following parameters: 

   * train_mode:=<true, false> 
   * use_viz:=<true, false> (default true)
   * local_planner:=<teb,dwa,mpc,cadrl,arena2d> (default dwa)
   * task_mode:=<random, manual, scenario> (default random)
   * obs_vel:=<float> # maximum velocity of dynamic obstacles [m/s]. It is recommended to set a max velocity within [0.1,0.7] (default 0.3)
   * map

```bash
roslaunch arena_bringup start_arena_flatland.launch train_mode:=false use_viz:=true local_planner:=mpc map_file:=map1 obs_vel:=0.3
```
Now you can click on the generate task button in rviz to generator a new random task (random obstacles and goal is published to /goal). It will automatically navigate to that goal, once you start one of our local planners, which are triggered by a new /goal. If you starte with task_mode "manual" you can specify your goal using the specify Flatland Navigation goal (using normal 2D navigation goal will trigger the move_base planner, thus only works with teb and dwa)

### Run Arena2D models in Rosnav
* start simulation as above, then navigate to the folder arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_ros/scripts/ and execute arena_node_tb3.py

```
python arena_node_tb3.py
```
### 2.2. [Quick start] start simulation env & plan manager
````
roslaunch arena_bringup start_arena_flatland.launch  train_mode:=false
````
start_flatland.launch will start several other sublaunch files and some neccesary ros packages:
   1. **start simulator node**: start flatland, load robot model
   2. **start map server node**: load map, which will provide occupancy grid used for mapping functions later
   3. **start fake localization**: which will provide static tf map_to_odom, in order to have localization of the robot.
   4. **start task generator node**: which provide task generation service for rviz_plugin(Generate Task)
   5. **start plan manager node**: provide manager for robot state estimation, mapping, global planner and local planner,  which is the key for navigation framework. The move_base is contained, because currently we need its global_planner and mapping functions, later they won't be needed.
   6. **/train_mode/**: 
   * if true, the simulator(flatland) will provide a *step_world service* and the simulator will update its simulation when he receives a *step_world service request*.
   * if true, the plan manager will generate subgoal topic always as goal(global goal) topic.
   * if false, you can also use move_base action triggered by rviz_plugin button *2D Navigation Goal*. 

### 2.3. [Quick start] test DRL training
Export turtlebot model for simulation 

* In one terminnal, export turtlebot model and start simulation

```bash

roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode:=random

```
* In another terminal

```
workon rosnav
roscd arena_local_planner_drl
python scripts/training/training_example.py
```
first **activate your python3 env**, which contains libaraies stable_baseline3, geometry2
then python run the script.

Hint: During 2021-01-05 and 2021-01-10, arena_local_planner_drl package is still under the development, which means the api of the class could be drastically changed. Sorry about the inconvinience!

### 2.4. Rviz plugins:
   <p align="center">
      <img width="600" height="480" src="img/rviz_plugin_intro.png">
   </p>

   1. 2D Nav Goal: triggers move_base action
   2. Spawn Model: load a new model.yaml to flatland simulator
   3. Arena Nav Goal: set (global) goal for arena navigation
   4. Generate Task: change task, which changes the position of obstacles and set a new goal for arena navigation





### 4. DRL Local planner(Training and Testing)
<p align="center">
  <img width="400" height="300" src="img/local_planner.png">
  <img width="400" height="300" src="img/synchronization.png">
</p>

##### Communication:
DRL local planner get the needed observation info by using ROS communication. This may slows down the training, but for current version we just keep it.

DRL local planner get observation info from:
   * flatland server: laser scan
   * plan manager: robot state, subgoal

DRL local planner send action command to flatland server
   * flatland server: diff_drive

##### Observation synchronization
DRL local planner contains observation collector and we designed a synchronization mechanism for following important reasons & aspects:
   1. In real world, each sensor has its own publishing rate and are different from each other
   2. The action calculation should based on the observations that are synchronized, otherwise is useless.
   3. The calculated action is only valid for a specified time horizon(control horizon),e.g. 0.2s. For different control horizon, the action space should be different. 
      1. example 1: action is calculated every 0.01s, time horizon=0.01s, suppose calculated action=1m/s, in this time horizon the robot will actually move 0.01m.
      2. example 2: action is calculated every 0.5s, time horizon=0.5s, suppose calculated action=1m/s, in this time horizon the robot will actually move 0.5m.
      * From 1 & 2, one can see for a same action space, a different time horizon will result in different actual result.


To be added...


#### 4.1 DRL Agent Training

As a fundament for our Deep Reinforcement Learning approaches [StableBaselines3](https://stable-baselines3.readthedocs.io/en/master/index.html) was used. 

##### Features included so far
* Simple handling of the training script through program parameters
* Choose a predefined Deep Neural Network
* Create your own custom Multilayer Perceptron via program parameters
* Networks will get trained, evaluated and saved
* Load your trained agent to continue training
* Optionally log training and evaluation data
* Enable and modify training curriculum


##### Quick Start

* In one terminnal, start simulation

```bash
roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode=random
```
* In another terminal

```bash
workon rosnav
roscd arena_local_planner_drl
python scripts/training/train_agent.py --agent MLP_ARENA2D
```


#### 4.1.1 Program Arguments

**Generic program call**:
```
train_agent.py [agent flag] [agent_name | unique_agent_name | custom mlp params] [optional flag] [optional flag] ...
```

| Program call         | Agent Flag (mutually exclusive)  | Usage                                                           |Description                                         |
| -------------------- | -------------------------------- |---------------------------------------------------------------- |--------------------------------------------------- | 
| ``` train_agent.py```|```--agent```                     | *agent_name* ([see below](#training-with-a-predefined-dnn))   | initializes a predefined network from scratch
|                      |```--load ```                     | *unique_agent_name* ([see below](#load-a-dnn-for-training))   | loads agent to the given name
|                      |```--custom-mlp```                | _custom mlp params_ ([see below](#training-with-a-custom-mlp))| initializes custom MLP according to given arguments 

_Custom Multilayer Perceptron_ parameters will only be considered when ```--custom-mlp``` was set!
|  Custom Mlp Flags | Syntax                | Description                                   |
| ----------------  | ----------------------| ----------------------------------------------|
|  ```--body ```    | ```{num}-{num}-...``` |architecture of the shared latent network      |
|  ```--pi```       | ```{num}-{num}-...``` |architecture of the latent policy network      |
|  ```--vf```       | ```{num}-{num}-...``` |architecture of the latent value network       |
|  ```--act_fn ```  | ```{relu, sigmoid or tanh}```|activation function to be applied after each hidden layer |

|  Optional Flags        | Description                                    |
| ---------------------- | -----------------------------------------------|
|  ```--n    {num}```    | timesteps in total to be generated for training|
|  ```--tb```            | enables tensorboard logging                    |
|  ```-log```, ```--eval_log```| enables logging of evaluation episodes   |
|  ```--no-gpu```        | disables training with GPU                     |

#### Examples

##### Training with a predefined DNN

Currently you can choose between 3 different Deep Neural Networks each of which have been object of research projects:

| Agent name    | Inspired by   | 
| ------------- | ------------- | 
| MLP_ARENA2D     | [arena2D](https://github.com/ignc-research/arena2D) | 
| DRL_LOCAL_PLANNER | [drl_local_planner](https://github.com/RGring/drl_local_planner_ros_stable_baselines)  |
| CNN_NAVREP | [NavRep](https://github.com/ethz-asl/navrep) | 

e.g. training with the MLP architecture from arena2D:
```
train_agent.py --agent MLP_ARENA2D
```

##### Load a DNN for training

In order to differentiate between agents with similar architectures but from different runs a unique agent name will be generated when using either ```--agent``` or ```--custom-mlp``` mode (when train from scratch).

The name consists of:
```
[architecture]_[year]_[month]_[day]__[hour]_[minute]
```

To load a specific agent you simply use the flag ```--load```, e.g.:
```
train_agent.py --load MLP_ARENA2D_2021_01_19__03_20
```
**Note**: currently only agents which were trained with PPO given by StableBaselines3 are compatible with the training script. 

##### Training with a custom MLP

Instantiating a MLP architecture with an arbitrary number of layers and neurons for training was made as simple as possible by providing the option of using the ```--custom-mlp``` flag. By typing in the flag additional flags for the architecture of latent layers get accessible ([see above](#411-program-arguments)).

e.g. given following architecture:
```
					   obs
					    |
					  <256>
					    |
					  ReLU
					    |
					  <128>
					    |
					  ReLU
				    /               \
				 <256>             <16>
				   |                 |
				 action            value
```

program must be invoked as follows:
```
train_agent.py --custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn relu
```

#### 4.1.2 Hyperparameters

You can modify the hyperparameters in the upper section of the training script which is located at:
```
/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training/train_agent.py
```

Following hyperparameters can be adapted:
|Parameter|Description|
|-----|-----|
| robot | Robot name to load robot specific .yaml file containing its settings. 
| gamma | Discount factor 
| n_steps | The number of steps to run for each environment per update
| ent_coef | Entropy coefficient for the loss calculation
| learning_rate | The learning rate, it can be a function of the current progress remaining (from 1 to 0)  (i.e. batch size is n_steps * n_env where n_env is number of environment copies running in parallel)
| vf_coef | Value function coefficient for the loss calculation
| max_grad_norm | The maximum value for the gradient clipping
| gae_lambda | Factor for trade-off of bias vs variance for Generalized Advantage Estimator
| batch_size | Minibatch size
| n_epochs | Number of epoch when optimizing the surrogate loss
| clip_range | Clipping parameter, it can be a function of the current progress remaining (from 1 to 0).
| reward_fnc | Number of the reward function (defined in _../rl_agent/utils/reward.py_)
| discrete_action_space | If robot uses discrete action space
| task_mode | Mode tasks will be generated in (custom, random, staged). In custom mode one can place obstacles manually via Rviz. In random mode there's a fixed number of obstacles which are spawned randomly distributed on the map after each episode. In staged mode the training curriculum will be used to spawn obstacles. ([more info](#414-training-curriculum))
| curr_stage | When "staged" training is activated which stage to start the training with.

([more information on PPO implementation of SB3](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html))

**Note**: For now further parameters like _max_steps_per_episode_ or _goal_radius_ have to be changed inline (where FlatlandEnv gets instantiated). _n_eval_episodes_ which will take place after _eval_freq_ timesteps can be changed also (where EvalCallback gets instantiated).

#### 4.1.3 Reward Functions

The reward functions are defined in
```
../arena_local_planner_drl/rl_agent/utils/reward.py
```
At present one can chose between two reward functions which can be set at the hyperparameter section of the training script:

  
<table>
<tr>
   <th>rule_00</th> <th>rule_01</th>
</tr>
<tr>
   <td>

   | Reward Function at timestep t                                     |  
   | ----------------------------------------------------------------- |   
   | <img src="https://latex.codecogs.com/gif.latex?r^t&space;=&space;r_{s}^t&space;&plus;&space;r_{c}^t&space;&plus;&space;r_{d}^t&space;&plus;&space;r_{p}^t&space;&plus;&space;r_{m}^t" title="r^t = r_{s}^t + r_{c}^t + r_{d}^t + r_{p}^t + r_{m}^t" /> |
   
   | reward    | description | value |
   | --------- | ----------- | ----- |
   | <img src="https://latex.codecogs.com/gif.latex?r_{s}^t" title="r_{s}^t" /> | success reward | <img src="https://latex.codecogs.com/gif.latex?r_{s}^t&space;=\begin{cases}15&space;&&space;goal\:reached\\0&space;&&space;otherwise\end{cases}" title="r_{s}^t =\begin{cases}15 & goal\:reached\\0 & otherwise\end{cases}" />
   | <img src="https://latex.codecogs.com/png.latex?r_{c}^t" title="r_{c}^t"/> | collision reward | <img src="https://latex.codecogs.com/png.latex?r_{c}^t&space;=\begin{cases}0&space;&&space;otherwise\\-10&space;&&space;agent\:collides\end{cases}" title="r_{c}^t =\begin{cases}0 & otherwise\\-10 & agent\:collides\end{cases}" />
   | <img src="https://latex.codecogs.com/png.latex?r_{d}^t" title="r_{d}^t" />| danger reward | <img src="https://latex.codecogs.com/png.latex?r_{d}^t&space;=\begin{cases}0&space;&&space;otherwise\\-0.15&space;&&space;agent\:oversteps\:safe\:dist\end{cases}" title="r_{d}^t =\begin{cases}0 & otherwise\\-0.15 & agent\:oversteps\:safe\:dist\end{cases}" />
   |<img src="https://latex.codecogs.com/png.latex?r_{p}^t" title="r_{p}^t" />| progress reward | <img src="https://latex.codecogs.com/png.latex?r_{d}^t&space;=&space;w*d^t,&space;\quad&space;d^t=d_{ag}^{t-1}-d_{ag}^{t}\\&space;w&space;=&space;0.25&space;\quad&space;d_{ag}:agent\:goal&space;\:&space;distance" title="r_{d}^t = w*d^t, \quad d^t=d_{ag}^{t-1}-d_{ag}^{t}\\ w = 0.25 \quad d_{ag}:agent\:goal \: distance" />
   | <img src="https://latex.codecogs.com/png.latex?r_{m}^t" title="r_{m}^t" /> | move reward | <img src="https://latex.codecogs.com/png.latex?r_{d}^t&space;=\begin{cases}0&space;&&space;otherwise\\-0.01&space;&&space;agent\:stands\:still\end{cases}" title="r_{d}^t =\begin{cases}0 & otherwise\\-0.01 & agent\:stands\:still\end{cases}" />

   </td>
   <td>

   | Reward Function at timestep t                                     |  
   | ----------------------------------------------------------------- |   
   | <img src="https://latex.codecogs.com/png.latex?r^t&space;=&space;r_{s}^t&space;&plus;&space;r_{c}^t&space;&plus;&space;r_{d}^t&space;&plus;&space;r_{p}^t&space;&plus;&space;r_{m}^t" title="r^t = r_{s}^t + r_{c}^t + r_{d}^t + r_{p}^t + r_{m}^t" />|
   
   | reward    | description | value |
   | --------- | ----------- | ----- |
   | <img src="https://latex.codecogs.com/gif.latex?r_{s}^t" title="r_{s}^t" /> | success reward | <img src="https://latex.codecogs.com/gif.latex?r_{s}^t&space;=\begin{cases}15&space;&&space;goal\:reached\\0&space;&&space;otherwise\end{cases}" title="r_{s}^t =\begin{cases}15 & goal\:reached\\0 & otherwise\end{cases}" />
   | <img src="https://latex.codecogs.com/png.latex?r_{c}^t" title="r_{c}^t" /> | collision reward |<img src="https://latex.codecogs.com/png.latex?r_{c}^t&space;=\begin{cases}0&space;&&space;otherwise\\-10&space;&&space;agent\:collides\end{cases}" title="r_{c}^t =\begin{cases}0 & otherwise\\-10 & agent\:collides\end{cases}" />
   | <img src="https://latex.codecogs.com/png.latex?r_{d}^t" title="r_{d}^t" /> | danger reward | <img src="https://latex.codecogs.com/png.latex?r_{d}^t&space;=\begin{cases}0&space;&&space;otherwise\\-0.15&space;&&space;agent\:oversteps\:safe\:dist\end{cases}" title="r_{d}^t =\begin{cases}0 & otherwise\\-0.15 & agent\:oversteps\:safe\:dist\end{cases}" />
   | <img src="https://latex.codecogs.com/png.latex?r_{p}^t" title="r_{p}^t" />| progress reward | <img src="https://latex.codecogs.com/png.latex?r_{d}^t&space;=\begin{cases}w_{p}*d^t&space;&&space;d^t>=0,\;d^t=d_{ag}^{t-1}-d_{ag}^{t}\\w_{n}*d^t&space;&&space;else\end{cases}\\&space;w_{p}&space;=&space;0.25&space;\quad&space;w_{n}&space;=&space;0.4&space;\quad&space;d_{ag}:agent\:goal&space;\:&space;distance" title="r_{d}^t =\begin{cases}w_{p}*d^t & d^t>=0,\;d^t=d_{ag}^{t-1}-d_{ag}^{t}\\w_{n}*d^t & else\end{cases}\\ w_{p} = 0.25 \quad w_{n} = 0.4 \quad d_{ag}:agent\:goal \: distance" />(*)
   | <img src="https://latex.codecogs.com/png.latex?r_{m}^t" title="r_{m}^t" />| move reward | <img src="https://latex.codecogs.com/png.latex?r_{d}^t&space;=\begin{cases}0&space;&&space;otherwise\\-0.01&space;&&space;agent\:stands\:still\end{cases}" title="r_{d}^t =\begin{cases}0 & otherwise\\-0.01 & agent\:stands\:still\end{cases}" />

   *_higher weight applied if robot drives away from goal (to avoid driving unneccessary circles)_
   </td>
</tr>
</table>


#### 4.1.4 Training Curriculum

For the purpose of speeding up the training an exemplary training currucilum was implemented. But what exactly is a training curriculum you may ask. We basically divide the training process in difficulty levels, here the so called _stages_, in which the agent will meet an arbitrary number of obstacles depending on its learning progress. Different metrics can be taken into consideration to measure an agents performance.

In our implementation a reward threshold or a certain percentage of successful episodes must be reached to trigger the next stage. The statistics of each evaluation run is calculated and considered. Moreover when a new best mean reward was reached the model will be saved automatically.

Exemplary training curriculum:
| Stage           | Static Obstacles | Dynamic Obstacles  |
| :-------------: | :--------------: | ------------------ |
| 1               |  0               | 0                  |
| 2               |  10              | 0                  |
| 3               |  20              | 0                  |
| 4               |  0               | 10                 |
| 5               |  10              | 10                 |
| 6               |  13              | 13                 |

#### 4.1.5 Important Directories

|Path|Description|
|-----|-----|
|```../arena_local_planner_drl/agents```| models and associated hyperparameters.json will be saved to and loaded from here ([uniquely named directory](#load-a-dnn-for-training)) 
|```../arena_local_planner_drl/configs```| yaml files containing robots action spaces and the training curriculum
|```../arena_local_planner_drl/training_logs```| tensorboard logs and evaluation logs
|```../arena_local_planner_drl/scripts```| python file containing the predefined DNN architectures and the training script
