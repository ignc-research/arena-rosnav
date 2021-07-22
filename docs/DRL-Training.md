# DRL Agent Training

As a fundament for our Deep Reinforcement Learning approaches [StableBaselines3](https://stable-baselines3.readthedocs.io/en/master/index.html) was used. 

**Features included so far:**
* Simple handling of the training script through program parameters
* Choose a predefined Deep Neural Network
* Create your own custom Multilayer Perceptron via program parameters
* Networks will get trained, evaluated and saved
* Load your trained agent to continue training
* Optionally log training and evaluation data
* Enable and modify a custom training curriculum
* Multiprocessed rollout collection for training

**Table of Contents**
- [DRL Agent Training](#drl-agent-training)
    - [Quick Start](#quick-start)
    - [Training Script](#training-script)
      - [Examples](#examples)
        - [Training with a predefined DNN](#training-with-a-predefined-dnn)
        - [Load a DNN for training](#load-a-dnn-for-training)
        - [Training with a custom MLP](#training-with-a-custom-mlp)
      - [Multiprocessed Training](#multiprocessed-training)
    - [Hyperparameters](#hyperparameters)
    - [Reward Functions](#reward-functions)
    - [Training Curriculum](#training-curriculum)
    - [Run the trained Agent](#run-the-trained-agent)
      - [Sequential Evaluation of multiple Agents](#sequential-evaluation-of-multiple-agents)
    - [Important Directories](#important-directories)

### Quick Start

* In one terminnal, start simulation

```bash
roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode:=random
```
* In another terminal

```bash
workon rosnav
roscd arena_local_planner_drl
python scripts/training/train_agent.py --agent MLP_ARENA2D
```


### Training Script

**Generic program call**:
```
train_agent.py [agent flag] [agent_name | unique_agent_name | custom mlp params] [optional flag] [optional flag] ...
```

| Program call         | Agent Flag (mutually exclusive)  | Usage                                                           |Description                                         |
| -------------------- | -------------------------------- |---------------------------------------------------------------- |--------------------------------------------------- | 
| ``` train_agent.py```|```--agent```                     | *agent_name* ([see below](#training-with-a-predefined-dnn))   | initializes a predefined network from scratch
|                      |```--load ```                     | *unique_agent_name* ([see below](#load-a-dnn-for-training))   | loads agent to the given name
|                      |```--custom-mlp```                | _custom_mlp_params_ ([see below](#training-with-a-custom-mlp))| initializes custom MLP according to given arguments 
|                      |```--num_envs```            | *integer* | number of environments to collect experiences from for training (for more information on multiprocessed training, refer to [Multiprocessed Training](#multiprocessed-training))

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
[architecture]_[year]_[month]__[hour]_[minute]
```

To load a specific agent you simply use the flag ```--load```, e.g.:
```
train_agent.py --load MLP_ARENA2D_2021_01_19__03_20
```
**Note**: currently only agents which were trained with PPO given by StableBaselines3 are compatible with the training script. 

##### Training with a custom MLP

Instantiating a MLP architecture with an arbitrary number of layers and neurons for training was made as simple as possible by providing the option of using the ```--custom-mlp``` flag. By typing in the flag additional flags for the architecture of latent layers get accessible ([see above](#program-arguments)).

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


#### Multiprocessed Training
We provide for either testing and training purposes seperate launch scripts: 
- [start_arena_flatland.launch](/arena-rosnav/arena_bringup/launch/start_arena_flatland.launch) encapsulates the simulation environment featuring the different intermediate planners in a single process. Training is also possible within this simulation.
- [start_training.launch](../arena_bringup/launch/start_training.launch) depicts the slimer simulation version as we target a higher troughput here in order to be able to gather training data as fast as possible. The crucial feature of this launch file is that it is able to spawn an arbitrary number of environments to collect the rollouts with and thus allows for significant speedup through asynchronicity.

**First terminal: Simulation**
The first terminal is needed to run arena.

Run these commands:
```shell                      
workon rosnav
roslaunch arena_bringup start_training.launch train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4
```

**Second terminal: Training script**
A second terminal is needed to run the training script.

- Run these four commands:
```shell                 
workon rosnav
roscd arena_local_planner_drl
```

- Now, run one of the two commands below to start a training session:
```shell
python scripts/training/train_agent.py --load pretrained_ppo_mpc --n_envs 4 --eval_log 
python scripts/training/train_agent.py --load pretrained_ppo_baseline --n_envs 4 --eval_log
```

**Note**: Please inform yourself how many cores are provided by your processor in order to fully leverage local computing capabilities.

**Third terminal: Visualization**
A third terminal is needed in order to start rviz for visualization.

- Run this command:
```shell
roslaunch arena_bringup visualization_training.launch ns:=*ENV NAME*
```

**Note**: The training environments start with prefix *sim_* and end with the index. For example: *sim_1*, *sim_2* and so on. The evaluation environment which is used during the periodical benchmarking in training can be shown with ```ns:=eval_sim```.

**Ending a training session**

When the training script is done, it will print the following information and then exit:
```
Time passed: {time in seconds}s
Training script will be terminated
```

### Hyperparameters

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
| task_mode | Mode tasks will be generated in (custom, random, staged). In custom mode one can place obstacles manually via Rviz. In random mode there's a fixed number of obstacles which are spawned randomly distributed on the map after each episode. In staged mode the training curriculum will be used to spawn obstacles. ([more info](#training-curriculum))
| curr_stage | When "staged" training is activated which stage to start the training with.

([more information on PPO implementation of SB3](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html))

**Note**: For now further parameters like _max_steps_per_episode_ or _goal_radius_ have to be changed inline (where FlatlandEnv gets instantiated). _n_eval_episodes_ which will take place after _eval_freq_ timesteps can be changed also (where EvalCallback gets instantiated).

### Reward Functions

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

### Training Curriculum

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

For an explicit example, [click here](/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/configs/training_curriculum_map1small.yaml).

### Run the trained Agent

Now that you've trained your agent you surely want to deploy and evaluate it. For that purpose we've implemented a specific task mode in which you can specify your scenarios in a .json file. The agent will then be challenged according to the scenarios defined in the file. Please refer to https://github.com/ignc-research/arena-scenario-gui/ in order to read about the process of creating custom scenarios.

As with the training script, one can start the testing simulation environment with either one of two launch scripts:
- [start_arena_flatland.launch](/arena-rosnav/arena_bringup/launch/start_arena_flatland.launch): 
  - Allows for evaluation in continuous simulation time (emulates real time) as well as in controlled time stepping with four different subgoal modes, consisting of 3 intermediate planner approaches (*spatial horizon, timed A-star, simple sample*) with the DRL agent acting as the local planner.
  - Episode information can be logged via *rosbag* (refer to [document](/arena-rosnav/docs/eval_25042021.md))
- [start_training.launch](../arena_bringup/launch/start_training.launch):
  - Starts an evaluation environment in continuous simulation time (emulates real time) as well as in controlled time stepping with either the spatial horizon intermediate planner or the end goal being the only subgoal.
  - One can test multiple agents sequentially with *run_script.py*. This feature is only realized with this launch file, as *start_arena_flatland.launch* starts an own plan manager which interfers with the plan manager of the run script. Both plan managers have their own goal radius and thus might detect an end of episode differently. This can mess up the logged statistics.
  - Episode information can optionally be logged in a csv file by setting the ```--log``` flag for the run script dedicated plan manager to control the episodes.
  
</br>

Firstly, you need to start the *simulation environment*:
```shell
# Start the simulation with one of the launch files
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1"  disable_scenario:="false" scenario_file:="eval/obstacle_map1_obs20.json"

roslaunch arena_bringup start_training.launch num_envs:=1 map_folder_name:=map1 train_mode:=false
```
**Note**:
- The ```train_mode``` parameter determines if the simulation will run in emulated real time (where ```step_size``` and ```update_rate``` determine the simulation speed) or in the manipulating time stepping modus (via */step_world* rostopic).

</br>

Then, run the ```run_agent.py``` script with the desired scenario file:
```shell
python run_agent.py --load DRL_LOCAL_PLANNER_2021_03_22__19_33 --scenario obstacle_map1_obs20
```

**Generic program call**:
```
roscd arena_local_planner_drl/scripts/deployment/
run_agent.py --load [agent_name] -s [scenario_name] -v [number] [optional flag]
```

| Program call         | Flags                            | Usage                                 |Description                                         |
| -------------------- | -------------------------------- |-------------------------------------- |--------------------------------------------------- | 
| ```run_agent.py```   |```--load ```                     | *agent_name* ([see below](#load-a-dnn-for-training))     | loads agent to the given name
|                      |```-s``` or ```--scenario```      | *scenario_name* (as in *../scenario/eval/*)                       | loads the scenarios to the given .json file name
|                      |(optional)```-v``` or ```--verbose```| *0 or 1*                              | verbose level
|                      |(optional) ```--no-gpu```           | *None*                                | disables the gpu for the evaluation
| |(optional) ```--num_eps``` | *Integer*, defaults to 100 | number of episodes the agent/s get/s challenged |
| | (optional) ```--max_steps```| *Integer*, defaults to np.inf | max amount of actions per episode, before the simulation is resetted automatically |


- Example call:
``` 
python run_agent.py --load DRL_LOCAL_PLANNER_2021_03_22__19_33 -s obstacle_map1_obs20
```
**Notes**: 
- The ```--log``` flag should only be set with *start_training.launch* as simulation launcher as it requires the dedicated plan manager to control the episodes in order to log correct statistics.
- *When running start_arena_flatland.launch*: Make sure that ```drl_mode``` is activated in [plan_fsm_param.yaml](../arena_bringup/launch/plan_fsm_param.yaml)
- Make sure that the simulation speed doesn't overlap the agent's action calculation time (an obvious indicator: same action gets published multiple times successively and thus the agent moves unreasonably)
- If your agent was trained with normalized observations, it's necessary to provide the *vec_normalize.pkl* 

#### Sequential Evaluation of multiple Agents
For the automatic testing of several agents in a single call, one can specify a list containing an arbitrary number of agent names in [run_script.py](/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/deployment/run_agent.py). 

**Note**: 
- Guaranteed execution of each agent is currently only provided with the *start_training.launch* as simulation launcher 
- ```--load``` flag has to be set *None*, otherwise the script will only consider the agent provided with the flag.

### Important Directories

|Path|Description|
|-----|-----|
|```../arena_local_planner_drl/agents```| models and associated hyperparameters.json will be saved to and loaded from here ([uniquely named directory](#load-a-dnn-for-training)) 
|```../arena_local_planner_drl/configs```| yaml files containing robots action spaces and the training curriculum
|```../arena_local_planner_drl/training_logs```| tensorboard logs and evaluation logs
|```../arena_local_planner_drl/scripts```| python file containing the predefined DNN architectures and the training script

