## DRL Agent Training

As a fundament for our Deep Reinforcement Learning approaches [StableBaselines3](https://stable-baselines3.readthedocs.io/en/master/index.html) was used. 

##### Features included so far
* Simple handling of the training script through program parameters
* Choose a predefined Deep Neural Network
* Create your own custom Multilayer Perceptron via program parameters
* Networks will get trained, evaluated and saved
* Load your trained agent to continue training
* Optionally log training and evaluation data
* Enable and modify training curriculuu


##### Quick Start

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


#### Program Arguments

**Generic program call**:
```
train_agent.py [agent flag] [agent_name | unique_agent_name | custom mlp params] [optional flag] [optional flag] ...
```

| Program call         | Agent Flag (mutually exclusive)  | Usage                                                           |Description                                         |
| -------------------- | -------------------------------- |---------------------------------------------------------------- |--------------------------------------------------- | 
| ``` train_agent.py```|```--agent```                     | *agent_name* ([see below](#training-with-a-predefined-dnn))   | initializes a predefined network from scratch
|                      |```--load ```                     | *unique_agent_name* ([see below](#load-a-dnn-for-training))   | loads agent to the given name
|                      |```--custom-mlp```                | _custom_mlp_params_ ([see below](#training-with-a-custom-mlp))| initializes custom MLP according to given arguments 

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

#### Hyperparameters

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

#### Reward Functions

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

#### Training Curriculum

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

#### Run the trained agent

Now that you've trained your agent you surely want to deploy and evaluate it. For that purpose we've implemented a specific task mode in which you can specify your scenarios in a .json file. The agent will then be challenged according to the scenarios defined in the file. (*TODO: link zum scenario mode readme*).  

- Firstly, you need to start the *simulation environment*:
```
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1"  disable_scenario:="false" scenario_file:="eval/obstacle_map1_obs20.json"
```

- Then, start the *time-space plan manager*:
```
roslaunch arena_bringup timed_space_planner_fsm.launch
```

- Afterwards, start the *action publisher*:
```
roscd arena_local_planner_drl/scripts/deployment/
python action_publisher.py
```

- Then run the ```run_agent.py``` script.
```python run_agent.py --load DRL_LOCAL_PLANNER_2021_03_22__19_33 --scenario obstacle_map1_obs20```

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



- Example call:
``` 
python run_agent.py --load DRL_LOCAL_PLANNER_2021_03_22__19_33 -s obstacle_map1_obs20
```
**Notes**: 
- Make sure that drl mode is activated in *Parameter.yaml* (*../arena-rosnav/arena_bringup/launch*)
- Make sure that the simulation speed doesn't overlap the agent's calculation time for an action (an obvious indicator: same action gets published multiple times successively)
- If your agent was trained with normalized observations it's necessary to provide the vec_normalize.pkl 


#### Important Directories

|Path|Description|
|-----|-----|
|```../arena_local_planner_drl/agents```| models and associated hyperparameters.json will be saved to and loaded from here ([uniquely named directory](#load-a-dnn-for-training)) 
|```../arena_local_planner_drl/configs```| yaml files containing robots action spaces and the training curriculum
|```../arena_local_planner_drl/training_logs```| tensorboard logs and evaluation logs
|```../arena_local_planner_drl/scripts```| python file containing the predefined DNN architectures and the training script


#### Evaluation
Firstly, you need to start the *simulation environment*:
```
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1"  disable_scenario:="false"
```

Then, start the *time-space plan manager*:
```
roslaunch arena_bringup timed_space_planner_fsm.launch
```

Afterwards start the *action publisher*:
```
roscd arena_local_planner_drl/scripts/deployment/
python action_publisher.py
```

Now 
