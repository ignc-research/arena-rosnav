## DRL Agent Training

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


#### Program Arguments

**Generic program call**:
```
train_agent.py [agent flag] [agent_name | unique_agent_name | custom mlp params] [optional flag] [optional flag] ...
```

| Program call         | Agent Flag (mutually exclusive)  | Usage                                                           |Description                                         |
| -------------------- | -------------------------------- |---------------------------------------------------------------- |--------------------------------------------------- | 
| ``` train_agent.py```|```--agent```                     | [*agent_name*] ([see below](#training-with-a-predefined-dnn))   | initializes a predefined network from scratch
|                      |```--load ```                     | [*unique_agent_name*] ([see below](#load-a-dnn-for-training))   | loads agent to the given name
|                      |```--custom-mlp```                | [_custom mlp params_] ([see below](#training-with-a-custom-mlp))| initializes custom MLP according to given arguments 

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
   | <img src="https://bit.ly/2MdUJXp" align="center" border="0" alt="r^t = r_{s}^t + r_{c}^t + r_{d}^t + r_{p}^t + r_{m}^t" width="201" height="28" /> |
   
   | reward    | description | value |
   | --------- | ----------- | ----- |
   | <img src="https://bit.ly/3pmKoHa" align="center" border="0" alt=" r_{s}^t" width="18" height="26" /> | success reward | <img src="https://bit.ly/2LW8pGO" align="center" border="0" alt="r_{s}^t =\begin{cases}15 & goal \: reached \\0 & otherwise\end{cases} " width="204" height="50" />
   | <img src="https://bit.ly/3645pit" align="center" border="0" alt=" r_{c}^t" width="18" height="26" /> | collision reward | <img src="https://bit.ly/3c4W4KT" align="center" border="0" alt=" r_{m}^t =\begin{cases}0 & otherwise\\-10 & agent\:collides\end{cases} " width="231" height="50" />
   | <img src="https://bit.ly/3qLaMeo" align="center" border="0" alt=" r_{d}^t" width="21" height="26" /> | danger reward | <img src="https://bit.ly/362XG46" align="center" border="0" alt="r_{d}^t =\begin{cases}0 & otherwise \\-0.15 & agent\: breaks\: safe\: dist\end{cases} " width="307" height="50" />
   | <img src="https://bit.ly/3sSepRI" align="center" border="0" alt=" r_{p}^t" width="19" height="28" /> | progress reward | <img src="https://bit.ly/367LfE6" align="center" border="0" alt="r_{p}^t =\begin{cases} w* d^t   &  0 < d_{ag}^{t-1} - d_{ag}^{t}\; or \; 0 > d_{ag}^{t-1} - d_{ag}^{t} \\ 0 & else \end{cases} \\w = 0.25 \\d_{ag}: distance\:agent\:and\:goal " width="389" height="99" />
   | <img src="https://bit.ly/364H9fX" align="center" border="0" alt=" r_{m}^t" width="24" height="25" /> | move reward | <img src="https://bit.ly/3pci0HI" align="center" border="0" alt=" r_{m}^t =\begin{cases}0 & otherwise\\-0.01 & agent\:stands\:still\end{cases} " width="275" height="50" />

   </td>
   <td>

   | Reward Function at timestep t                                     |  
   | ----------------------------------------------------------------- |   
   | <img src="https://bit.ly/2MdUJXp" align="center" border="0" alt="r^t = r_{s}^t + r_{c}^t + r_{d}^t + r_{p}^t + r_{m}^t" width="201" height="28" /> |
   
   | reward    | description | value |
   | --------- | ----------- | ----- |
   | <img src="https://bit.ly/3pmKoHa" align="center" border="0" alt=" r_{s}^t" width="18" height="26" /> | success reward | <img src="https://bit.ly/2LW8pGO" align="center" border="0" alt="r_{s}^t =\begin{cases}15 & goal \: reached \\0 & otherwise\end{cases} " width="204" height="50" />
   | <img src="https://bit.ly/3645pit" align="center" border="0" alt=" r_{c}^t" width="18" height="26" /> | collision reward | <img src="https://bit.ly/3c4W4KT" align="center" border="0" alt=" r_{m}^t =\begin{cases}0 & otherwise\\-10 & agent\:collides\end{cases} " width="231" height="50" />
   | <img src="https://bit.ly/3qLaMeo" align="center" border="0" alt=" r_{d}^t" width="21" height="26" /> | danger reward | <img src="https://bit.ly/362XG46" align="center" border="0" alt="r_{d}^t =\begin{cases}0 & otherwise \\-0.15 & agent\: beaks\: safe\: dist\end{cases} " width="307" height="50" />
   | <img src="https://bit.ly/3sSepRI" align="center" border="0" alt=" r_{p}^t" width="19" height="28" /> | progress reward | <img src="https://bit.ly/2NuPm6U" align="center" border="0" alt="r_{p}^t =\begin{cases} w_{p}* d^t   &  0 < d_{ag}^{t-1} - d_{ag}^{t} \\ w_{n}* d^t   &  0>d_{ag}^{t-1} - d_{ag}^{t} } \\ 0 & else \end{cases} \\w_{p} = 0.25\\ w_{n}=0.4 \\d_{ag}: distance\:agent\:and\:goal \\" width="258" height="151" /> (*)
   | <img src="https://bit.ly/364H9fX" align="center" border="0" alt=" r_{m}^t" width="24" height="25" /> | move reward | <img src="https://bit.ly/3pci0HI" align="center" border="0" alt=" r_{m}^t =\begin{cases}0 & otherwise\\-0.01 & agent\:stands\:still\end{cases} " width="275" height="50" />

   *higher weight applied if robot drives away from goal (to avoid driving unneccessary circles) 
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

#### Important Directories

|Path|Description|
|-----|-----|
|```../arena_local_planner_drl/agents```| models and associated hyperparameters.json will be saved to and loaded from here ([uniquely named directory](#load-a-dnn-for-training)) 
|```../arena_local_planner_drl/configs```| yaml files containing robots action spaces and the training curriculum
|```../arena_local_planner_drl/training_logs```| tensorboard logs and evaluation logs
|```../arena_local_planner_drl/scripts```| python file containing the predefined DNN architectures and the training script
