# DRL Agent Training

As a fundament for our Deep Reinforcement Learning approaches [StableBaselines3](https://stable-baselines3.readthedocs.io/en/master/index.html) was used.

**Features included so far:**

- Simple handling of the training script through program parameters
- Choose a predefined Deep Neural Network
- Create your own custom Multilayer Perceptron via program parameters
- Networks will get trained, evaluated and saved
- Load your trained agent to continue training
- Optionally log training and evaluation data
- Enable and modify a custom training curriculum
- Multiprocessed rollout collection for training

**Table of Contents**

- [DRL Agent Training](#drl-agent-training)
    - [Quick Start](#quick-start)
    - [Training Script](#training-script)
      - [Usage](#usage)
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

- In one terminnal, start the arena simulation:

```bash
roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode:=random
```

- In a second terminal, run the train script:

```bash
workon rosnav
roscd arena_local_planner_drl
python scripts/training/train_agent.py --agent MLP_ARENA2D
```

### Training Script

#### Usage

**Generic program call**:

```
train_agent.py [agent flag] [agent_name | unique_agent_name | custom mlp params] [optional flag] [optional flag] ...
```

| Program call      | Agent Flag (mutually exclusive) | Usage                                                          | Description                                         |
| ----------------- | ------------------------------- | -------------------------------------------------------------- | --------------------------------------------------- |
| ` train_agent.py` | `--agent`                       | _agent_name_ ([see below](#training-with-a-predefined-dnn))    | initializes a predefined network from scratch       |
|                   | `--load `                       | _unique_agent_name_ ([see below](#load-a-dnn-for-training))    | loads agent to the given name                       |
|                   | `--custom-mlp`                  | _custom_mlp_params_ ([see below](#training-with-a-custom-mlp)) | initializes custom MLP according to given arguments |

_Custom Multilayer Perceptron_ parameters will only be considered when `--custom-mlp` was set!
| Custom Mlp Flags | Syntax | Description |
| ---------------- | ----------------------| ----------------------------------------------|
| `--body ` | `{num}-{num}-...` |architecture of the shared latent network |
| `--pi` | `{num}-{num}-...` |architecture of the latent policy network |
| `--vf` | `{num}-{num}-...` |architecture of the latent value network |
| `--act_fn ` | `{relu, sigmoid or tanh}`|activation function to be applied after each hidden layer |

| Optional Flags                               | Description                                                                                                                                                                                                                                     |
| -------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--config {string}`, defaults to _"default"_ | Looks for the given config file name in [../arena_local_planner_drl/configs/hyperparameters](/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/configs/hyperparameters) to load the configurations from |
| `--n {integer}`                              | timesteps in total to be generated for training                                                                                                                                                                                                 |
| `--tb`                                       | enables tensorboard logging                                                                                                                                                                                                                     |
| `-log`, `--eval_log`                         | enables logging of evaluation episodes                                                                                                                                                                                                          |
| `--no-gpu`                                   | disables training with GPU                                                                                                                                                                                                                      |
| `--num_envs {integer}`                       | number of environments to collect experiences from for training (for more information refer to [Multiprocessed Training](#multiprocessed-training))                                                                                             |

#### Examples

##### Training with a predefined DNN

Currently you can choose between several different Deep Neural Networks each of which have been object of research projects, for example:

| Agent name        | Inspired by                                                                           |
| ----------------- | ------------------------------------------------------------------------------------- |
| MLP_ARENA2D       | [arena2D](https://github.com/ignc-research/arena2D)                                   |
| DRL_LOCAL_PLANNER | [drl_local_planner](https://github.com/RGring/drl_local_planner_ros_stable_baselines) |
| CNN_NAVREP        | [NavRep](https://github.com/ethz-asl/navrep)                                          |

e.g. training with the MLP architecture from arena2D:

```
train_agent.py --agent MLP_ARENA2D
```

You can find the most recently implemented neural network architectures in: [custom_policy.py](/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/custom_policy.py)

##### Load a DNN for training

In order to differentiate between agents with similar architectures but from different runs a unique agent name will be generated when using either `--agent` or `--custom-mlp` mode (when train from scratch).

The name consists of:

```
[architecture]_[year]_[month]__[hour]_[minute]
```

To load a specific agent you simply use the flag `--load`, e.g.:

```
train_agent.py --load MLP_ARENA2D_2021_01_19__03_20
```

**Note**: currently only agents which were trained with PPO given by StableBaselines3 are compatible with the training script.

##### Training with a custom MLP

Instantiating a MLP architecture with an arbitrary number of layers and neurons for training was made as simple as possible by providing the option of using the `--custom-mlp` flag. By typing in the flag additional flags for the architecture of latent layers get accessible ([see above](#program-arguments)).

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

**Note**: The training environments start with prefix \_sim\__ and end with the index. For example: \_sim_1_, _sim_2_ and so on. The evaluation environment which is used during the periodical benchmarking in training can be shown with `ns:=eval_sim`.

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
| n*steps | The number of steps to run for each environment per update
| ent_coef | Entropy coefficient for the loss calculation
| learning_rate | The learning rate, it can be a function of the current progress remaining (from 1 to 0) (i.e. batch size is n_steps \* n_env where n_env is number of environment copies running in parallel)
| vf_coef | Value function coefficient for the loss calculation
| max_grad_norm | The maximum value for the gradient clipping
| gae_lambda | Factor for trade-off of bias vs variance for Generalized Advantage Estimator
| batch_size | Minibatch size
| n_epochs | Number of epoch when optimizing the surrogate loss
| clip_range | Clipping parameter, it can be a function of the current progress remaining (from 1 to 0).
| reward_fnc | Number of the reward function (defined in *../rl*agent/utils/reward.py*)
| discrete_action_space | If robot uses discrete action space
| task_mode | Mode tasks will be generated in (custom, random, staged). In custom mode one can place obstacles manually via Rviz. In random mode there's a fixed number of obstacles which are spawned randomly distributed on the map after each episode. In staged mode the training curriculum will be used to spawn obstacles. ([more info](#training-curriculum))
| curr_stage | When "staged" training is activated which stage to start the training with.

([more information on PPO implementation of SB3](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html))

**Note**: For now further parameters like _max_steps_per_episode_ or _goal_radius_ have to be changed inline (where FlatlandEnv gets instantiated). _n_eval_episodes_ which will take place after _eval_freq_ timesteps can be changed also (where EvalCallback gets instantiated).

### Reward Functions

The reward functions are defined in: (alternatively, [click here](../arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/utils/reward.py))

```
../arena_local_planner_drl/rl_agent/utils/reward.py
```

At present, one can chose between five reward functions which can be set in the [hyperparameters yaml file](#hyperparameters):

<table>
<tr>
   <th>rule_00</th> <th>rule_01</th> <th>rule_02</th> <th>rule_03</th> <th>rule_04</th>
</tr>
<tr>

  <td>

| Reward Function at timestep t                                                                                                                                                                                                                   |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{00}^{t}&space;=&space;r_{s}^{t}&space;&plus;&space;r_{c}^{t}&space;&plus;&space;r_{d}^{t}&space;&plus;&space;r_{p}^{t}" title="r_{00}^{t} = r_{s}^{t} + r_{c}^{t} + r_{d}^{t} + r_{p}^{t}" /> |

| reward                                                                         | description      | value                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| ------------------------------------------------------------------------------ | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}" title="r_{s}^{t}" /> | success reward   | <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}&space;=&space;\begin{cases}&space;15&space;&&space;\text{&space;if&space;goal&space;reached}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{s}^{t} = \begin{cases} 15 & \text{ if goal reached} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}" title="r_{c}^{t}" /> | collision reward | <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}&space;=&space;\begin{cases}&space;-10&space;&&space;\text{&space;if&space;robot&space;collides}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{c}^{t} = \begin{cases} -10 & \text{ if robot collides} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}" title="r_{d}^{t}" /> | danger reward    | <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}&space;=&space;\begin{cases}&space;-0.25&space;&&space;\text{&space;if&space;}&space;\exists{o&space;\in&space;O}&space;:&space;d(p_{robot}^t,&space;p_{obs}^t)&space;<&space;D_{s}\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{d}^{t} = \begin{cases} -0.25 & \text{ if } \exists{o \in O} : d(p_{robot}^t, p_{obs}^t) < D_{s}\\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                          |
| <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}" title="r_{p}^{t}" /> | progress reward  | <img src="https://latex.codecogs.com/gif.latex?\text{diff}_{robot,x}^t&space;=&space;d(p_{robot}^{t-1},&space;p_{x}^{t-1})&space;-&space;d(p_{robot}^t,&space;p_{x}^t)" title="\text{diff}_{robot,x}^t = d(p_{robot}^{t-1}, p_{x}^{t-1}) - d(p_{robot}^t, p_{x}^t)" /> <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}&space;=&space;\begin{cases}&space;0.3&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;if&space;}&space;\text{diff}_{robot,goal}^t&space;>&space;0\\&space;0.4&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{p}^{t} = \begin{cases} 0.3 * \text{diff}_{robot,goal}^t & \text{ if } \text{diff}_{robot,goal}^t > 0\\ 0.4 * \text{diff}_{robot,goal}^t & \text{ otherwise } \end{cases}" /> |

  </td>

  <td>

| Reward Function at timestep t                                                                                                                                                                                                                                                              |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| <img src="https://latex.codecogs.com/gif.latex?r_{01}^{t}&space;=&space;r_{s}^{t}&space;&plus;&space;r_{c}^{t}&space;&plus;&space;r_{d}^{t}&space;&plus;&space;r_{p}^{t}&space;&plus;&space;r_{dt}^{t}" title="r_{01}^{t} = r_{s}^{t} + r_{c}^{t} + r_{d}^{t} + r_{p}^{t} + r_{dt}^{t}" /> |

| reward                                                                           | description               | value                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| -------------------------------------------------------------------------------- | ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}" title="r_{s}^{t}" />   | success reward            | <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}&space;=&space;\begin{cases}&space;15&space;&&space;\text{&space;if&space;goal&space;reached}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{s}^{t} = \begin{cases} 15 & \text{ if goal reached} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}" title="r_{c}^{t}" />   | collision reward          | <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}&space;=&space;\begin{cases}&space;-10&space;&&space;\text{&space;if&space;robot&space;collides}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{c}^{t} = \begin{cases} -10 & \text{ if robot collides} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}" title="r_{d}^{t}" />   | danger reward             | <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}&space;=&space;\begin{cases}&space;-0.25&space;&&space;\text{&space;if&space;}&space;\exists{o&space;\in&space;O}&space;:&space;d(p_{robot}^t,&space;p_{obs}^t)&space;<&space;D_{s}\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{d}^{t} = \begin{cases} -0.25 & \text{ if } \exists{o \in O} : d(p_{robot}^t, p_{obs}^t) < D_{s}\\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                          |
| <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}" title="r_{p}^{t}" />   | progress reward           | <img src="https://latex.codecogs.com/gif.latex?\text{diff}_{robot,x}^t&space;=&space;d(p_{robot}^{t-1},&space;p_{x}^{t-1})&space;-&space;d(p_{robot}^t,&space;p_{x}^t)" title="\text{diff}_{robot,x}^t = d(p_{robot}^{t-1}, p_{x}^{t-1}) - d(p_{robot}^t, p_{x}^t)" /> <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}&space;=&space;\begin{cases}&space;0.3&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;if&space;}&space;\text{diff}_{robot,goal}^t&space;>&space;0\\&space;0.4&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{p}^{t} = \begin{cases} 0.3 * \text{diff}_{robot,goal}^t & \text{ if } \text{diff}_{robot,goal}^t > 0\\ 0.4 * \text{diff}_{robot,goal}^t & \text{ otherwise } \end{cases}" /> |
| <img src="https://latex.codecogs.com/gif.latex?r_{dt}^{t}" title="r_{dt}^{t}" /> | distance travelled reward | <img src="https://latex.codecogs.com/gif.latex?r_{dt}^{t}&space;=&space;(vel_{linear}^{t}&space;&plus;&space;(vel_{angular}^{t}*0.001))*-0.0075" title="r_{dt}^{t} = (vel_{linear}^{t} + (vel_{angular}^{t}*0.001))*-0.0075" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

   </td>

  <td>

| Reward Function at timestep t                                                                                                                                                                                                                                                                                                         |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{02}^{t}&space;=&space;r_{s}^{t}&space;&plus;&space;r_{c}^{t}&space;&plus;&space;r_{d}^{t}&space;&plus;&space;r_{p}^{t}&space;&plus;&space;r_{dt}^{t}&space;&plus;&space;r_{fg}^{t}" title="r_{02}^{t} = r_{s}^{t} + r_{c}^{t} + r_{d}^{t} + r_{p}^{t} + r_{dt}^{t} + r_{fg}^{t}" /> |

| reward                                                                           | description                  | value                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| -------------------------------------------------------------------------------- | ---------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}" title="r_{s}^{t}" />   | success reward               | <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}&space;=&space;\begin{cases}&space;15&space;&&space;\text{&space;if&space;goal&space;reached}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{s}^{t} = \begin{cases} 15 & \text{ if goal reached} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}" title="r_{c}^{t}" />   | collision reward             | <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}&space;=&space;\begin{cases}&space;-10&space;&&space;\text{&space;if&space;robot&space;collides}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{c}^{t} = \begin{cases} -10 & \text{ if robot collides} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}" title="r_{d}^{t}" />   | danger reward                | <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}&space;=&space;\begin{cases}&space;-0.25&space;&&space;\text{&space;if&space;}&space;\exists{o&space;\in&space;O}&space;:&space;d(p_{robot}^t,&space;p_{obs}^t)&space;<&space;D_{s}\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{d}^{t} = \begin{cases} -0.25 & \text{ if } \exists{o \in O} : d(p_{robot}^t, p_{obs}^t) < D_{s}\\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                          |
| <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}" title="r_{p}^{t}" />   | progress reward              | <img src="https://latex.codecogs.com/gif.latex?\text{diff}_{robot,x}^t&space;=&space;d(p_{robot}^{t-1},&space;p_{x}^{t-1})&space;-&space;d(p_{robot}^t,&space;p_{x}^t)" title="\text{diff}_{robot,x}^t = d(p_{robot}^{t-1}, p_{x}^{t-1}) - d(p_{robot}^t, p_{x}^t)" /> <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}&space;=&space;\begin{cases}&space;0.3&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;if&space;}&space;\text{diff}_{robot,goal}^t&space;>&space;0\\&space;0.4&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{p}^{t} = \begin{cases} 0.3 * \text{diff}_{robot,goal}^t & \text{ if } \text{diff}_{robot,goal}^t > 0\\ 0.4 * \text{diff}_{robot,goal}^t & \text{ otherwise } \end{cases}" /> |
| <img src="https://latex.codecogs.com/gif.latex?r_{dt}^{t}" title="r_{dt}^{t}" /> | distance travelled reward    | <img src="https://latex.codecogs.com/gif.latex?r_{dt}^{t}&space;=&space;(vel_{linear}^{t}&space;&plus;&space;(vel_{angular}^{t}*0.001))*-0.0075" title="r_{dt}^{t} = (vel_{linear}^{t} + (vel_{angular}^{t}*0.001))*-0.0075" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| <img src="https://latex.codecogs.com/gif.latex?r_{fg}^{t}" title="r_{fg}^{t}" /> | following global plan reward | <img src="https://latex.codecogs.com/gif.latex?r_{fg}^{t}&space;=&space;\begin{cases}&space;\begin{aligned}&space;0.1&space;*&space;vel_{linear}^{t}&space;&&space;\text{&space;if&space;}&space;\min_{wp&space;\in&space;G}d(p_{wp}^t,&space;p_{r}^t)&space;<&space;0.5&space;\text{m}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{aligned}&space;\end{cases}" title="r_{fg}^{t} = \begin{cases} \begin{aligned} 0.1 * vel_{linear}^{t} & \text{ if } \min_{wp \in G}d(p_{wp}^t, p_{r}^t) < 0.5 \text{m} \\ 0 & \text{ otherwise } \end{aligned} \end{cases}" />                                                                                                                                                                                                                    |

  </td>

  <td>

| Reward Function at timestep t                                                                                                                                                                                                                                                                                                         |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{03}^{t}&space;=&space;r_{s}^{t}&space;&plus;&space;r_{c}^{t}&space;&plus;&space;r_{d}^{t}&space;&plus;&space;r_{p}^{t}&space;&plus;&space;r_{fg}^{t}&space;&plus;&space;r_{dg}^{t}" title="r_{03}^{t} = r_{s}^{t} + r_{c}^{t} + r_{d}^{t} + r_{p}^{t} + r_{fg}^{t} + r_{dg}^{t}" /> |

| reward                                                                           | description                   | value                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| -------------------------------------------------------------------------------- | ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}" title="r_{s}^{t}" />   | success reward                | <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}&space;=&space;\begin{cases}&space;15&space;&&space;\text{&space;if&space;goal&space;reached}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{s}^{t} = \begin{cases} 15 & \text{ if goal reached} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}" title="r_{c}^{t}" />   | collision reward              | <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}&space;=&space;\begin{cases}&space;-10&space;&&space;\text{&space;if&space;robot&space;collides}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{c}^{t} = \begin{cases} -10 & \text{ if robot collides} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}" title="r_{d}^{t}" />   | danger reward                 | <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}&space;=&space;\begin{cases}&space;-0.25&space;&&space;\text{&space;if&space;}&space;\exists{o&space;\in&space;O}&space;:&space;d(p_{robot}^t,&space;p_{obs}^t)&space;<&space;D_{s}\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{d}^{t} = \begin{cases} -0.25 & \text{ if } \exists{o \in O} : d(p_{robot}^t, p_{obs}^t) < D_{s}\\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}" title="r_{p}^{t}" />   | progress reward               | <img src="https://latex.codecogs.com/gif.latex?\text{diff}_{robot,x}^t&space;=&space;d(p_{robot}^{t-1},&space;p_{x}^{t-1})&space;-&space;d(p_{robot}^t,&space;p_{x}^t)" title="\text{diff}_{robot,x}^t = d(p_{robot}^{t-1}, p_{x}^{t-1}) - d(p_{robot}^t, p_{x}^t)" /> <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}&space;=&space;\begin{cases}&space;0.3&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;if&space;}&space;\text{diff}_{robot,goal}^t&space;>&space;0\\&space;0.4&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{p}^{t} = \begin{cases} 0.3 * \text{diff}_{robot,goal}^t & \text{ if } \text{diff}_{robot,goal}^t > 0\\ 0.4 * \text{diff}_{robot,goal}^t & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                  |
| <img src="https://latex.codecogs.com/gif.latex?r_{fg}^{t}" title="r_{fg}^{t}" /> | following global plan reward  | <img src="https://latex.codecogs.com/gif.latex?r_{fg}^{t}&space;=&space;\begin{cases}&space;\begin{aligned}&space;0.1&space;*&space;vel_{linear}^{t}&space;&&space;\text{&space;if&space;}&space;\min_{wp&space;\in&space;G}d(p_{wp}^t,&space;p_{r}^t)&space;<&space;0.5&space;\text{m}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{aligned}&space;\end{cases}" title="r_{fg}^{t} = \begin{cases} \begin{aligned} 0.1 * vel_{linear}^{t} & \text{ if } \min_{wp \in G}d(p_{wp}^t, p_{r}^t) < 0.5 \text{m} \\ 0 & \text{ otherwise } \end{aligned} \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| <img src="https://latex.codecogs.com/gif.latex?r_{dg}" title="r_{dg}" />         | distance to globalplan reward | <img src="https://latex.codecogs.com/gif.latex?r_{dg}&space;=&space;\begin{cases}&space;\begin{aligned}&space;0.2*&space;\text{diff}_{robot,&space;wp}^{t}&space;&&space;\text{&space;if&space;}\min_{wp&space;\in&space;G}d(p_{r}^t,&space;p_{wp}^t):&space;\text{diff}_{robot,&space;wp}^{t}&space;>&space;0&space;\\&space;0.3*&space;\text{diff}_{robot,&space;wp}^{t}&space;&&space;\text{&space;if&space;}&space;\min_{wp&space;\in&space;G}d(p_{r}^t,&space;p_{wp}^t):&space;\text{diff}_{robot,&space;wp}^{t}&space;<=&space;0&space;\\&space;0&space;&&space;\text{&space;if&space;}&space;\min_{o&space;\in&space;O}d(p_{r}^t,&space;p_{o}^t)&space;<&space;D_s&space;\end{aligned}&space;\end{cases}" title="r_{dg} = \begin{cases} \begin{aligned} 0.2* \text{diff}_{robot, wp}^{t} & \text{ if }\min_{wp \in G}d(p_{r}^t, p_{wp}^t): \text{diff}_{robot, wp}^{t} > 0 \\ 0.3* \text{diff}_{robot, wp}^{t} & \text{ if } \min_{wp \in G}d(p_{r}^t, p_{wp}^t): \text{diff}_{robot, wp}^{t} <= 0 \\ 0 & \text{ if } \min_{o \in O}d(p_{r}^t, p_{o}^t) < D_s \end{aligned} \end{cases}" /> |

  </td>

  <td>

| Reward Function at timestep t                                                                                                                                                                                                                                                                                                                                                    |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{04}^{t}&space;=&space;r_{s}^{t}&space;&plus;&space;r_{c}^{t}&space;&plus;&space;r_{d}^{t}&space;&plus;&space;r_{p}^{t}&space;&plus;&space;r_{fg}^{t}&space;&plus;&space;r_{dg}^{t}&space;&plus;&space;r_{dc}^{t}" title="r_{04}^{t} = r_{s}^{t} + r_{c}^{t} + r_{d}^{t} + r_{p}^{t} + r_{fg}^{t} + r_{dg}^{t} + r_{dc}^{t}" /> |

| reward                                                                           | description                   | value                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| -------------------------------------------------------------------------------- | ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ | --------------------------------------------- | ---------------------------------------------- | --------------- |
| <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}" title="r_{s}^{t}" />   | success reward                | <img src="https://latex.codecogs.com/gif.latex?r_{s}^{t}&space;=&space;\begin{cases}&space;15&space;&&space;\text{&space;if&space;goal&space;reached}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{s}^{t} = \begin{cases} 15 & \text{ if goal reached} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}" title="r_{c}^{t}" />   | collision reward              | <img src="https://latex.codecogs.com/gif.latex?r_{c}^{t}&space;=&space;\begin{cases}&space;-10&space;&&space;\text{&space;if&space;robot&space;collides}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{c}^{t} = \begin{cases} -10 & \text{ if robot collides} \\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}" title="r_{d}^{t}" />   | danger reward                 | <img src="https://latex.codecogs.com/gif.latex?r_{d}^{t}&space;=&space;\begin{cases}&space;-0.25&space;&&space;\text{&space;if&space;}&space;\exists{o&space;\in&space;O}&space;:&space;d(p_{robot}^t,&space;p_{obs}^t)&space;<&space;D_{s}\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{d}^{t} = \begin{cases} -0.25 & \text{ if } \exists{o \in O} : d(p_{robot}^t, p_{obs}^t) < D_{s}\\ 0 & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}" title="r_{p}^{t}" />   | progress reward               | <img src="https://latex.codecogs.com/gif.latex?\text{diff}_{robot,x}^t&space;=&space;d(p_{robot}^{t-1},&space;p_{x}^{t-1})&space;-&space;d(p_{robot}^t,&space;p_{x}^t)" title="\text{diff}_{robot,x}^t = d(p_{robot}^{t-1}, p_{x}^{t-1}) - d(p_{robot}^t, p_{x}^t)" /> <img src="https://latex.codecogs.com/gif.latex?r_{p}^{t}&space;=&space;\begin{cases}&space;0.3&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;if&space;}&space;\text{diff}_{robot,goal}^t&space;>&space;0\\&space;0.4&space;*&space;\text{diff}_{robot,goal}^t&space;&&space;\text{&space;otherwise&space;}&space;\end{cases}" title="r_{p}^{t} = \begin{cases} 0.3 * \text{diff}_{robot,goal}^t & \text{ if } \text{diff}_{robot,goal}^t > 0\\ 0.4 * \text{diff}_{robot,goal}^t & \text{ otherwise } \end{cases}" />                                                                                                                                                                                                                                                                                  |
| <img src="https://latex.codecogs.com/gif.latex?r_{fg}^{t}" title="r_{fg}^{t}" /> | following global plan reward  | <img src="https://latex.codecogs.com/gif.latex?r_{fg}^{t}&space;=&space;\begin{cases}&space;\begin{aligned}&space;0.1&space;*&space;vel_{linear}^{t}&space;&&space;\text{&space;if&space;}&space;\min_{wp&space;\in&space;G}d(p_{wp}^t,&space;p_{r}^t)&space;<&space;0.5&space;\text{m}&space;\\&space;0&space;&&space;\text{&space;otherwise&space;}&space;\end{aligned}&space;\end{cases}" title="r_{fg}^{t} = \begin{cases} \begin{aligned} 0.1 * vel_{linear}^{t} & \text{ if } \min_{wp \in G}d(p_{wp}^t, p_{r}^t) < 0.5 \text{m} \\ 0 & \text{ otherwise } \end{aligned} \end{cases}" />                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| <img src="https://latex.codecogs.com/gif.latex?r_{dg}" title="r_{dg}" />         | distance to globalplan reward | <img src="https://latex.codecogs.com/gif.latex?r_{dg}&space;=&space;\begin{cases}&space;\begin{aligned}&space;0.2*&space;\text{diff}_{robot,&space;wp}^{t}&space;&&space;\text{&space;if&space;}\min_{wp&space;\in&space;G}d(p_{r}^t,&space;p_{wp}^t):&space;\text{diff}_{robot,&space;wp}^{t}&space;>&space;0&space;\\&space;0.3*&space;\text{diff}_{robot,&space;wp}^{t}&space;&&space;\text{&space;if&space;}&space;\min_{wp&space;\in&space;G}d(p_{r}^t,&space;p_{wp}^t):&space;\text{diff}_{robot,&space;wp}^{t}&space;<=&space;0&space;\\&space;0&space;&&space;\text{&space;if&space;}&space;\min_{o&space;\in&space;O}d(p_{r}^t,&space;p_{o}^t)&space;<&space;D_s&space;\end{aligned}&space;\end{cases}" title="r_{dg} = \begin{cases} \begin{aligned} 0.2* \text{diff}_{robot, wp}^{t} & \text{ if }\min_{wp \in G}d(p_{r}^t, p_{wp}^t): \text{diff}_{robot, wp}^{t} > 0 \\ 0.3* \text{diff}_{robot, wp}^{t} & \text{ if } \min_{wp \in G}d(p_{r}^t, p_{wp}^t): \text{diff}_{robot, wp}^{t} <= 0 \\ 0 & \text{ if } \min_{o \in O}d(p_{r}^t, p_{o}^t) < D_s \end{aligned} \end{cases}" /> |
| <img src="https://latex.codecogs.com/gif.latex?r_{dc}^t" title="r_{dc}^t" />     | direction change reward       | <img src="https://latex.codecogs.com/gif.latex?r_{dc}^t&space;=&space;-&space;\frac{\left&space;|&space;vel_{angular}^{t-1}&space;-&space;vel_{angular}^{t}&space;\right&space;|^{4}}{2500}" title="r_{dc}^t = - \frac{\left | vel_{angular}^{t-1} - vel_{angular}^{t} \right |^{4}}{2500}" /> |

  </td>

</tr>
</table>

### Training Curriculum

For the purpose of speeding up the training an exemplary training currucilum was implemented. But what exactly is a training curriculum you may ask. We basically divide the training process in difficulty levels, here the so called _stages_, in which the agent will meet an arbitrary number of obstacles depending on its learning progress. Different metrics can be taken into consideration to measure an agents performance.

In our implementation a reward threshold or a certain percentage of successful episodes must be reached to trigger the next stage. The statistics of each evaluation run is calculated and considered. Moreover when a new best mean reward was reached the model will be saved automatically.

Exemplary training curriculum:
| Stage | Static Obstacles | Dynamic Obstacles |
| :-------------: | :--------------: | :-----------------: |
| 1 | 0 | 0 |
| 2 | 10 | 0 |
| 3 | 20 | 0 |
| 4 | 0 | 10 |
| 5 | 10 | 10 |
| 6 | 13 | 13 |

For an explicit example, [click here](/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/configs/training_curriculum_map1small.yaml).

### Run the trained Agent

Now that you've trained your agent you surely want to deploy and evaluate it. For that purpose we've implemented a specific task mode in which you can specify your scenarios in a .json file. The agent will then be challenged according to the scenarios defined in the file. Please refer to https://github.com/ignc-research/arena-scenario-gui/ in order to read about the process of creating custom scenarios.
Moreover, you can test your agent on custom maps in randomly generated scenarios with a predefined number of dynamic obstacles.

As with the training script, one can start the testing simulation environment with either one of two launch scripts:

- [start_arena_flatland.launch](/arena-rosnav/arena_bringup/launch/start_arena_flatland.launch):
  - Allows for evaluation in continuous simulation time (emulates real time) as well as in controlled time stepping with four different subgoal modes, consisting of 3 intermediate planner approaches (_spatial horizon, timed A-star, simple sample_) with the DRL agent acting as the local planner.
  - Episode information can be logged via _rosbag_ (refer to [document](/arena-rosnav/docs/eval_25042021.md))
- [start_training.launch](../arena_bringup/launch/start_training.launch):
  - Starts an evaluation environment in continuous simulation time (emulates real time) as well as in controlled time stepping with either the spatial horizon intermediate planner or the end goal being the only subgoal.
  - One can test multiple agents sequentially with _run_script.py_. This feature is only realized with this launch file, as _start_arena_flatland.launch_ starts an own plan manager which interfers with the plan manager of the run script. Both plan managers have their own goal radius and thus might detect an end of episode differently. This can mess up the logged statistics.
  - Episode information can optionally be logged in a csv file by setting the `--log` flag for the run script dedicated plan manager to control the episodes.

</br>

Firstly, you need to start the _simulation environment_:

```shell
# Start the simulation with one of the launch files
roslaunch arena_bringup start_arena_flatland.launch map_file:="map1"  disable_scenario:="false" scenario_file:="eval/obstacle_map1_obs20.json"

roslaunch arena_bringup start_training.launch num_envs:=1 map_folder_name:=map1 train_mode:=false
```

**Note**:

- The `train_mode` parameter determines if the simulation will run in emulated real time (where `step_size` and `update_rate` determine the simulation speed) or in the manipulating time stepping modus (via _/step_world_ rostopic).

</br>

Then, run the `run_agent.py` script with the desired scenario file:

```shell
python run_agent.py --load DRL_LOCAL_PLANNER_2021_03_22__19_33 --scenario obstacle_map1_obs20
```

**Generic program call**:

```
roscd arena_local_planner_drl/scripts/deployment/
run_agent.py --load [agent_name] -s [scenario_name] -v [number] [optional flag]
```

| Program call   | Flags                         | Usage                                                | Description                                                                     |
| -------------- | ----------------------------- | ---------------------------------------------------- | ------------------------------------------------------------------------------- |
| `run_agent.py` | `--load `                     | _agent_name_ ([see below](#load-a-dnn-for-training)) | loads agent to the given name                                                   |
|                | `-s` or `--scenario`          | _scenario_name_ (as in _../scenario/eval/_)          | loads the scenarios to the given .json file name                                |
|                | (optional)`-v` or `--verbose` | _0 or 1_                                             | verbose level                                                                   |
|                | (optional) `--no-gpu`         | _None_                                               | disables the gpu for the evaluation                                             |
|                | (optional) `--num_eps`        | _Integer_, defaults to 100                           | number of episodes the agent/s get/s challenged                                 |
|                | (optional) `--max_steps`      | _Integer_, defaults to np.inf                        | max amount of actions per episode, before the episode is resetted automatically |

- Example call:

```
python run_agent.py --load DRL_LOCAL_PLANNER_2021_03_22__19_33 -s obstacle_map1_obs20
```

**Notes**:

- The `--log` flag should only be set with _start_training.launch_ as simulation launcher as it requires the dedicated plan manager to control the episodes in order to log correct statistics.
- _When running start_arena_flatland.launch_: Make sure that `drl_mode` is activated in [plan_fsm_param.yaml](../arena_bringup/launch/plan_fsm_param.yaml)
- Make sure that the simulation speed doesn't overlap the agent's action calculation time (an obvious indicator: same action gets published multiple times successively and thus the agent moves unreasonably)
- If your agent was trained with normalized observations, it's necessary to provide the _vec_normalize.pkl_

#### Sequential Evaluation of multiple Agents

For automatic testing of several agents in a sequence, one can specify a list containing an arbitrary number of agent names in [run_script.py](/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/deployment/run_agent.py).

**Note**:

- Guaranteed execution of each agent is currently only provided with the _start_training.launch_ as simulation launcher
- `--load` flag has to be set _None_, otherwise the script will only consider the agent provided with the flag.

### Important Directories

| Path                                       | Description                                                                                                                             |
| ------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------- |
| `../arena_local_planner_drl/agents`        | models and associated hyperparameters.json will be saved to and loaded from here ([uniquely named directory](#load-a-dnn-for-training)) |
| `../arena_local_planner_drl/configs`       | yaml files containing robots action spaces and the training curriculum                                                                  |
| `../arena_local_planner_drl/training_logs` | tensorboard logs and evaluation logs                                                                                                    |
| `../arena_local_planner_drl/scripts`       | python file containing the predefined DNN architectures and the training script                                                         |
