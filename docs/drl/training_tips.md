# Useful Tips for Training

On this page, you will find a list of potentially useful tips for the training of a DRL agent in Rosnav. Many training runs have been conducted. This site poses as a collection of helpful findings.

**Disclaimer**
_No guarantee for general/cross-platform correctness!_

**Table of Content**

- [Useful Tips for Training](#useful-tips-for-training)
  - [Training Setup](#training-setup)
      - [Leverage Local Computing Capabilities](#leverage-local-computing-capabilities)
      - [Right Map Size](#right-map-size)
  - [PPO Training Parameters](#ppo-training-parameters)
      - [Batch Size](#batch-size)
      - [Max Steps per Episode](#max-steps-per-episode)
      - [Clipping Range](#clipping-range)
  - [Task Mode](#task-mode)
      - [Training Curriculum](#training-curriculum)
      - [Random](#random)
  - [Agents](#agents)
      - [Reward Functions](#reward-functions)
      - [Network Type and Size](#network-type-and-size)
      - [Observations](#observations)
      - [Actions](#actions)

## Training Setup

#### Leverage Local Computing Capabilities

- Leverage your CPU's as best as possible - train your agents with multiple parallel environments
- Best practice: as many parallel simulations as your machine possesses kernels

#### Right Map Size

- Choose a map size with respect to the robot model's specs
- The map must not be too small, as one would overfit the model with limited laser scan
- E.g. the laser range is 15m but we chose a 5m x 5m map, thus the model might perform unreasonably in evaluation when deployed in maps larger than 10m x 10m (especially in situations where there are no obstacles in a radius larger than the size of the training map)
- At the same time, the larger the map the slower the simulation throughput -> **trade off between performance robustness and training speed**
- Finding the perfect map size for training proves non-trivial

## PPO Training Parameters

For a detailed description of the algorithm, please refer to the official [Proximal Policy Optimization paper](https://arxiv.org/abs/1707.06347).

#### Batch Size

- Consider increasing batch size when the average episode length is relatively long, that way the model can better generalize eventually, as it learns from more (different) experiences
- Also the training script requires the batch size to be devisible by the number of environments, since each simulation is allocated the same number of timesteps to collect states from.

#### Max Steps per Episode

- Keep the maximum number of steps per episode during training as low as possible! But high enough that the robot is able to realistically navigate the hardest possible scenario on the map.
- Thus, one avoids unnecessary long paths and allows for better generalization as our batch is more likely to consist of a multitude of different scenarios.
- The same parameter is not that relevant for the evaluation episodes during training. It is recommended to even chose a slightly higher number of max episodes during evaluation.

#### Clipping Range

- It is not recommended to increase clipping range further than 0.3 as that could lead to a more unstable performance and longer training time.

## Task Mode

- Generally in training, one can say: _the more different the scenarios are to each other, the more robust and performant the agent's tend to navigate eventually_.
- It is recommended to consider the learning curriculum paradigm for training as it can significantly speed up the training process.

#### Training Curriculum

- The training curriculum needs to be adjusted to different map sizes
- The tasks shoudn't be too hard - agent should be able to realistically navigate through 90% of the generated scenarios in the last stage otherwise it might not converge optimally
- Choose a representative number of evaluation episodes (at least 100)

#### Random

- The number of obstacles need to be adjusted to different map sizes
- The tasks shoudn't be too hard - agent should be able to realistically navigate through 90% of the generated scenarios in the last stage otherwise it might not converge optimally
- Choose a representative number of evaluation episodes (at least 100)

## Agents

#### Reward Functions

- Choosing the right reward function proves to be an integral, non-trivial part in reinforcement learning
- Findings showed that including global plan information speeds up training (when training from scratch)
- Many times, agents tend to slightly move in slalom trajectories - by adding a term that penalizes abrupt direction changes one can counteract that type of inefficient behavior.
- A well-tuned example implementation that incorporates the tips above can be found [here](https://github.com/ignc-research/arena-rosnav/blob/228c4ed1c2f906adacc2c7aa251e77f206b6f652/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/utils/reward.py#L192).
- Generally, marginally adjusting some parameters of the reward function might already have an noticeable impact on the agent's performance.

#### Network Type and Size

- So far, the deeper the network the better the performance
- Larger neural networks tend to require longer training time though
- Thus, there is a trade off between computation time and learning ability
- Our research has shown, that MLPs perform less respectable than CNNs with comparable model sizes.
- Even shallower CNNs clearly outperformed deeper MLPs that we constructed - the different networks can be found at: [custom_sb3_policy.py](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/model/custom_sb3_policy.py)

#### Observations

- More laser does not seem to correlate with better performance
- On the other hand, more laser scans more calculations
- Perhaps downsample?
- Moreover, it is recommended to normalize the observations - especially when the agent has continouos action space, as is described [here](https://stable-baselines3.readthedocs.io/en/master/guide/rl_tips.html#tips-and-tricks-when-creating-a-custom-environment).
- When the agent's learning process comprises a reward function that somehow incorporates the agent's latest action, one should extend the observation space by the actions (x linear velocity, y linear velocity, angular velocity). That is already implemented and can be enabled by setting the `actions_in_observationspace` paramater in the hyperparameter.json _true_.

#### Actions

- It is recommended to normalize the actions - especially when the agent has continouos action space, as is described [here](https://stable-baselines3.readthedocs.io/en/master/guide/rl_tips.html#tips-and-tricks-when-creating-a-custom-environment).
- **continuous vs discrete action space**:
  - Training with discrete action space might be faster, the robot's trajectories seem to be less smooth though
  - Robots with continuous action space require a bit longer training time, their trajectory seem to be far more natural and smooth though
