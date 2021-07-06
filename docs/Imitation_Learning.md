## Imitation Learning

This section outlines the steps and commands that need to be executed to train a pretrained agent and its corresponding baseline agent.

#### 0. Running long training jobs using Screen (OPTIONAL)

The training jobs will run up to 12 hours, so it can be a good idea to run them using Screen. This way you can create a virtual session which will keep running on the server, even if the SSH connection is interrupted.

Alternatively, you can also use ```tmux```.

You will need two terminals: one for arena and one for the training script.

-Create screens:
```
screen -S simulation
screen -S training
```

Then run the training commands listed in section 1 below.

-To detach screens during training (for example if you want to power off your own machine), press ```CTRL-A + CTRL-D```

-To reatach screens to check progress, run these commands in terminals:
```
screen -r simulation
screen -r training
```

#### 1. Launching a training session

#### 1.1 First terminal
The first terminal (the simulation session, if using Screen) is needed to run arena.
Run these four commands:
```
source $HOME/.zshrc                         
source $HOME/catkin_ws/devel/setup.zsh    
workon rosnav
roslaunch arena_bringup start_training.launch train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=24
```

#### 1.2 Second terminal 
A second terminal (the training session if using Screen) is needed to run the training script.
Run these four commands:
```
source $HOME/.zshrc                        
source $HOME/catkin_ws/devel/setup.zsh   
workon rosnav
roscd arena_local_planner_drl
```

Now, run one of the two commands below to start a training session:
```python scripts/training/train_agent.py --load pretrained_ppo_mpc --n_envs 24 --eval_log --tb```
```python scripts/training/train_agent.py --load pretrained_ppo_baseline --n_envs 24 --eval_log --tb```

Setting ```--tb``` could throw a tensorboard error during training if there is a dependency mismatch and terminate the process. Remove ```--tb``` flag if this happens.

#### 2. Ending a training session

When the training script is done, it will print the following information and then exit:
```
Time passed: {time in seconds}s
Training script will be terminated
```

Please make a note of this time taken.

The simulation will not terminate automatically, so it needs to be stopped with CTRL-C.