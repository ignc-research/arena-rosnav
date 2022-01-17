# Training on the GPU Server

This section outlines the steps and commands that need to be executed to train on a GPU cluster.
For our case, the training is carried out on an NVIDIA RTX 3090 GPU. 

### NVIDIA RTX 3090 - Install the latest pytorch version 
In order to train with an NVIDIA GPU RTX3090 you need the latest version of pytorch. Inside your rosnav venv, do:
```
workon rosnav
pip3 install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
```

#### 0. Running long training jobs using Screen (OPTIONAL)

The training jobs could run several days, so you should use tmux. This way you can create a virtual session which will keep running on the server, even if the SSH connection is interrupted.

1. Install tmux
```
sudo apt-get update
sudo apt-get install tmux
```
2. You will need two terminals: one for the arena simulation and one for the training script.

-Create screens using tmux <sessionname>:
```
tmux simulation
tmux training
```
If you just type tmux, the session name will be a enumerated (0,1,2,...)

3. Then run the training commands listed in section 1 below.

-To detach screens during training (for example if you want to power off your own machine), press 
```
CTRL-B and afterwards CTRL-D
```

-To reatach screens to check progress, run these commands in terminals:

```
tmux a -t training
```
Explanation:
a: attach to a session
-t: flag to indicate name of the session
training: name of the session

For more shortcuts, see https://tmuxcheatsheet.com/

#### 1. Launching a training session

#### 1.1 First terminal
The first terminal (the simulation session, if using Screen) is needed to run arena.
Run these four commands:
```                     
source $HOME/arena_ws/devel/setup.zsh    
workon rosnav
roslaunch arena_bringup start_training.launch train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=24
```

#### 1.2 Second terminal 
A second terminal (the training session if using Screen) is needed to run the training script.
Run these four commands:
```                 
source $HOME/arena_ws/devel/setup.zsh   
workon rosnav
roscd arena_local_planner_drl
```

Now, run one of the two commands below to start a training session:
```
python scripts/training/train_agent.py --load pretrained_ppo_mpc --n_envs 24 --eval_log 
python scripts/training/train_agent.py --load pretrained_ppo_baseline --n_envs 24 --eval_log
```

#### 2. Ending a training session

When the training script is done, it will print the following information and then exit:
```
Time passed: {time in seconds}s
Training script will be terminated
```

Please make a note of this time taken.

The simulation will not terminate automatically, so it needs to be stopped with CTRL-C.
