clone this repro
```
sudo git clone -b docker_new --depth 1 https://github.com/Ann-Rachel/arena-rosnav /the/folder/you/want
```
go to the docker folder
```
cd arena-rosnav/docker
```
build the docker
```
sudo docker build --no-cache -t arena_rosnav . -f Dockerfile
```
default environment variables are:
roslaunch arena_bringup start_arena_flatland.launch train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4
.../train_agent.py --custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn relu

set environment variables
```
sudo docker build \
--build-arg ROS_START="see [https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/DRL-Training.md](DRL-Training.md)" \
--build-args PYTHON_START="see [https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/DRL-Training.md](DRL-Training.md)" \
-t simple . -f Dockerfile
```
for example
```
sudo docker build \
--build-arg ROS_START="train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4" \
--build-arg PYTHON_START="--custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn tanh" \
-t arena_rosnav . -f Dockerfile
```
run docker
```
sudo docker run \
 -v "$HOME/your/folder:$HOME/folder/in/docker" \
 -it --rm --net=host arena_rosnav
```
