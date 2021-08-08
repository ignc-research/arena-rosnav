# Start training with docker
0. install docker (optional)
https://docs.docker.com/get-docker/

Ubuntu 20.04: latest version of Docker
```bash
sudo apt update
sudo apt install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
docker -v
Docker version 20.10.7, build f0df350
```
1. clone this repro
```
sudo git clone -b docker_new --depth 1 https://github.com/Ann-Rachel/arena-rosnav /the/folder/you/want
```
2. go to the docker folder ../arena-rosnav/docker
```
cd arena-rosnav/docker
```
3. build the docker images 
```
sudo docker build --no-cache -t arena_rosnav . -f Dockerfile
```
default environment variables are:
roslaunch arena_bringup start_arena_flatland.launch train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4
.../train_agent.py --custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn relu

set environment variables

DRL-Training.md: https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/docs/DRL-Training.md
```
sudo docker build \
--build-arg ROS_START="see DRL-Training.md" \
--build-args PYTHON_START="see DRL-Training.md" \
-t simple . -f Dockerfile
```
for example
```
sudo docker build \
--build-arg ROS_START="train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4" \
--build-arg PYTHON_START="--custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn tanh" \
-t arena_rosnav . -f Dockerfile
```
4. run docker
```
sudo docker run \
 -v "$HOME/your/folder:$HOME/folder/in/docker" \
 -it --rm --net=host arena_rosnav
```
for example
```
sudo docker run -v "/yourhome/arena-rosnav/docker:/root/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/agents"  -it --rm arena_rosnav
```
# woking with docker
1.clone this repro
```
sudo git clone -b docker_new --depth 1 https://github.com/Ann-Rachel/arena-rosnav /the/folder/you/want
```
2.go to the right folder ../arena-rosnav/docker
```
cd arena-rosnav/docker
```
3.build the docker images 
```
sudo docker build --no-cache -t arena_rosnav . -f Dockerfile_w
```
4.start without rviz
```
sudo docker run -it --rm --net=host arena_rosnav
```
5. with rviz: close container and go back to ../arena-rosnav in host machine
```
cd ..
```
6. start docker-compose

```
docker-compose up -d \
docker-compose start \
```
7. enter the contaier
```
docker exec -it arena-rosnav bash
```
8. quickstart
```
roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode:=random
```
9. open http://localhost:8080/vnc.html you will see the rviz window in browser
