sudo git clone -b docker_new --depth 1 https://github.com/Ann-Rachel/arena-rosnav /root/catkin_ws/src/arena-rosnav
cd docker
sudo docker build -t arena_rosnav . -f Dockerfile
default:
roslaunch arena_bringup start_arena_flatland.launch train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4
.../train_agent.py --custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn relu
or
sudo docker build --build-arg ROS_START="see DRL_training" PYTHON_START="see DRL_training" -t simple . -f Dockerfile
example
sudo docker build --build-arg ROS_START=train_mode:=true use_viz:=false task_mode:=random map_file:=map_small num_envs:=4 --body 256-128 --pi 256 --vf 16 --act_fn relu PYTHON_START=--custom-mlp --body 256-128 --pi 256 --vf 16 --act_fn relu
or -t arena_rosnav . -f Dockerfile
sudo docker run -it --rm --net=host arena_rosnav
