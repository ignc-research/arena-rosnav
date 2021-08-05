#! /bin/sh
trainmode = $1
map=$2
n_envs = $3

roslaunch arena_bringup start_arena_flatland.launch train_mode:=true use_viz:=false task_mode:=$trainmode map_file:=$map num_envs:=$n_envs
