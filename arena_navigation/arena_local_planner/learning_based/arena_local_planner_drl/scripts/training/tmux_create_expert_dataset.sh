#!/bin/sh
iterations=$1
num_envs=$2
robot_model=$3
pretrain_config=$4
s_d=$5

tmux new-session \; \
send-keys 'workon rosnav' C-m \; \
send-keys "roslaunch arena_bringup start_training_all_in_one_planner.launch num_envs:=$num_envs robot_model:=$robot_model" C-m\; \
split-window -h \; \
send-keys 'workon rosnav' C-m \; \
send-keys 'source catkin_ws/devel/setup.zsh' C-m \; \
send-keys 'roscd arena_local_planner_drl/' C-m \; \
send-keys "python3 scripts/training/pretrain_agent.py $iterations \"data_${robot_model}_dyn_${s_d}_$iterations\" $num_envs $pretrain_config ${s_d}" C-m \; \
split-window -h \; \
send-keys 'roslaunch arena_bringup visualization_training.launch use_rviz:=true ns:=eval_sim rviz_file:=allinone_train' \; \
select-layout even-horizontal \; \
split-window -v \; \
select-pane -t 2 \; \
