#!/bin/sh
num_envs=$1
agent_name=$2
aio_config=$3
hyperparam=$4
pre_train_file=$5
robot_model=$6

tmux new-session \; \
send-keys "workon rosnav" C-m \; \
send-keys "roslaunch arena_bringup start_training_all_in_one_planner.launch num_envs:=$num_envs robot_model:=$robot_model" C-m\; \
split-window -h \; \
send-keys 'workon rosnav' C-m \; \
send-keys 'source catkin_ws/devel/setup.zsh' C-m \; \
send-keys 'roscd arena_local_planner_drl/' C-m \; \
send-keys "python3 scripts/training/train_all_in_one_agent.py --agent AGENT_14  --eval_log --tb --n_envs $num_envs --agent_name $agent_name --all_in_one_config $aio_config --pretrain_file $pre_train_file --config $hyperparam" C-m \; \
split-window -h \; \
send-keys 'roslaunch arena_bringup visualization_training.launch use_rviz:=true rviz_file:=allinone_train' \; \
select-layout even-horizontal \; \
split-window -v \; \
select-pane -t 2 \; \
