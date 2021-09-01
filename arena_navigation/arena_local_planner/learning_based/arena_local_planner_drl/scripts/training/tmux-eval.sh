#!/bin/sh

tmux new-session \; \
send-keys 'workon rosnav' C-m \; \
send-keys "roslaunch arena_bringup start_training_all_in_one_planner.launch num_envs:=1" C-m\; \
split-window -h \; \
send-keys 'workon rosnav' C-m \; \
send-keys 'source catkin_ws/devel/setup.zsh' C-m \; \
send-keys 'roscd arena_local_planner_drl/' C-m \; \
send-keys "python3 scripts/training/evaluate_all_in_one_agent.py --load all_in_one_0.6" C-m \; \
split-window -h \; \
send-keys 'roslaunch arena_bringup visualization_training.launch use_rviz:=true ns:=eval_sim rviz_file:=allinone_evalsim' \; \
select-layout even-horizontal \; \
split-window -v \; \
select-pane -t 2 \; \
