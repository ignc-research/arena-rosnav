#! /bin/bash
# roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode:=random
echo $1
roslaunch=$1
python_train=$2

screen -dmS roslaunch bash -c "source ./ros.sh $roslaunch"
screen -S roslaunch -X logfile screenlog_roslaunch.log
screen -S roslaunch -X log
sleep 10

# python scripts/training/train_agent.py --agent MLP_ARENA2D

#bash ./py_train.sh $trainmode $agent
screen -dmS python_training bash -c "source ./py_train.sh $python_train"
screen -S python_training -X logfile screenlog_python_train.log
screen -S python_training -X log
sleep 4
# bash
