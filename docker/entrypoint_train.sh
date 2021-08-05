#! /bin/bash
# roslaunch arena_bringup start_arena_flatland.launch  train_mode:=true 	use_viz:=true  task_mode:=random

train_name_in_the_csv=$1
taskmode = $2
map_file=$3
numenvs=$4

INPUT=train_params.csv
OLDIFS=$IFS
IFS=,
[ ! -f $INPUT ] && { echo "$INPUT file not found"; }
init=false

screen -dmS roslaunch bash -c "source ./ros.sh $taskmode $map_file $numenvs"
screen -S roslaunch -X logfile screenlog_roslaunch.log
screen -S roslaunch -X log
sleep 10

# python scripts/training/train_agent.py --agent MLP_ARENA2D

while read trainmode agent body pi vf act_fn config n tb eval_log no_gpu num_envs 
do
       
    if [ "$init" = false ] ;
        then
            init=true
        continue;
    fi
    
    if [ "$trainmode" = "$train_name_in_the_csv" ]; 
    then

	 echo "$trainmode"
         echo "$agent" 
         echo "$body"
	 echo "$pi"
	 echo "$vf"
	 echo "$act_fn"
	 echo "$config"
         #bash ./py_train.sh $trainmode $agent
         screen -dmS python_training bash -c "source ./py_train.sh $trainmode $agent $body $pi $vf $act_fn $config $n $tb $eval_log $no_gpu $num_envs"
         screen -S python_training -X logfile screenlog_python_train.log
         screen -S python_training -X log
         sleep 4
    fi

done < $INPUT
# bash
