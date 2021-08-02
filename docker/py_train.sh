 
#!/bin/bash
#get params
#MLP_ARENA2D
#column

train_mode=$1
agent_mode=$2
body_mode=$3
pi_mode=$4
vf_mode=$5
act_fn_mode=$6
config_mod=${7}
n_mode=${8}
tb_mode=${9}
eval_log_mode=${10}
no_gpu_mode=${11}
num_envs=${12}

if [ "$agent_mode" = "custom-mlp" ]
  then
    argument="--custom-mlp"
  if ! [ "$body_mode" = "" ]
  then argument="$argument --body \"$body_mode\""
  fi
  
  if ! [ "$pi_mode" = "" ]
  then argument="$argument --pi \"$pi_mode\""
  fi

  if ! [ "$vf_mode" = "" ]
  then argument="$argument --vf \"$vf_mode\""
  fi
  
  if ! [ "$act_fn_mode" = "" ]
  then argument="$argument --act_fn \"$act_fn_mode\""
  fi  
#roscd arena_local_planner_drl
echo ${argument}|xargs python /root/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training/train_agent.py
fi
#recover the old IFS fully
#deactivate
#IFS=$OLDIFS
