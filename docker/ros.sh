#! /bin/sh
$trainmode = true
$useviz = true
$taskmode = random

echo "$trainmode"
echo "$useviz"
echo "$taskmode"

roslaunch arena_bringup start_arena_flatland.launch  train_mode:=$trainmode use_viz:=$useviz  task_mode:=random
