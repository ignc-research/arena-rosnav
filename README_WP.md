#Pretraining
#Training
#Evaluation and deployment
1. In the 1st terminal run the command `roslaunch arena_bringup start_arena_flatland_waypoint.launch`
2. go the directory *arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training* and run `python train_agent_wp.py --conf_file=/home/joe/ssd/projects/arena-rosnav-ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/output/CNN_Laser_MLP_Goal_Dyn_Obs_2021_08_26_16_18_09/Hyperparams.yaml","--deploy"]`