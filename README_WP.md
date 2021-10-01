# Prerequisites
### compile crings used for vae
   conda activate <your-env>
   cd crings and run `python setup.py install`

# Visualize training
1.An Example `roslaunch arena_bringup visualization_training.launch ns_map:=outdoor rviz_file:=vtwg ns:=sim_lei_map_outdoor_1`

# Pretraining
## collecting export data
1. Run `roslaunch arena_bringup start_arena_flatland_waypoint.launch train_mode:=true map_file:=outdoor  global_planner_active_mode:=false use_plan_manager:=false rviz_file:=vtwg_pretrain` to disable local planner and plan manager.`global_planner_active_mode` is actually not work(anyone fix it?)
2. Go the directory *arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training* and run `python pretrainig_wp_single_env.py -p1 -p2 TRAINING.MAX_STEPS_PER_EPISODE 200  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[20,8,13]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 192 TRAINING.ROBOT_START_POS_GOAL_POS_MIN_DIST 10`, some params can not be set as args, you can check the code and modify it.
3. add params `-p2` to enable pretraining and save trained model.
# Training

1. Run `roslaunch arena_bringup start_training_waypoint.launch`.Some important arguments are:
   * *ns_prefix* set it if you want running multiple training scripts in parallel. This is the key point to isolate the training environments.
   * *num_envs* number of the training environments and one of them will be used for evaluation.
   * *local_planner* could be *teb*
  
   Hints:
   To reduce the RAM consumption Server and client mode is used for deployment of the drl as the local planner.
   If you want to change it
   go to the file **arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/deployment/local_planner_deploy.py** and find the flowing code block
   ```python
   #PARAM
   agent = DrlAgent(drl_model_path,'drl_rule_04',ns,True,2)
   ```
   where `True` is use server-client mode or not, `2` is number of the client every server serve to. 

2. Go the directory *arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training* and run `python train_agent_wp.py`, the default setting located in the following directory **arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/rl_agent/config/defaultsWP2.py**. To overwrite the default setting, you can append the parameters in the command line, an example would be `python train_agent_wp.py NET_ARCH.FEATURE_EXTRACTOR.NAME Pure_Dyn_Obs` which changes the feature extractor. There are some others features such as loading pretrained model, you can check the usage using '-h' argument. 
   EXAMPLES:
      1. 
# Evaluation and deployment
1. In the 1st terminal run the command `roslaunch arena_bringup start_arena_flatland_waypoint.launcroslaunch arena_bringup start_arena_flatland_waypoint.launchh`, remember to set `map_file:=???`
2. Go the directory *arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/training* and run `python train_agent_wp.py --conf_file=/home/joe/ssd/projects/arena-rosnav-ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/output/CNN_Laser_MLP_Goal_Dyn_Obs_2021_08_26_16_18_09/Hyperparams.yaml","--deploy"]`