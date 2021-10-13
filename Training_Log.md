## 1. 2021.09.04
### 1.1. Validate transformed laserscan is correct.
### 1.2. Try training on empty map with a feature-extrator onlys handles obstacles positions.
- [ ] STATE
#### 1.2.1. First Run
```
1. roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map_empty
2. python train_agent_wp.py --pretrained_policy=pretrain_policy_40.pkl --rms=vecnorm_obs_rms_40.pkl EVAL.EVAL_FREQ 2 TRAINING.MAX_STEPS_PER_EPISODE 40
3. roslaunch arena_bringup visualization_training.launch ns:=sim_lei_2 rviz_file:=vtwg
```
#####
Problems:
1. obstacles stick to the wall
   - [x] change the code in flatland -> new push done!
2. not always moving to the goal, oscilocations happenend
   - [x] check pretrain data theta distribution -> use data augumentation

#### 1.2.2. Second Run
**Dynamic ENV**

1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map_empty ns_prefix:=sim_lei_map_empty`

1.2 `python train_agent_wp.py --pretrained_policy=map_empty_pretrain_policy_20.pkl --rms=map_empty_vecnorm_obs_rms_20.pkl --ns_prefix=sim_lei_map_empty  EVAL.EVAL_FREQ 2 TRAINING.MAX_STEPS_PER_EPISODE 60  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[16,3,0]' NET_ARCH.FEATURE_EXTRACTOR.NAME "Pure_Dyn_Obs"`

Found pretraining still used old false data -> recollect pretraining data

***HINT***: make sure the feature extractor is set correctly in the pretraining script(on the same way), otherwise changing the feature extractor won't work properly.

**Static ENV**

1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1`

1.2 `python train_agent_wp.py --pretrained_policy=map1_pretrain_policy_20.pkl --rms=map1_vecnorm_obs_rms_20.pkl --ns_prefix=sim_lei_map1  EVAL.EVAL_FREQ 2 TRAINING.MAX_STEPS_PER_EPISODE 60  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[0,0,0]'`

#### 1.2.3. Collect pretraining data for Dynamic Environment
1.1 `roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode:=true map_file:=map1`
1.2 `python pretrainig_wp_single_env -p1 -p2 `

### 1.3. Try training on map1 with a feature-extrator only handles laser scan.
 - [x] STATE

## 2021.09.06
### RUN dynamic and static run with pretrained model
#### Run
**Static ENV**

1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=3 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1`

1.2 `python train_agent_wp.py --pretrained_policy=map1_pretrain_policy_20.pkl --rms=map1_vecnorm_obs_rms_20.pkl --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 60  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[0,0,0]'`

## 2021.09.07
1. Training clip fraction extremely high(about 0.66),although the the loss keeps  decreasing.--> it can move to the goal, but explorration is not enough,need to increase the epsilon, using pretrain 10 data, to make the agent less likely to reach the goal at the beginning
### Daliy PLAN
- [x] Evalutation mode not correctly running, need to find issue in the environment.
- [x] Training again with new settings.(if not working, 2 change feature extractor of laser, use the one in the paper(Point net?Deep Local Trajectory Replanning and Control for Robot Navigation) 1 discreat action)
- [x] Collect pretraining data for a mixed environment. 
### RUN
***IMPORTANT CHANGE***
Rename previous trained policy with the postfix 'con_actions'

## Pretraining  ##
1.1 `roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode:=true`
1.2 `python pretrainig_wp_single_env -p2 --epochs=20 --expert_data_np_name=expert_data_map1_00.npz --name_prefix=dis_actions NET_ARCH.FEATURE_EXTRACTOR.NAME Pure_Static_Obs WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True `
   pretraining script does't support discrete action
   - [x] solve it!

***MODIFICATION ***
timeout will not terminate the environment anymore

**Static ENV**

1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=3 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1`

1.2 `python train_agent_wp.py --pretrained_policy=map1_pretrain_policy_10.pkl --rms=map1_vecnorm_obs_rms_10.pkl --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 100  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[0,0,0]'`
Results:Pure_Static_Obs_2021_09_07_12_11_46
1. RL seems not working well,(but only training for 7 hours)
2. In deploy mode, it can be observed that the NET is kind of likely put waypoint away from the the obstacle.

## 2021.09.08
### Dynamic ENV(con actions)
#### Pretraining
1.1 use expert_data_map1_00.npz for pretraining, target is to move to the goal
1.2 `roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode:=true`
1.3 `python pretrainig_wp_single_env.py -p2 --epochs=20 --expert_data_np_name=expert_data_map1_00.npz --name_prefix=con_actions NET_ARCH.FEATURE_EXTRACTOR.NAME Pure_Dyn_Obs WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE False`
#### Training
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=6 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map_empty ns_prefix:=sim_lei_map_empty`
1.2 `python train_agent_wp.py --pretrained_policy=con_actions_pretrain_policy_10.pkl --rms=con_actions_pretrain_vecnormvecnorm_obs_rms_10.pkl --ns_prefix=sim_lei_map_empty TRAINING.MAX_STEPS_PER_EPISODE 120  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[10,15,20]' NET_ARCH.FEATURE_EXTRACTOR.NAME "Pure_Dyn_Obs"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]' `
## 2021.09.09
### RESULT
1. it's hard to

## 2021.09.10
### Dynamic ENV(con actions) 
1. investigate why agent easily failed when get close to the goal.
   A* consider the orientation of the goal  -> ***IMPORTANT CHANGE: change expert_data_map1_00.npz to expert_data_map1_00_raw.npz and save the corrected version to expert_data_map1_00.npz  ***
   it works!
#### Pretraining
same as in 09.08
#### Training
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map_empty ns_prefix:=sim_lei_map_empty`
1.2 `python train_agent_wp.py --pretrained_policy=con_actions_pretrain_policy_10.pkl --rms=con_actions_pretrain_vecnormvecnorm_obs_rms_10.pkl --ns_prefix=sim_lei_map_empty TRAINING.MAX_STEPS_PER_EPISODE 120  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[10,15,20]' NET_ARCH.FEATURE_EXTRACTOR.NAME "Pure_Dyn_Obs"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]'` 
- [x] sometimes after the reset, the waypoint is not set correctly,far away from robot's position, need to check! -> already published the pos, no idea.
- [ ] how to define the waypoint is good or not. (time cost? progress like in the paper "where to go next?", but how to shape the reward if the local_planner have the strong ability to recover the cost).

## 2021.09.11
### Mixed ENV
- [x] Timo's talk is totally correct, shall we include global path into the observation space????, done in 2021.09.28
#### Pretraining
1.1 `roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode:=true`
1.2 `python pretrainig_wp_single_env.py -p2 --epochs=20 --expert_data_np_name=expert_data_map1_00.npz --name_prefix=con_actions_mix_Env NET_ARCH.FEATURE_EXTRACTOR.NAME CNN_Laser_MLP_Goal_Dyn_Obs WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE False NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 70 WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE False`
#### Test Training
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1`
1.2 `python train_agent_wp.py --pretrained_policy=con_actions_mix_Env_pretrain_policy_10.pkl --rms=con_actions_mix_Env_pretrain_vecnorm_obs_rms_10.pkl --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 100 EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[16,3,0]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_Laser_MLP_Goal_Dyn_Obs" REWARD.NO_TIMEOUT_REWARD False`
#### Training
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=6 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1`
1.2 `python train_agent_wp.py --pretrained_policy=con_actions_mix_Env_pretrain_policy_10.pkl --rms=con_actions_mix_Env_pretrain_vecnorm_obs_rms_10.pkl --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 100 EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[13,16,0]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_Laser_MLP_Goal_Dyn_Obs" REWARD.NO_TIMEOUT_REWARD False`

#### IMPORTANT:
- [x] Introduce global path into observation (Adaption include pretraining script)!
- [x] V shape for laser
- [x] https://arxiv.org/pdf/2102.13073.pdf objects, stable-baselines3 dont support recurrent policy. 
- [ ] Include Johanns code to dynamically change the map!
#### Training 

## 2021.09.12
### Training Test
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1 include_global_planner:=true`
1.2 `python train_agent_wp.py --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 100 EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[16,3,0]' NET_ARCH.FEATURE_EXTRACTOR.NAME CNN_Laser_Obstalcle_GlobalPlan NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 196 ENV.NAME WPEnvMapFrame2 `

## 2021.09.17
Eval pure static and dynamic env 
pretraining remote server 100? local 0.8????????
use zeromq!!!!
### Dynamic
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=6 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map_empty ns_prefix:=sim_lei_map_empty`
1.2 `python train_agent_wp.py --pretrained_policy=con_actions_pretrain_policy_10.pkl --rms=con_actions_pretrain_vecnormvecnorm_obs_rms_10.pkl --ns_prefix=sim_lei_map_empty TRAINING.MAX_STEPS_PER_EPISODE 800  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[2,8,13]' NET_ARCH.FEATURE_EXTRACTOR.NAME "Pure_Dyn_Obs"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]'` 
### Static
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=6 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1`

1.2 `python train_agent_wp.py --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 800  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[0,0,0]'`

## 2021.09.26
### Important Notes
1.Don't forget to change eval json file path accordingly.
2.change eval behaviour, follow subgoal or goal,  self.ref_pos_topic_name = f"{self.ns_prefix}goal"
3.added server client mode, enable in the file **arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/scripts/deployment/local_planner_deploy.py**
   #PARAM
   agent = DrlAgent(drl_model_path,'drl_rule_04',ns,True,2)

### Check Results
1. tensorboard too less data, EVAL_FREQ=4e4???? check it!!!!!!!!
2. timeout should terminate or not? 
#### Check Dynamic
1. change the field `SCENERIOS_JSON_PATH` in the Hyperparameters.json file to empty map
2. change server path to local path
3. `roslaunch arena_bringup start_arena_flatland_waypoint.launch map_file:=map_empty`
4. `python train_agent_wp.py --conf_file=/home/joe/ssd/projects/arena-rosnav-ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/output/Pure_Dyn_Obs_2021_09_16_19_31_16/Hyperparams.yaml --deploy`
Results:
1.action space too small, mention it in thesis
   -> redefine action_space?????,pretraining is also important
2. proper pretraining is necessary
#### check Static
1. change the field `SCENERIOS_JSON_PATH` in the Hyperparameters.json file to map1
2. change server path to local path
3. `roslaunch arena_bringup start_arena_flatland_waypoint.launch`
4. `python train_agent_wp.py --conf_file=/home/joe/ssd/projects/arena-rosnav-ws/src/arena-rosnav/arena_navigation/arena_local_planner/learning_based/arena_local_planner_drl/output/Pure_Static_Obs_2021_09_16_19_34_06/Hyperparams.yaml --deploy`
Results:
1. global path too close to the obstacle
2. pure static will not be researched anymore,sure?



### TODO List
- [x] enlarge the distance between global trajectory and obstacle

   change here:\
   node_.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, 0.01);

- [ ] check Eval_FREQ, change it to a small value.

### 2021.09.28
#### Test new env
1.  `roslaunch arena_bringup start_training_waypoint.launch num_envs:=6 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1 include_global_planner:=True`
2.  `python train_agent_wp.py --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 800  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[2,8,13]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserV_Obstalcle_GlobalPlan"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" `
### 2021.09.29
####
CHANGES
1. Mapping.cpp not subscribed to sensor anymore. fuck this!


### 2021.10.1
### TODO List
- [x] intermediate planning will not subscribe to goal and accordingly generate the global path, currently it can 
#### Collect pretraining data for mixed Environment
1.1 `roslaunch arena_bringup start_arena_flatland_waypoint.launch train_mode:=true map_file:=outdoor global_planner_active_mode:=false rviz_file:=vtwg_pretrain use_plan_manager:=false`
1.2 ``
1.3 `python pretrainig_wp_single_env.py -p1 -p2 TRAINING.MAX_STEPS_PER_EPISODE 800  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[20,8,13]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 192 `
### 2021.10.2
#### Unfortunately pretraining not working, the loss doesn't change.
- [x] check in the observation, actually sample only 10 points from the global path seems not enough.
- [x] dynamic obstacle's states 
- [x] visualize the net again. -> vis and debug done
- [x] even no pretraining at the beginning,the action is not correct!.print it and debug it, -> it's correct now, i did nothing to it. 
#### try without pretraining.
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=6 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=outdoor ns_prefix:=sim_lei_map_outdoor` 
1.2 `python train_agent_wp.py --ns_prefix=sim_lei_map_outdoor TRAINING.MAX_STEPS_PER_EPISODE 800  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[3,20,13]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 192`
### 2021.10.06
### Pretraining results
1. After reduced the layers of nn, loss reduced but not much.

### Training with pretrained model (no attention)
#### Test and visualization
1.`roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=outdoor ns_prefix:=sim_lei_map_outdoor` 
1. `python train_agent_wp.py --ns_prefix=sim_lei_map_outdoor --pretrained_policy=WPEnvMapFrame3/pretrain_policy_10.pkl TRAINING.MAX_STEPS_PER_EPISODE 800  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[17,20,23]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.5, 0.75]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 96 `
### 2021.10.10
Eventually find the error in the pretraining script. it works now.
1. training with *WPEnvMapFrame3_fake_action_filtered.npz*(Data will be ruled out if robot too close(within 1m) to the goal or the action is abnormal) are much more better. 
2. Attension based method the weight of global plan will come to 1 very soon, which is not wanted, maybe illustrated on the thesis.
3. Have no idea why if use cpu for training the loss decreased not much 
#### Pretraining
##### Attension based
1. `roslaunch arena_bringup start_arena_flatland_waypoint.launch train_mode:=true map_file:=outdoor global_planner_active_mode:=false rviz_file:=vtwg_pretrain use_plan_manager:=false`
2. `python pretrainig_wp_single_env.py -p2 -e=WPEnvMapFrame3_fake_action_filtered.npz  --epochs=100 --name_prefix=attension_based  TRAINING.MAX_STEPS_PER_EPISODE 800  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[17,20,23]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan_Attention"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.75, 0.85]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 96`

#### Training
##### Attension based
1.`roslaunch arena_bringup start_training_waypoint.launch num_envs:=6 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=outdoor ns_prefix:=sim_lei_outdoor`
2.`python train_agent_wp.py --pretrained_policy=WPEnvMapFrame3/attension_based_pretrain_policy_100.pkl --ns_prefix=sim_lei_outdoor TRAINING.MAX_STEPS_PER_EPISODE 400  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[17,20,23]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan_Attention"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.75, 0.85]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 96 WAYPOINT_GENERATOR.GOAL_RADIUS 1.5`

### 2021.10.13
#### local training
1. `roslaunch arena_bringup start_training_waypoint.launch num_envs:=8 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=outdoor ns_prefix:=sim_lei_outdoor`
2. `python train_agent_wp.py --pretrained_policy=WPEnvMapFrame3/attension_based_pretrain_policy_40.pkl --ns_prefix=sim_lei_outdoor TRAINING.MAX_STEPS_PER_EPISODE 400  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[17,20,23]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan_Attention"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.75, 0.85]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 96 WAYPOINT_GENERATOR.GOAL_RADIUS 1.5` 
##### Results
1. directly terminating the training when the event "TIMEOUT" or "COLLISION" is maybe not good idea, so we need to change this behaviour and adjust the reward
2. eval frequenz should reduce a bit

# TODO
 - [x] check training results on the server which use *20.pkl at 12:30 pm, if it has similar results, we need adjust api
 to enable customize the behaviour of "TIMEOUT" or "COLLISION" (terminate or continue with a relaive higher negative reward,especially collision.

##### Changes
1. made a new map *outdoor2* which is basically rotated from the orignal outdoor map -> test indicate the pretraining learns shortcut which make the robot not simply follow the global path, but considerd the points much far away, lets see how RL merge this desired behaviour and local info together-> pay more attention on local environment
2.  added new setting `TRAINING.Teminate_ON_TIMEOUT = True`, we want to test what would happen if we set it to False

### 2021.10.14
one with attention one without
##### local training
1. `roslaunch arena_bringup start_training_waypoint.launch num_envs:=8 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=outdoor2 ns_prefix:=sim_lei_outdoor`
2. `python train_agent_wp.py --pretrained_policy=WPEnvMapFrame3/attension_based_pretrain_policy_20.pkl --ns_prefix=sim_lei_outdoor TRAINING.MAX_STEPS_PER_EPISODE 400  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[17,20,23]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan_Attention"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.75, 0.85]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 96 WAYPOINT_GENERATOR.GOAL_RADIUS 1.5 TRAINING.TERMINATE_ON_TIMEOUT False ` 
##### server training
1. `roslaunch arena_bringup start_training_waypoint.launch num_envs:=11 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=outdoor2 ns_prefix:=sim_lei_outdoor`
2. `python train_agent_wp.py --pretrained_policy=WPEnvMapFrame3/normal_pretrain_policy_20.pkl --ns_prefix=sim_lei_outdoor TRAINING.MAX_STEPS_PER_EPISODE 400  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[17,20,23]' NET_ARCH.FEATURE_EXTRACTOR.NAME "CNN_LaserVAE_Obstalcle_GlobalPlan"  EVAL.CURRICULUM.THRESHOLD_RANGE '[0.75, 0.85]' INPUT.NORM False ENV.NAME "WPEnvMapFrame3" WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 96 WAYPOINT_GENERATOR.GOAL_RADIUS 1.5 TRAINING.TERMINATE_ON_TIMEOUT False ` 