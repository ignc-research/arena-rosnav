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
- [ ] Training again with new settings.(if not working, 2 change feature extractor of laser, use the one in the paper(Point net?Deep Local Trajectory Replanning and Control for Robot Navigation) 1 discreat action)
- [ ] Collect pretraining data for a mixed environment. 
### RUN
***IMPORTANT CHANGE***
Rename previous trained policy with the postfix 'con_actions'

## Pretraining  ##
1.1 `roslaunch arena_bringup start_arena_flatland_waypoint.launch local_planner:=None train_mode:=true`
1.2 `python pretrainig_wp_single_env -p2 --epochs=20 --expert_data_np_name=expert_data_map1_00.npz --name_prefix=dis_actions NET_ARCH.FEATURE_EXTRACTOR.NAME Pure_Static_Obs WAYPOINT_GENERATOR.IS_ACTION_SPACE_DISCRETE True `
   pretraining script does't support discrete action
   - [ ] solve it!

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
- [ ] Timo's talk is totally correct, shall we include global path into the observation space????
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
- [ ] Introduce global path into observation (Adaption include pretraining script)!
- [ ] V shape for laser
- [x] https://arxiv.org/pdf/2102.13073.pdf objects, stable-baselines3 dont support recurrent policy. 
- [ ] Include Johanns code to dynamically change the map!
#### Training 

## 2021.09.12
### Training Test
1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1 include_global_planner:=true`
1.2 `python train_agent_wp.py --ns_prefix=sim_lei_map1 TRAINING.MAX_STEPS_PER_EPISODE 100 EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[16,3,0]' NET_ARCH.FEATURE_EXTRACTOR.NAME CNN_Laser_Obstalcle_GlobalPlan NET_ARCH.FEATURE_EXTRACTOR.FEATURES_DIM 196 ENV.NAME WPEnvMapFrame2 `