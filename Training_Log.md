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
   - [ ] change the code in flatland
2. not always moving to the goal, oscilocations happenend
   - [ ] check pretrain data theta distribution

#### 1.2.2. Second Run
**Dynamic ENV**

1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map_empty ns_prefix:=sim_lei_map_empty`

1.2 `python train_agent_wp.py --pretrained_policy=map_empty_pretrain_policy_20.pkl --rms=map_empty_vecnorm_obs_rms_20.pkl --ns_prefix=sim_lei_map_empty  EVAL.EVAL_FREQ 2 TRAINING.MAX_STEPS_PER_EPISODE 60  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[16,3,0]' NET_ARCH.FEATURE_EXTRACTOR.NAME "Pure_Dyn_Obs"`

***HINT***: make sure the feature extractor is set correctly in the pretraining script(on the same way), otherwise changing the feature extractor won't work properly.

**Static ENV**

1.1 `roslaunch arena_bringup start_training_waypoint.launch num_envs:=2 pretrain_mode:=false local_planner:=drl env_start_idx:=1 map_folder_name:=map1 ns_prefix:=sim_lei_map1`

1.2 `python train_agent_wp.py --pretrained_policy=pretrain_policy_20.pkl --rms=vecnorm_obs_rms_20.pkl --ns_prefix=sim_lei_map1  EVAL.EVAL_FREQ 2 TRAINING.MAX_STEPS_PER_EPISODE 60  EVAL.CURRICULUM.STAGE_DYNAMIC_OBSTACLE '[0,0,0]'`


### 1.3. Try training on map1 with a feature-extrator only handles laser scan.
 - [ ] STATE