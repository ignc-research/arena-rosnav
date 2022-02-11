from arena_navigation.arena_local_planner.learning_based.arena_local_planner_drl.scripts.training.evaluate_all_in_one_agent import \
    evaluate_agents

base_Agent1 = 'mixed_teb_drl4_rule06_policy2'  # 0.87
base_Agent2 = 'mixed_teb_drl4_rule07_policy3'
base_Agent3 = 'teb_drl4_rule07_policy2'  # 0.86
base_Agent4 = "2xteb_drl4_rule07_policy2"  # 88
base_agent_13 = "teb_drl4_rule07_nn13_16+d_mixed_5M_2"  # 0.93

base_Agent11 = "teb_drl4_rule06_nn7_fx3_10M"  # 86
base_Agent12 = "tebx2_drl4_rule06_nn7_fx3_10obst_20M"  # 81
base_agent_14 = "teb_drl4_rule07_nn21_fx3_mixed_5M_2"  # 0.84
base_agent_15 = "teb_drl4_rule06_nn22_fx3_mixed_5M"  # 0.88

AGENTS = ['arena_ros_only']
eval_episodes = 300
seed = 21
# map_config = ["indoor_obs05.json", "indoor_obs10.json", "indoor_obs15.json", "outdoor_obs05.json",
#               "outdoor_obs10.json", "outdoor_obs15.json"]
map_config = ["outdoor_obs10.json", "outdoor_obs15.json"]

evaluation_name = ["outdoor_obs10_arena", "outdoor_obs15_arena"]

if __name__ == "__main__":
    assert len(map_config) == len(evaluation_name), "Error: map_config and evaluation_name have to have equal length"
    for i in range(len(map_config)):
        evaluate_agents(AGENTS=AGENTS, eval_episodes=eval_episodes, seed=seed, map_config=map_config[i],
                        evaluation_name=evaluation_name[i])
