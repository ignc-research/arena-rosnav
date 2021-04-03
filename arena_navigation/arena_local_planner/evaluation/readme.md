# Evaluation

The evaluation script will compare different planners provided as rosbags. The bags need to be recorded while running a scenario (see here), since some topics are needed by the script. The script file is located in script/scenario_eval.py

Once the config id ready, one can easily start the evaluation by ```python scenario_eval.py``` 


## yml config

The config file needs to be placed in the script folder. Suppose there is a config file called "test.yml"
In this file the user can define how many figures will be created, which plots each figure will contain, which color and style each planners will have ...etc.

## example cfg: test.yml

````
# Each yml file should start with the default cfg block, these parameters will apply to all
# figures below in order to prevent redundant parameter definitions:


**default_cfg:** 
    **plot_trj:**        1
    **plot_zones:**      1
    **plot_obst:**       1
    **plot_collisions:** 1
    **plot_grid:**       0
    **plot_sm:**         1
    **plot_gp:**         0
    **plot_subgoals:**   0
    **folder:**          "run_3/" 


# fig 1
custom_cfg_1: 
    plot_zones: 1
    plot_sm:    1
    plot_obst:  1

# each drl planner with all wpg

map1_obs20_vel02_Cadrl:
            
    planner:

        cadrl_sh:
            model:     "cadrl"
            linestyle: "tab:red,-"
            wpg:       "spatialhorizon"
        cadrl_ts:
            model:     "cadrl"
            linestyle: "tab:blue,--"
            wpg:       "timespace"
        cadrl_ss:
            model:     "cadrl"
            linestyle: "tab:grey,-."
            wpg:       "subsampling"

map1_obs20_vel02_drl1:
            
    planner:

        drl1_sh:
            model:     "drl1"
            linestyle: "tab:red,-"
            wpg:       "spatialhorizon"
        drl1_ts:
            model:     "drl1"
            linestyle: "tab:blue,--"
            wpg:       "timespace"
        drl1_ss:
            model:     "drl1"
            linestyle: "tab:grey,-."
            wpg:       "subsampling"

