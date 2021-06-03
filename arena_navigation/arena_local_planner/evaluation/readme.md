# Installation
The evaluation script need some packages in order to work and a python version above 3.7. We recommend to create a new venv for evaluation purposes.
```
pip install sklearn
```

# Evaluation

The evaluation script will compare different planners provided as rosbags. The bags need to be recorded while running a scenario (see here), since some topics are needed by the script. The script file is located in script/scenario_eval.py

Once the config id ready, one can easily start the evaluation with the following steps

 * Start Simulation
```
roslaunch arena_bringup start_arena_flatland.launch disable_scenario:="false" map_file:="map1" scenario_file:="eval/obstacle_map1_obs10.json" local_planner:="teb"
```
Here, one should specify the map on which the rosbags were recorded. 
* Afterwards run:

```
python scenario_eval.py test.yml png
``` 
Where test.yml is the config file to specify which plots you want to create. More details below. Pay attention that each map may have different origins and you have to specify the origin within the scenario_eval-py script. (Currently the origin is hardcoded on line 845 but we plan to include that parameter into the config in future.) 

Following files will be created: 
* A new folder will be created inside the [../evaluations/plots](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/plots) folder containing all the plots. 
* Additionally, the .json files with all metrics for the [quantitative evaluation scripts](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/sim_evaluation_v3.py) are generated inside [../evaluations/quantitative](https://github.com/ignc-research/arena-rosnav/tree/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/quantitative) 

**Important**: One should put all the generated .json files into one specified folder in order to run the quantitative evaluation scripts properly. For more information about quantiative plots please refer to [quantitative.md](https://github.com/ignc-research/arena-rosnav/blob/local_planner_subgoalmode/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/readme.md)

## yml config

The config file needs to be placed in the script folder. Suppose there is a config file called "test.yml"
In this file the user can define how many figures will be created, which plots each figure will contain, which color and style each planners will have ...etc.

## example cfg: test.yml

```yml
# Each yml file should start with the default cfg block, these parameters will apply to all
# figures below in order to prevent redundant parameter definitions:

# ------------ default config block ------------
# 1: will plot, 0: will not plot
default_cfg:
    plot_trj:        1        # plot agent trajectory
    plot_zones:      0        # plot collision zones
    plot_obst:       1        # plot obstacles (start, end, direction)
    plot_collisions: 1        # plot a circle where agent collided
    plot_grid:       0        # plot collision grid  (eval)
    plot_sm:         0        # plot static map
    plot_gp:         0        # plot global plan
    plot_subgoals:   0        # plot subgoals
    folder:          "run_3/" # folder containing the planners


# ------------ figure block ------------
# Each figure name needs to be unique
# naming convention: $map_obs$n_vel$v_$name (unique)
# $map             : name of the map 
# $n               : number of obstacles
# $v               : obstacle velocity
# $name            : an arbitary name
# ---------------------------------------

# fig 1
# in figure 1 we want to have the empty map with 20 obstacles and vel of 0.2
# The script will generate a "map1_obs20_vel02_testplot1.png" file inside plots/
# We can include any amount of plots to this figure by adding them to "planner"
# In figure 1 we want to compare three wpg's for the cadrl model
map1_obs20_vel02_testplot1:            
    planner:                                
        cadrl_sh:                          # name of the planner (will be used in plot legend)
            model:     "cadrl"             # folder name containing the bags for desired model
            linestyle: "tab:red,-"         # linestyle and color for all plots related to this planner
            wpg:       "spatialhorizon"    # waypoint generator
        cadrl_ts:
            model:     "cadrl"
            linestyle: "tab:blue,--"
            wpg:       "timespace"
        cadrl_ss:
            model:     "cadrl"
            linestyle: "tab:grey,-."
            wpg:       "subsampling"

# fig 2
# Suppose in fig 2 we want to plot the zones, static map and obstacles
# we can override these parameters by using the custom_cfg block
# figure 2 will have the same comparison as fig 1 with additional information
# ------------ figure block ------------
# naming convention: custom_cfg_$name
# $name:             any name (but needs to be unique)
custom_cfg_2: 
    plot_zones: 1 
    plot_sm:    1
    plot_obst:  1

map1_obs20_vel02_testplot2:
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


# fig 3
# Since we did not define any custom config for figure 3, the default config will be used.
# The parameters set on "custom_cfg_2" will not have any impact here.
# figure 3 compares 3 planners using the wpg spatialhorizon. 
# We also want the cadrl model from a different folder than the folder defined in the default cfg:
# In order to do that we can just simply put an additional parameter called "folder"
# We could also ovverride the default folder param with a "custom_cfg_3", but this would apply to all planners !
map1_obs20_vel02_testplot3:
    planner:
        cadrl (from run 2):
            folder:    "run_2/" # model from folder "run_2/"  
            model:     "esdf"
            linestyle: "tab:red,-"
            # wpg:     "esdf" --> not necessary here
            # no need to define the wpg since there is only one wpg in run_2/esdf  
        rlca:                   # other planners will be from "folder_3/" (see default config)
            model:     "rlca"
            linestyle: "tab:blue,--"
            wpg:       "spatialhorizon"
        mpc:
            model:     "mpc"
            linestyle: "tab:grey,-."
        teb:
            model:     "mpc"
            linestyle: "tab:grey,-."


# We can easily customize the output of the eval script by creating a similar yml config. 
# The user can combine any planner with anny figure even comparing planners from different runs.
# If the corresponding bag exists, any combination will work.
```
<!-- <img width="400" height="400" src="/arena_navigation/arena_local_planner/evaluation/plots/test_plots/map1_obs20_vel02_testplot1.png">
<img width="400" height="400" src="/arena_navigation/arena_local_planner/evaluation/plots/test_plots/map1_obs20_vel02_testplot2.png">
<img width="400" height="400" src="/arena_navigation/arena_local_planner/evaluation/plots/test_plots/map1_obs20_vel02_testplot3.png"> -->

<p float="left">
  <img src="/arena_navigation/arena_local_planner/evaluation/plots/test_plots/map1_obs20_vel02_testplot1.png" width="320" />
  <img src="/arena_navigation/arena_local_planner/evaluation/plots/test_plots/map1_obs20_vel02_testplot2.png" width="320" /> 
  <img src="/arena_navigation/arena_local_planner/evaluation/plots/test_plots/map1_obs20_vel02_testplot3.png" width="320" />
</p>

Here, you can see the three figures generated using the test cfg. First figure uses the default cfg, 2nd figure uses a custom cfg including collision zones and the static map. Figure three uses the default cfg again and uses the cadrl model from run 2. 
