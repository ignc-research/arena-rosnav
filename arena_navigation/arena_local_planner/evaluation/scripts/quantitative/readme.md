# preparations:
## 1. make sure you have all these packages installed
```
matplotlib.pyplot
numpy
pandas
argparse
glob
```
## 2. make sure all json files you want to evaluate lie in one directory

path for run 3:
arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3

## 3. make sure all json files consistently follow a naming convention

example: "localPlanner_map_obsXX_velXX_waypointGenerator.json"

Labeling can be later specified in the script, just make sure that the naming is consistent throughout the files.

# specifications in the script
Go to the main function (bottom in the script) and specify values for the following:
- cols:		(usually not necessary to change), all metrics that are measured in the json
- obs:		values for the obstacle number
- vel:		values for the velocities
- maps:		maps
- wpgen:		waypoint generators, special case: set to classic only if you dont want to evaluate several waypoint generators
- planner:	planners
- classic:	which of the planners are classic planners (no waypoint generator), special case: classic = planner
- labels:	labels which will be used in the plots as e.g. labels, titles, legend
- colors:	color scheme for the planners
- colors_wp:	color scheme for the waypoint generators if you want to plot with --byplanner

# usage:
python sim_evaluation_v2.py [path] --metrics time path collision success --quantity obs --latex --csv --withclassic --byplanner --allplot_quantity obs
- --metrics	
	- plot these metrics, if not specified nothing will be plotted, none by default
- --quantity	
	- specify quantity over which shall be evaluated/plotted, by default obs
- --latex		
	- flag: creates latex table of data, format specified (changes need to be done manually), off by default
- --csv		
	- flag: creates csv files: all data, data of top table of latex table, data of bottom table of latex table, off by default
- --withclassic	
	- flag: plot all combinations waypoint generator and local planners always WITH classic planners as well, off by default
- --byplanner	
	- flag: plot for each local planner all combinations with waypoint generators, by default: plot for each waypoint generator all combinations with local planners
- --allplot_quantity
	- flag: plot a all in one plot, default: none
	- also: specify for which quantity the all-in-one plot shall be plotted
	- uses same metrics as specified by --metrics

other options:
- --legendsoff	
	- flag: no legends
- -- show		
	- flag: show the plots created (not recommended)
- -- nosubtitle	
	- flag: dont show the local planner/waypoint generator in the subtitle, NOTE: can be usefull if you dont have many waypoint generators

# Remarks
- Remark 1:
This script can be used to plot e.g. 5 local planner-waypoint generator combinations.
Therefore specify in the main function of the script every combination as planners as well as classic planners.
"classic" should the only waypoint generator.
The script might then show errors, when trying to run this script with --byplanner flag.

- Remark 2:
When json files are missing, the script will substitute the missing files with zero vectors and print a message in the console.

- Remark 3:
The flags --latex and --csv are sufficient to provide once.

- Remark 4:
The plots will be saved into the directory in which the script is run.

- If you want to get every possible plots:
```
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity obs
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity vel
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity obs --withclassic
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity obs --withclassic
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity obs --byplanner
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity vel --byplanner
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity obs --byplanner --withclassic
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity vel --byplanner --withclassic
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity vel --byplanner --withclassic --allplot_quantity obs
python sim_evaluation_v2.py [path] --metrics time success path collision --quantity vel --byplanner --withclassic --allplot_quantity vel
``` 
- example:
```
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity obs
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity vel
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity obs --withclassic
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity obs --withclassic
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity obs --byplanner
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity vel --byplanner
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity obs --byplanner --withclassic
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity vel --byplanner --withclassic
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity vel --byplanner --withclassic --allplot_quantity obs
python sim_evaluation_v2.py $HOME/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/run_3 --metrics time success path collision --quantity vel --byplanner --withclassic --allplot_quantity vel
```
# Color convention for plots

Colors for planners:
```
MPC  tab:red,
TEB  tab:blue,
RLCA tab:green,
R0   tab:pink,
R1   tab:purple,
R2   tab:orange,
R4   tab:cyan
```

# Error Handling
## Multiple json files for one simulation setup
The following error is caused by having multiple json with the same simulation specifications in the path directory. E.g. R2_map0_obs20_vel03_spatialhorizon_x.json and R2_map0_obs20_vel03_spatialhorizon.json.

```
Traceback (most recent call last):
  File "/home/ducanor/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/sim_evaluation_v3.py", line 606, in <module>
    plot_metrics(data,labels,colors,wpgen,planner,maps,param_list,quantity,metrics,legendsoff,show,classic,withclassic,byplanner,nosubtitle)
  File "/home/ducanor/catkin_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/quantitative/sim_evaluation_v3.py", line 325, in plot_metrics
    ax.bar(r[plan], oq_data[metric], width=barwidth, label=labels[plan],  color=colors[plan], alpha=1-a/l)
  File "/home/ducanor/python_env/rosnav/lib/python3.6/site-packages/matplotlib/__init__.py", line 1565, in inner
    return func(ax, *map(sanitize_sequence, args), **kwargs)
  File "/home/ducanor/python_env/rosnav/lib/python3.6/site-packages/matplotlib/axes/_axes.py", line 2341, in bar
    np.atleast_1d(x), height, width, y, linewidth)
  File "<__array_function__ internals>", line 6, in broadcast_arrays
  File "/home/ducanor/python_env/rosnav/lib/python3.6/site-packages/numpy/lib/stride_tricks.py", line 258, in broadcast_arrays
    shape = _broadcast_shape(*args)
  File "/home/ducanor/python_env/rosnav/lib/python3.6/site-packages/numpy/lib/stride_tricks.py", line 189, in _broadcast_shape
    b = np.broadcast(*args[:32])
ValueError: shape mismatch: objects cannot be broadcast to a single shape
```

Please make sure that for every simulation specification only one json file is delivered. For this purpose we prepared commands at the end of the sim_evaluation.py script to print out the whole pandas dataframe. Simply command out the plotting commands and run the sim_evaluation.py script. Now check for multiple entries and delete spare jsons.

## KeyError
When an KeyError occures its mostly involving the labels and/or color dictionary. Check your mappings whether they are complete and consistently named.
