# Arena Evaluation
The Arena Evaluation package consists of 3 parts:
- data recording
- data transformation and evaluation
- plotting
## Prerequisites

```
workon rosnav
pip install scikit-learn seaborn pandas matplotlib
```

## 01 Data Recording
To record data as csv file while doing evaluation runs set the flag `use_recorder:="true"` in your `roslaunch` command. For example:

```
workon rosnav
roslaunch arena_bringup start_arena_gazebo.launch world:="aws_house" scenario_file:="aws_house_obs05.json" local_planner:="teb" model:="turtlebot3_burger" use_recorder:="true"
```

The data will be recorded in `.../catkin_ws/src/arena-rosnav-3D/arena_navigation/arena_local_planner/evaluation/arena_evaluation/01_recording`.
The script stops recording as soon as the agent finishes the scenario and stops moving or after termination criterion is met. Termination criterion as well as recording frequency can be set in `data_recorder_config.yaml`.

```
max_episodes: 15 # terminates simulation upon reaching xth episode
max_time: 1200 # terminates simulation after x seconds
record_frequency: 0.2 # time between actions recorded
```

After doing all evaluation runs please remember to push your csv files or gather them in `/01_recording`.

NOTE: Leaving the simulation running for a long time after finishing the set number of repetitions does not influence the evaluation results as long as the agent stops running. Also, the last episode of every evaluation run is pruned before evaluating the recorded data.

NOTE: Sometimes csv files will be ignored by git so you have to use git add -f <file>. We recommend using the code below.
```
roscd arena_evaluation
git add -f .
git commit -m "evaluation run"
git pull
git push
```

## 02 Data Transformation and Evaluation
After finishing all the evaluation runs, recording the desired csv files and allocating them in `/01_recording` you can run the `get_metrics.py` script in `/02_evaluation`.
This script will evaluate the raw data recorded from the evaluation runs und summarize them in a compact dataset for each map, planner and scenario, which are all stored in one single JSON file with the following naming convention: `data_<timestamp>.json`. During this process all the csv files will be moved from `/01_recording` to `/02_evaluation` into a directory with the naming convention `data_<timestamp>`. The JSON file will be stored in `/02_evaluation`.

Some configurations can be set in the `get_metrics_config.yaml` file. Those are:
- `robot_radius`: dictionary of robot radii, relevant for collision measurement
- `time_out_treshold`: treshold for episode timeout in seconds
- `collision_treshold`: treshold for allowed number of collisions until episode deemed as failed

NOTE: Do NOT change the `get_metrics_config_default.yaml`!

We recommend using the code below:
```
workon rosnav
roscd arena_evaluation/02_evaluation
python get_metrics.py
```

NOTE: If you want to reuse csv files, simply move the desired csv files from the data directory to `/01_recording` and execute the `get_metrics.py` script again.

## 03 Plotting
The `get_plots.py` script grabs all `data.json` files located in `/02_evaluation` and moves them to `/03_plotting/data`. During the process the last in order JSON file from the grabbed files will be deemed as "most recent" file. If no file was grabbed, the last data.json used for plotting will remain the "most recent" file. Alternatively, it's possible to specify a `data.json` to be used for plotting. To specify a dataset set the following keys in the `get_plots_config.yaml`:

```
specify_data: true
specified_data_filename: <your_dataset>.json
```

For runnning the script recommend using the code below:
```
workon rosnav
roscd arena_evaluation/03_plotting
python get_plots.py
```

### Mandatory fields:
- `labels`
- `color_scheme`

Make sure for those fields **all** your local planner or planner-waypoint-generator combinations with the robot they were used on are defined. Examples:
- labels:
    - rlca_jackal: RLCA
    - rlca_turtlebot3_burger: RLCA
- color_scheme:
    - rlca_jackal
    
### Fields explained ###
The script generates quantitative (pathing) and quantitative (metrics) plots into a directory in `/03_plotting` with the naming convention `plots_<timestamp>`.
There are many configurations that can be set int `get_plots_config.yaml`. They will be listed below:

- `collision_alpha`: alpha of collision points (QUALI)
- `collision_size`: size of collision points (QUALI)
- `collision_zone_alpha`: alpha of collision zones (QUALI)
- `collision_zone_base_diameter`: base diameter of collision zones, total diameter = base diameter x number of collisions (QUALI)
- `color_scheme`: color scheme for planner or planner-waypoint-generator combination, examle below
    - drl: tab:blue
    - drl_spatial_horizon: tab:blue
- `goal_marker`: marker for the goal (QUALI)
- `goal_point_color`: color of goal (QUALI)
- `goal_size`: size of goal (QUALI)
- `labels`: labels for planner and planner-waypoint-generator combination, example below:
    - drl: DRL
    - drl_spatial_horizon: DRL Spatial Horizon

- `leave_out_metric`: metric to be left out from plotting (QUANTI)
- `leave_out_planner`: planner or planner-waypoint-generator combination to be left out, example below
  - drl
  - drl_subsampling
- `most_recent_file`: DO NOT modify this
- `obstacle_color`: color of obstacles (QUALI)
- `obstacle_radius`: radius of obstacles (QUALI)
- `path_alpha`: alpha of robot path (QUALI)
- `path_arrow_color`: arrow color of dynamic obstacle path (QUALI)
- `path_arrow_size`: size of dynamic obstacle path arrow (QUALI)
- `path_size`: size of robot path (QUALI)
- `plot_barplot_alpha`: alpha for barplots (QUANTI)
- `plot_barplot_capsize`: barplot error bar capsize (QUANTI)
- `plot_barplot_errorcolor`: barplots error bar color (QUANTI)
- `plot_barplot_width`: barplot width (QUANTI)
- `plot_collision_zones`: option to lot collision zones (QUALI)
- `plot_collisions`: option to plot collision points (QUALI)
- `plot_progression`: option to plot robot progression (QUALI)
- `plot_qualitative`: option to plot qualitative plots (QUALI)
- `plot_qualitative_axes`: option to plot axes (QUALI)
- `plot_qualitative_axes_size`: axes size (QUALI)
- `plot_qualitative_legend`: option to plot legend (QUALI)
- `plot_qualitative_legend_location`: legend position (QUALI)
- `plot_qualitative_title`: option to plot title (QUALI)
- `plot_qualitative_title_size`: title size (QUALI)
- `plot_quantitative`: option to plot quantitative plots (QUANTI)
- `plot_quantitative_axes_label_size`: axis label size (QUANTI)
- `plot_quantitative_axes_tick_size`: axis ticklabel size (QUANTI)
- `plot_quantitative_labels`: labels for metrics used in axis label and title (QUANTI)
- `plot_quantitative_suptitle`: option to plot title (metric name) (QUANTI)
- `plot_quantitative_suptitle_size`: title size (QUANTI)
- `plot_quantitative_title`: option to plot subtitle (map, obstacle number, velocity) (QUANTI)
- `plot_quantitative_title_size`: subtitle size (QUANTI)
- `plot_quantitative_violin`: option to use violin plots instead of barplots (QUANTI)
- `plot_quantitative_violin_inner`: choose inner representation of violinplots {"box", "quartile", "point", "stick", None} (QUANTI)
- `plot_quantitative_ygrid`: option to plot horizontal grid (QUANTI)
- `plot_success_alpha`: alpha for success barplots (QUANTI)
- `plot_success_collision_color`: color for collision in success plot (QUANTI)
- `plot_success_legend`: option to plot legend for success plot (QUANTI)
- `plot_success_legend_location`: legend location for success plot (QUANTI)
- `plot_success_success_color`: color for success in success plot (QUANTI)
- `plot_success_time_out_color`: color for timeout in success plot (QUANTI)
- `plot_success_width`: barwidth for success plot (QUANTI)
- `progression_size`: size for progression of robot (QUALI)
- `progression_steps`: number of timesteps between each progression point (QUALI)
- `start_marker`: marker for start point (QUALI)
- `start_point_color`: color for start point (QUALI)
- `start_size`: size of start point (QUALI)
