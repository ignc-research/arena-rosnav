# Quantitative Plotting of .ods files
This `ods_eval.py` script creates quantitative plots (bar plots) from .ods files.
The format of the ods files should be as follows:
- first column: planners
- other columns: metrics to be evaluated
- the .ods file name should be distinguishable and contain information about the obstacle number used.
- naming of the planners and metrics must be consistent

## Config
In the `eval_config.yaml` file you have to specify the color_mapping and label_mapping.
In the label_mapping section you must assign each planner and metric a label, which will then be used in the plot.
You must also assign a label to the .ods files which should be the obstacle number used.
It's possible to leave out planners which you have data for but don't want to include in the plots. Simply create a list of unwanted planners in the leave_out_planners section.

Note: KeyErrors mostly appear due to missing assignments or typos in the assignments.

## How to use the script
```
python <path_to_script/ods.eval.py> <path_to_ods_file_directory>
```

The script will return a directory with one .png file per metric in it.

# Plot drl-teb distribution as pie charts
The script `plot_pies.py` plots the drl-teb distribution as pie charts.
Color scheme is the same as defined in `eval_config.yaml` for drl and teb.

## How to use the script
```
python <path_to_script/ods.eval.py> <path_to_parent_directory_to_ods_file_directory>
```

Example:
```
$HOME/arena_ws/src/arena-rosnav/arena_navigation/arena_local_planner/evaluation/scripts/eval_johannes/ods_files_210913
```

You need to specify the subdirectory names, e.g. indoor and outdoor, in the subdirectory_names section. Labels for the pie charts must also be defined in the pie_labels section of the `eval_config.yaml`. 

The script will return a .png file with the pie charts.