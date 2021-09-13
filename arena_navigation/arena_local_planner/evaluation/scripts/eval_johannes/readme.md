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
python <path_to_script/ods.eval.py> <path_to_ods_file_directory/ods_files>
```

The script will return a directory with one .png file per metric in it.