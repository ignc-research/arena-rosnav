# Pipeline for generating training data for DNN

## Introduction

In this repository you will find all of the necessary scripts to generate training data for the Neural Network used for defining the navigation of the robots in arena-rosnav.

## Prerequisites
- Python 3.8.10
- arena-rosnav/navprediction branch
- arena-evaluation

## Files
```
.
└── pipeline_pkg
    ├── cliMapGenerator.py
    ├── createAverage.py
    ├── distance_between.py
    ├── world_complexity.py
    ├── pipeline_script.py
    ├── maps
    │    ├── <UUID4>
    │    │   ├── <UUID4>.png
    │    │   ├── map.world.yaml
    │    │   └── map.yaml
    │    └── ...
    └── sims_data_records
         ├── <UUID4>
         │   └── <UUID4>_<local_planner>_<robot>--<scenario>--<date>.png
         └── ...
```

- cliMapGenerator.py : Command line program that generates a random map given some parameters.
- createAverage.py : Program that takes the recorded data of each of the simulations, cleans it and extracts the most relevant data out of it.
- distance_between.py and world_complexity.py : Program that calculates specific metrics to estimate how difficult a particular map is to traverse.
- pipeline_script.py : Main pipeline program. Runs all the programs in the pipeline in a specific order and inputs the necessary parameters
- maps : Directory that contains the map files of each generated map. Each map is identified with a UUID.
- sims_data_records : Directory that contains the data that is recorded while running the simulations in each map.

## Usage
You must first go inside the pipeline_pkg directory in the terminal. From there you must run the following command:
```
python3 pipeline_script.py
```
```
usage: python3 pipeline_script.py [--num_maps NUM_MAPS] [--num_settings NUM_SETTINGS]

options:
  --num_maps        Number of maps you want to generate and run simulations on. Default value is 10.
  --num_settings    Number of times you want to run simulations on each map with different settings. Default value is 1. 
```