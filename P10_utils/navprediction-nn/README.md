# Navigation robot prediction project

## Introduction

This project is a neural network that predicts the success rate 
of a robot moving from one point to another in a given map.

## Prerequisites
- Python ~= 3.10.4
- poetry ~= 1.1.14
- CUDA enabled NVIDIA GPU
- CUDA Toolkit Version: 11.7 
- This code was tested on Ubuntu 22.04.1 LTS only. (jammy)

## About the dataset


The dataset is a collection of maps and their corresponding navigation metrics and navigation goals.
In the code, the map is termed as `img` and the navigation metrics are termed as `meta` or `metadata`
, while the navigation goals are termed as `target`.

- YAML files structure is one level deep, key value pairs where the value is always a float. The keys always named using snake case.
- We use UUID4 as the map identifier. Additionally, every file is named as `<uuid>_<file-name>.yml`.
- The total number of metric parameters (`metadata`) is `55` and the total number of goals (`target`) is `3`.
- Target parameters are stored in the `<UUID4>_performance_metrics.yml`.
- The image dimensions are `150x150` and the image is stored as a `PNG` file.
- The map image files always have the postfix `_map.png`. 

### Dataset structure

```
.
└── data
    ├── test
    │   ├── <UUID4>
    │   │   ├── <UUID4>_img.png
    │   │   ├── <UUID4>_map_complexity_metrics.yml
    │   │   ├── <UUID4>_map_obstacle_metrics.yml
    │   │   ├── <UUID4>_performance_metrics.yml
    │   │   └── <UUID4>_robot_metrics.yml
    │   └── ...
    ├── train
    │   ├── <UUID4>
    │   │   ├── <UUID4>_img.png
    │   │   ├── <UUID4>_map_complexity_metrics.yml
    │   │   ├── <UUID4>_map_obstacle_metrics.yml
    │   │   ├── <UUID4>_performance_metrics.yml
    │   │   └── <UUID4>_robot_metrics.yml
    │   └── ...
    └── val
        ├── <UUID4>
        │   ├── <UUID4>_img.png
        │   ├── <UUID4>_map_complexity_metrics.yml
        │   ├── <UUID4>_map_obstacle_metrics.yml
        │   ├── <UUID4>_performance_metrics.yml
        │   └── <UUID4>_robot_metrics.yml
        └── ...
```

## Setup

Configure the project with environment variables:

```bash
cp env.example .env
```

Set up your wandb API key to the .env file:

```toml
# Your wandb api key here. Get it from https://wandb.ai/authorize
WANDB_API_KEY="<your-api-key>"
```


Creating virtualenv:

```bash
poetry env use python3.10
```

Verify virtualenv:

```bash
poetry env info
```
The output should look something like this:

```bash
Virtualenv
Python:         3.10.4
Implementation: CPython
Path:           /home/<USER>/.cache/pypoetry/virtualenvs/navprediction-AuqfrZ_x-py3.10
Valid:          True

System
Platform: linux
OS:       posix
Python:   /usr
```

Then install dependencies:
    
```bash
poetry install
```

## Usage

Make sure to activate the virtualenv:

```bash
source $(poetry env info --path)/bin/activate
poetry env info # should show the virtualenv info
```

To train the model, run:

```bash
poetry run python main.py \
  --train \
  --data_root /home/amer/tmp/pycharm_project_navprediction/data/ \
  --log_level INFO \
  --num_workers 4 \
  --epochs 10
```

### More about the available arguments

```bash
poetry run python main.py  --help
```

```man
usage: main.py [-h] [--train] [--data_root DATA_ROOT] [--log_level {DEBUG,INFO,WARNING,ERROR,CRITICAL}] [--random_seed RANDOM_SEED]
               [--batch_size BATCH_SIZE] [--epochs EPOCHS] [--lr LR] [--num_workers NUM_WORKERS]

Navigation prediction experiment

options:
  -h, --help            show this help message and exit
  --train               Train model
  --data_root DATA_ROOT
                        Path to data root
  --log_level {DEBUG,INFO,WARNING,ERROR,CRITICAL}
                        Log level
  --random_seed RANDOM_SEED
                        Random seed
  --batch_size BATCH_SIZE
                        Batch size
  --epochs EPOCHS       Number of epochs
  --lr LR               Learning rate
  --num_workers NUM_WORKERS
                        Number of workers
```

## Unit tests

To run unit tests, run:

```bash
poetry run pytest
```
