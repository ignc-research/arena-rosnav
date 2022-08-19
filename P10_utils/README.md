# IAS NaviPrediction


## Pipeline Draft

<img width="500" alt="IAS_Naviprediction_v3 1" src="https://user-images.githubusercontent.com/13345/179000085-e8a09b9b-4030-4e63-9311-479b952fc3b4.png">


## Date augmentation with GAN (Generative adversarial networks)
A deep learning approach was used to complement the data set. for this approach the method Generative adversarial networks was chosen. to use the script, the file `gan.py` should be executed. 
The arguments are as follows:
- --image_path: path of dataset.
  - for generating the maps, there is in the folder data_augmentation_GAN, a folder with dataset. you can use this as a dataset or use your own maps in .png format. all maps (images) should be in one folder.
- --output_path: path of the output folder.
### Example:
```bash
python data_augmentation_GAN/gan.py --image_path /path/to/dataset --output_path /path/to/output
```
## Filter of maps
Second step is to filter the generated maps. to filter the maps, the script `filter.py` should be used. 
The arguments are as follows:
- --image_path: path of generated images.
- --output_path: path of the output folder.
### Example:
```bash
python data_augmentation_GAN/filter.py --image_path /path/to/generated_images --output_path /path/to/output
```

## Create yaml file
For the world complexity metrics, the maps should still have yaml files. 
to create yaml files, you should run the script `create_yaml.py`.
he arguments are as follows:
- --image_path: path of images.
### Example:
```bash
python create_yaml.py --image_path /path/to/folder/of/images
```
## calc_ComplexityValues.py

This script calculates the complexity values of the given maps in a folder and writes them into a csv-file as input for the DNN.

### Example:
```bash
python3 calc_ComplexityValues.py folderpath
```

## World complexity

in order to calculate the metrics of worlds, script `world.complexity.py` should be executed. 
the arguments are as follows:
- --image_path: path of the image.
- --yaml_path: path of the map.yaml file.
- --dest_path: path of the output folder.
- --folders_path: if each maps and associated yaml file are stored in a folder and separated from other files, you can use this argument. this way all folders will be loaded automatically and complexity metrics will be calculated.
  - if this argument is used, other arguments must not be specified

the name of yaml file must be map.yaml, if your file has another name, please rename it to map.yaml
### Example:
```bash
python world_complexity.py --image_path /path/to/folder/of/images --yaml_path path/to/map.yaml --dest_path path/to/output/folder
```
or 
```bash
python world_complexity.py --folders_path /path/to/folders/of/maps
```
### Step 1
1. Install [world_complexity.py](https://github.com/ignc-research/arena-evaluation/blob/main/static-world-complexity/world_complexity.py) script

### Step 2
1. Please read those [notes](https://drive.google.com/drive/folders/1Sw-r-8_AGxYAbrW0-csTRDjSVWHEy0A1?usp=sharing)
1. install and validate the package

### Step 3
1. Test script `world_complexity.py` (what are the limits of the script? what could be improved?)
1. Create enough maps (~100 - 200)
1. Collect values from maps via world_complexity.py
1. Test, if the collected values can be added to one value or if a CNN is necessary
