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

path for eval_human_safety_dist:
data/evaluation.csv
plot.py

# usage:
python plot.py
rename:
path = 'evaluation_360/' for directory of plts
data = pd.read_csv('data/evaluation_360.csv') for directory of data


