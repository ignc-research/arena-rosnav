import numpy as np 
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import csv
import os
import pandas as pd
import random 
import matplotlib.path as mpath
import matplotlib.patches as mpatches
from scipy.interpolate import make_interp_spline, BSpline

pts = [(0,0), (0,1), (3,1), (3,0)] # Corners of rectangle of height 1, length 3
apts = np.array(pts) # Make it a numpy array
lengths = np.sqrt(np.sum(np.diff(apts, axis=0)**2, axis=1)) # Length between corners
total_length = np.sum(lengths)