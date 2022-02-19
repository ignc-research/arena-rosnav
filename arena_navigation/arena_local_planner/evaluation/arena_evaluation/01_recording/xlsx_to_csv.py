import glob
import os
import pandas as pd

files = glob.glob("{0}/*.xlsx".format(os.path.dirname(os.path.abspath(__file__)))) # get all the xlsx files paths in the directory where this script is located

for file in files:
    df = pd.DataFrame(pd.read_excel(file))
    file_name_path = file.split(".xlsx")[0]
    df.to_csv(file_name_path + ".csv", index = False) 
    os.remove(file)