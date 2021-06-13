import csv
import numpy as np
# 'w' = write mode
class CSVWriter():
    def __init__(self):
        # create evaluation.csv file
        with open('evaluation.csv', 'w+', newline = '') as fp:
            a = csv.writer(fp, delimiter = ',')
            data = [['episode', 'nearest distance for adult','nearest distance for child','nearest distance for elder',
            'nearest distance for forklift','nearest distance for servicerobot','nearest distance for random_wanderer','reward',
            'safe distance adult','safe distance child','safe distance elder','safe distance forklift','safe distance servicerobot' 
            ,'safe distance random_wanderer','done_reason']]
            a.writerows(data)

    def addData(self, data:np.array):
        #add new data to the file 
        with open('evaluation.csv', 'a+', newline = '') as fp:
            a = csv.writer(fp, delimiter = ',')
            data =data.reshape(1,-1)
            a.writerows(data)
