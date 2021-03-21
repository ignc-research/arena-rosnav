import csv
import numpy as np
# create a file to open
# 'w' = write mode
# new line = blank
class CSVWriter():
    def __init__(self):
        # create evaluation.csv file
        with open('evaluation', 'w+', newline = '') as fp:
            a = csv.writer(fp, delimiter = ',')
            data = [['episode', 'nearest distance for adult','nearest distance for child','nearest distance for elder','reward','done_reason']]
            a.writerows(data)

    def addData(self, data:np.array):
        #add new data to the file 
        with open('evaluation', 'a+', newline = '') as fp:
            a = csv.writer(fp, delimiter = ',')
            data =data.reshape(1,-1)
            a.writerows(data)
