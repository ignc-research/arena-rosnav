# -*- coding: utf-8 -*-
"""
    @name:      plot.py
    @brief:     This file plot the noise data and original data from sensor
    @author:    Chang Liu
    @version:   3.7
    @date:      2020/12/16
"""
import matplotlib.pyplot as plt
import csv

def get_data(column):
    Original_data_address = 'Original_data_for_plot.csv'  
    Noise_data_address = 'Noise_data_for_plot.csv'
    with open(Original_data_address) as f: 
        reader=csv.reader(f)  
        Original_dates = []      
        for row in reader:  
            Original_dates.append(float(row[column]))   
        f.close()
    with open(Noise_data_address) as f: 
        reader=csv.reader(f)  
        Noise_dates = []      
        for row in reader:  
            Noise_dates.append(float(row[column]))   
        f.close()
    return Original_dates,Noise_dates
        
def plot(column):
    plt.figure(dpi=85,figsize=(8,6))  
    plt.ion()
    for _ in range(1000):
        plt.cla()
        Original_dates,Noise_dates = get_data(column)
        plt.plot(Original_dates,c='red',label = 'Original_dates',alpha=0.5)
        plt.plot(Noise_dates,c='blue',label = 'Noise_dates',alpha=0.5)  
        plt.title('Data from ideal sensor and after adding noise',fontsize=20)  
        plt.xlabel('Time',fontsize=16)  
        plt.ylabel('Laser sensor values',fontsize=16)  
        plt.legend()
        plt.pause(0.5)
    plt.ioff()
    plt.show()
    return
        

if __name__ == "__main__":
    column = int(input("Please enter the sensor number you wish to display. (Select from 0-90):"))
    plot(column)

    
    
    