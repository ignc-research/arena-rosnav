'''
    @name:      noise.py
    @brief:     This class adds noise to the received sensor signal
    @author:    Chang Liu
    @version:   3.7
    @date:      2020/12/14
'''
# python relevant
import numpy as np
import csv
import random

class Noise:
    """
    This class adds noise to the received sensor data.
    """
    def __init__(self, 
                 noise_mode,
                 max_value_of_data = 0,
                 gauss_mean = 0,
                 gauss_sigma = 1,
                 gauss_size = 0.015,  #0.015  
                 bias_noise = 0.1,   #0.1
                 offset_noise = 0.01, #0.01
                 angle_noise = 0.122): #0.122
        # Class variables
        self._noise_mode = noise_mode                      # Mode of noise
        self._max_value_of_data = max_value_of_data        # usually dont need to change,it will update itself. max value of the sensor data,prepare for standaraization.
        self._gauss_mean = gauss_mean
        self._gauss_sigma = gauss_sigma
        self._gauss_size = gauss_size
        self._bias_noise = bias_noise
        self._offset_noise = offset_noise
        self._angle_noise = angle_noise
        
        self._noise_count = -1
        
    def add_noise(self,scan_msg):
        #caculate the size sf msg,and create file to save data
        if self._noise_count == -1:
            self.__initialsing_noise(scan_msg)     
        scan_msg_data = np.array(scan_msg.ranges)

        if 1 in self._noise_mode:
            scan_noise_msg = self.__gaussian_noise(scan_msg_data)
            scan_msg_data = scan_noise_msg
            #print("Gaussian noise has been added")
        if 2 in self._noise_mode:
            scan_noise_msg = self.__bias_noise(scan_msg_data)
            scan_msg_data = scan_noise_msg
            print("bias noise has been added")
        if 3 in self._noise_mode:
            scan_noise_msg = self.__offset_noise(scan_msg_data)
            scan_msg_data = scan_noise_msg
            print("offset noise has been added")
        if 4 in self._noise_mode:
            scan_noise_msg = self.__angle_noise(scan_msg_data)
            scan_msg_data = scan_noise_msg
            print("angle noise has been added")
            
        #self.__save_data_for_plot(scan_msg.ranges,scan_noise_msg)
        return scan_noise_msg
        
    def __gaussian_noise(self,scan_msg):
        '''
        This function is used to simulate ranging noise.It is a noise that fits a Gaussian normal distribution.
        This function will generate Gaussian white noise
        input :
            scan_msg : Original scan data
        return:
            gaussian_out : Gaussian noise data 
        '''
        self.__change_noise()
        # Generate Gaussian noise
        noise = np.random.normal(self._gauss_mean, self._gauss_sigma, scan_msg.shape)
        noise = noise * self._gauss_size * self._noise_change
        print(self._noise_change)
        gaussian_out = scan_msg + noise
        # Set more than 1 to 1, and less than 0 to 0
        gaussian_out = np.clip(gaussian_out, 0,  self._max_value_of_data)
        return gaussian_out
    
    def __bias_noise(self,scan_msg):
        '''
        This function is used to simulate Physical offsets.
        This function will generate a fixed value of noise.
        input :
            scan_msg : Original scan data
        return:
            bias_out : Bias noise data
        '''
        # Generate bias noise
        bias_out = scan_msg + self._bias_noise
        bias_out = np.clip(bias_out, 0, self._max_value_of_data)
        return bias_out
    
    def __offset_noise(self,scan_msg):
        '''
        This function is used to model the linear error,it can be simulated by Random Walk errors.
        This function will generate an average random noise within a set range.
        input :
            scan_msg : Original scan data
        return:
            offset_out : offset noise data
        '''

        # Generate offset noise
        for i in range(len(scan_msg)):
            step = self._offset_noise if random.randint(0, 1) else -self._offset_noise
            self._distance[i] += step
        offset_out = scan_msg + self._distance
        offset_out= np.clip(offset_out, 0, self._max_value_of_data)
        return offset_out

    def __angle_noise(self,scan_msg):
        '''
        This function is used to model to simulate angular errors, which are made up of mechanical horizontal and vertical errors.
        This function will generate Gaussian white noise
        input :
            scan_msg : Original scan data
        return:
            bias_out : angle noise data
        '''
        # standardization
        angle_noise = [0] * len(scan_msg)
        # Generate angle noise
        for i in range(len(scan_msg)):
            noise = np.random.normal(0, 1) *scan_msg[i] * self._angle_noise*0.01
            angle_noise[i] = scan_msg[i] + noise 
            angle_noise[i] = np.clip(angle_noise[i], 0, self._max_value_of_data)
        return angle_noise

    def __save_data_for_plot(self,scan_msg,scan_noise_msg):
        '''
        This function is used to save the original and noise data,then programmer can use these to analyse
        input :
            scan_msg : the original scan data
            scan_noise_msg : the noise scan data
        return:
            
        '''
        self._list_for_noise[self._noise_count] = scan_noise_msg
        self._list_for_original[self._noise_count] = scan_msg
        
        if self._noise_count % 50 == 0:
            with open(self._Original_data_address,'w',newline='') as Original_data: 
                Original_writer = csv.writer(Original_data)
                Original_writer.writerows(self._list_for_original)
                Original_data.close()  
        
            with open(self._Noise_data_address,'w',newline='') as Noise_data:
                Noise_writer = csv.writer(Noise_data)
                Noise_writer.writerows(self._list_for_noise)
                Noise_data.close()  
            
        self._noise_count = (self._noise_count+1) % 300
    
    def __initialsing_noise(self,scan_msg):
        self._max_value_of_data = scan_msg.range_max
        self._noise_count += 1
        self._Original_data_address = 'Original_data_for_plot.csv'
        self._Noise_data_address = 'Noise_data_for_plot.csv'
        self.__initialising_csv_files()
        
        self._list_for_original = [([0] * len(scan_msg.ranges)) for _ in range(300)]
        self._list_for_noise = [([0] * len(scan_msg.ranges)) for _ in range(300)]

        self._distance = [0] * len(scan_msg.ranges) #initial the error for the offset_noise
        
    def __initialising_csv_files(self):
        with open(self._Original_data_address,'w') as Original_data:
            Original_data.close()
        with open(self._Noise_data_address,'w') as Noise_data:
            Noise_data.close()  
            
    def __change_noise(self):
        with open("noise_parameter",'r') as noise_data:
            self._noise_change = int(noise_data.read())
            noise_data.close()

