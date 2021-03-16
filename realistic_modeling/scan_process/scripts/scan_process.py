#!/usr/bin/env python
'''
    @name:      scan_process.py
    @brief:     This class accepts, processes and sends laser sensor information.
    @author:    Chang Liu Ke Wang Shaungrui Liu Jiajing Jiang
    @version:   3.7
    @date:      2021/2/10
'''

## sensor data processing that listens to sensor_msgs/LaserScan published to the 'scan' topic

import rospy
import json
import os
import collections

# observation msgs
from sensor_msgs.msg import LaserScan

#helper python
from noise import Noise


class Scan_process():

    def __init__(self, noise_model,timedelay):

        self._noise_model = self._identify_noise_model(noise_model)
        self._lst = collections.deque()                                        #the storage "cache" for the data with noise
        self._time_delay = timedelay                                           #the delay in the data transfering

        self.noise_parameter_address =os.path.join( os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".."),'config/noise_parameter.json')
        if 0 not in self._noise_model:    
            self._setup_by_configuration(self.noise_parameter_address)
            self.Noise_Generation = Noise(noise_mode = self._noise_model,
                                          gauss_mean = self._gauss_mean,
                                          gauss_sigma = self._gauss_sigma,
                                          gauss_size = self._gauss_size,
                                          bias_noise = self._bias_noise,
                                          offset_noise = self._offset_noise ,
                                          angle_noise = self._angle_noise)
           
    def _add_noise_and_publish(self,data):
        '''
        This function adds noise to the incoming sensor information
        '''
        if 0 not in self._noise_model:
            data.ranges = self.Noise_Generation.add_noise(data)
        
        self._add_delay_and_publish(data)                                          

    def _add_delay_and_publish(self,data):
        '''
        This function adds delay to the incoming sensor information and then publishes
        '''
        pub = rospy.Publisher('scan', LaserScan)
        if self._time_delay != 1:
            self._lst.append(data)
            if len(self._lst) == self._time_delay:
                pub.publish(self._lst.popleft())                                   # creat publisher ,add time dalay to the publisher,every time the publisher publish the msgs, when there are 10 data in cache creat
        else:
            pub.publish(data) 
            
    def _listener(self):
        '''
        This function receives the data from the original scan sensor
        '''
        rospy.Subscriber('scan_original', LaserScan, self._add_noise_and_publish)
        rospy.spin()                                               #simply keeps python from exiting until this node is stopped
        
    def _identify_noise_model(self,noise_model):
        '''
        This function defines the type of noise
        input :
            noise_model : noise model
            noise_model contains 0 means without noise
            noise_model contains 1 means with gaussian noise
            noise_model contains 2 means with offset noise
            noise_model contains 3 means with angle noise
            noise_model contains 4 means with bias noise
            it can contain several different noises at the same time
        '''
        ans = []
        if isinstance(noise_model,int):
            if noise_model == 0:
                ans.append(0)
            while noise_model != 0:
                ans.append(noise_model % 10)
                noise_model = noise_model // 10
        elif isinstance(noise_model,str):                                      #identify the noise_model is str
            pass
        
        return ans
    
    def _setup_by_configuration(self, noise_parameter_json_path):
        '''
        This function Read the noise parameter from the json file
        '''
        with open(noise_parameter_json_path, 'r',encoding = 'utf-8_sig') as f:
            lines = []
            for row in f.readlines():
                if row.strip().startswith("//"):
                    continue
                lines.append(row)
            d = json.loads("\n".join(lines))            
            self._max_value_of_data = float(d['max_value_of_data'])   
            self._gauss_mean = float(d['gauss_mean'])
            self._gauss_sigma = float(d['gauss_sigma'])
            self._gauss_size = float(d['gauss_size'])
            self._bias_noise = float(d['bias_noise'])
            self._offset_noise = float(d['offset_noise'])
            self._angle_noise = float(d['angle_noise'])
            f.close()



if __name__ == '__main__':

    rospy.init_node('noise', anonymous = False)
    noise_model = rospy.get_param('noise_mode')
    timedelay = rospy.get_param('delay')
    Scan_process = Scan_process(noise_model = noise_model,timedelay = timedelay)
#    Scan_process = Scan_process()

    while not rospy.is_shutdown():
        Scan_process._listener()

       
