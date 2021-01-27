# -*- coding: utf-8 -*-
"""
Created on Thu Jun 13 10:21:40 2019

@author: FujiiChang
"""

import sys
import subprocess

class WT_sensor:
    
    def __init__(self):
        self.SENSOR_ID = "28-020992450e02"
        self.SENSOR_W1_SLAVE = "/sys/bus/w1/devices/" + self.SENSOR_ID + "/w1_slave"
        self.ERR_VAL = 85000
        
        self.wt = 0.0
        
    def observation(self):
        res = self.get_water_temp()
        if res is not None:
            str_res = res.decode()
            temp_val = str_res.split("=")
            if temp_val[-1] == self.ERR_VAL:
                print("ERROR: Got value:85000. Circuit is ok, but something wrong happens...")
                sys.exit(1)
    
            temp_val = round(float(temp_val[-1]) / 1000, 1)
            return temp_val
        else:
            print("ERROR: Cannot read the value.")
            sys.exit(1)

    def get_water_temp(self):
        #try:
        res = subprocess.check_output(["cat", self.SENSOR_W1_SLAVE])
        return res
        """
        except:
            print("ERROR: Cannot get water temperature")
            return None
        """
        
    def update_wt(self):
        self.wt = self.observation()
        
