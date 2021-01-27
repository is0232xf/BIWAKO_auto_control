# -*- coding: utf-8 -*-
"""
Created on Fri Aug 30 15:51:35 2019

@author: FujiiChang
"""

import time
import datetime
import signal
import csv
from wt_sensor_class import WT_sensor

wt_sensor = WT_sensor()
data_list = []

def get_wt_data(arg1, args2):
    now = datetime.datetime.now()
    timestamp = now.strftime("%Y/%m/%d/%H:%M:%S")
    wt_sensor.update_wt()
    wt = wt_sensor.wt
    print("Water temperature: ", wt)
    data_list.append([timestamp, wt]) 
    
def wt_signal():
    signal.signal(signal.SIGALRM, get_wt_data)
    signal.setitimer(signal.ITIMER_REAL, 1, 1)

def return_file_path():
    todaydetail = datetime.datetime.today()
    date = todaydetail.strftime("%Y_%m_%d_%H_%M_%_S.csv")
    path = "./log_data/" + date
    return path
    
if __name__ == "__main__":
    try:
        filename = return_file_path()
        f = open(filename, 'a')
        csvWriter = csv.writer(f) 
        csvWriter.writerow(['timestamp', 'temperature'])
        
        wt_signal()

        while True:
            pass

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        print("Finish sensing")      
        for i in range(len(data_list)):
            timestamp = data_list[i][0]
            wt = data_list[i][1]
            csvWriter.writerow([timestamp, wt])
        f.close()
