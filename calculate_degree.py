# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 15:31:53 2018

@author: FujiiChang
"""

import math
import numpy as np

# use Hubeny theory
# e_radius is equatorial radius
# p_radius is polar radius
def huveny_distance(target_point, current_point):
    e_radius = 6378137.0
    p_radius = 6356752.314140
    dy = target_point[0] - current_point[0]
    dx = target_point[1] - current_point[1]
    my = (target_point[0] + current_point[0]) / 2
    e = math.sqrt((e_radius**2 - p_radius**2) / e_radius**2)
    W = math.sqrt(1 - (e * math.sin(my))**2 )
    M = e_radius * (1 - e**2) / W**3
    N = e_radius / W
    d = math.sqrt((dy * M)**2 + (dx * N * math.cos(my))**2)
    # translate m to km
    d = d / 1000
    return d

# translate sexagesimall to decimal
def translate_sexagesimal_to_decimal(degval):
    decimal, integer = math.modf(degval/ 100)
    decimal_val = integer + decimal / 60.0 * 100.0
    return decimal_val
    
def translate_GPGGA_to_decimal(GPGGAval):
    decimal_longtitude = translate_sexagesimal_to_decimal(GPGGAval[0])
    decimal_latitude = translate_sexagesimal_to_decimal(GPGGAval[1])
    decimal_val = [decimal_longtitude, decimal_latitude]
    return decimal_val

# translate degree to radian    
def translate_deg_to_rad(deg_val):
    rad = deg_val * math.pi / 180
    return rad

def translate_decimal_to_rad(decimal_val):
    rad_longtitude = translate_deg_to_rad(decimal_val[0])
    rad_latitude = translate_deg_to_rad(decimal_val[1])
    rad_val = [rad_longtitude, rad_latitude]
    return rad_val

# Using Haversine fomula
def calculate_bearing(target_point, current_point):
    lat1 = math.radians(target_point[0])
    lat2 = math.radians(current_point[0])
    lon1 = math.radians(target_point[1])
    lon2 = math.radians(current_point[1])   
    dlon = lon2 - lon1
    bearing = 90 - math.degrees(math.atan2(math.cos(lat1)*math.sin(lat2)
                 -math.sin(lat1)*math.cos(lat2)*math.cos(dlon), math.sin(dlon)*math.cos(lat2)))
    return bearing

def limit_angle(angle_in):
    angle_out = np.absolute(angle_in)
    while angle_out > np.pi:
        angle_out -= np.pi * 2

    if angle_in < 0:
        angle_out *= -1

    return angle_out
