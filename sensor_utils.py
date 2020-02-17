# -*- coding: utf-8 -*-
"""
Created on Tue Jul 23 20:05:30 2019

@author: Sean Nichols
"""

import numpy as np

speed_of_sound = 343 #m/s

def get_distance(a, sensor_pins, n_samples=1):
    
    distances = []
    for i in range(n_samples):
        # Get the echo time delay
        d_time = a.echoRead(sensor_pins)
        # Convert time (us) to distance (m)
        distances = np.append(distances,
                              speed_of_sound * (1e-6 * int(d_time)) / 2)
    
    return distances.mean()