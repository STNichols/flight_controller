# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 14:31:52 2020

@author: Sean Nichols

The aircraft_sim class object serves as a simulation of the Arduino class
object described below:

Python-Arduino_Proto-API-v2
https://github.com/vascop/Python-Arduino-Proto-API-v2/
"""

import time
import numpy as np
import matplotlib.pyplot as plt

a_gravity = 9.81     # m/s^2
speed_of_sound = 343 #m/s
crit_ratio = 0.5 # proportion of max speed

min_power = 0   # Minimum power for analog write
max_power = 255 # Maximum power for analog write

plt.ion()

class aircraft_sim(object):
    
    def __init__(self):
        '''Define the parameters of the simulated arduino aircraft
        '''
        
        self.mass = 0.453592; # Set mass to 1 lb
        self.sim_start = time.time()
        self.last_update = self.sim_start
        self.motor_speed = 0
        self.sim_crit_power = crit_ratio * max_power
        
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        
        self.x_vel = 0
        self.y_vel = 0
        self.z_vel = 0        
        
        self.x_acc = 0
        self.y_acc = 0
        self.z_acc = -a_gravity
        
        # Setup logging of simulated data
        self.log_fig = plt.figure()
        self.log_ax = self.log_fig.add_subplot(111)
        self.log_ax.grid()
        self.t_log = np.array([])
        self.z_pos_log = np.array([])
        self.z_vel_log = np.array([])
        self.z_acc_log = np.array([])
        
    def update_state(self):
        ''' Kinematic Equations are used to update the state of the aircraft
        Key component missing is drag, minor compensation is ramp down velocity
        when acceleration since the last update is 0
        '''
        
        current_t = time.time()
        dt = current_t - self.last_update
        
        # If no time has passed, nothing to update
        if dt == 0:
            return
        
        # Z-Component
        z_pos_n = (self.z_vel * dt) + (0.5 * self.z_acc * dt ** 2) + self.z_pos
        z_vel_n = np.sqrt(self.z_vel ** 2 + 
                          (2 * self.z_acc * (z_pos_n - self.z_pos)))
        # Check for negative root of velocity calculation
        if (z_pos_n - self.z_pos) < 0:
            z_vel_n *= -1
            
        # Compensation for drag using drag factor (fraction of vel lost per sec)
        if self.z_acc == 0:
            drag_factor = 0.995
            # Create velocity vector at every 1/100th of a second
            z_vel_vec = self.z_vel * (drag_factor ** 
                                      np.array(range(int(np.ceil(dt * 100)))))
            # Integrate the velocity over time to find the change in position
            d_pos = np.trapz(z_vel_vec) / 100
            z_pos_n = self.z_pos + d_pos
            z_vel_n = z_vel_vec[-1]
            
        # Boundary Conditon: We have reached the ground
        if z_pos_n <= 0:
            z_pos_n = 0
            z_vel_n = 0
        
        self.z_pos = z_pos_n
        self.z_vel = z_vel_n
        self.last_update = current_t
        
        self.plot_state()
        self.t_log = np.append(self.t_log, current_t)
        self.z_pos_log = np.append(self.z_pos_log, self.z_pos)
        self.z_vel_log = np.append(self.z_vel_log, self.z_vel)
        self.z_acc_log = np.append(self.z_acc_log, self.z_acc)
        
    def analogWrite(self, sim_pin, value):
        
        # Setting the motors to a value yields a change in acceleration
        #  Use the predefined acceleration curve to change aircraft acceleration
        self.update_state()
        self.motor_speed = value
        self.z_acc = self.calc_z_acc(value)
    
    def echoRead(self, sim_pins):
        
        self.update_state()
        
        # echoRead for arduino class returns time in microseconds, convert
        #  z_pos from m to us
        time_us = self.z_pos * (1 / speed_of_sound) * 1e6 * 2
        
        return time_us

    def calc_z_acc(self, value):
        
        acc_slope = a_gravity / crit_ratio
        
        # The acceleration is determined from linear equation with parameters:
        #  a(critical_power) = 0
        #  a(0) = -9.81 m/s^2 (gravitational acceleration)
        z_acc = acc_slope * (value / (max_power - min_power)) - a_gravity
        
        return z_acc
    
    def plot_state(self):
        
        self.log_ax.plot(self.last_update - self.sim_start, self.z_pos, '.-')
    
    def close():
        
        return True
    
    