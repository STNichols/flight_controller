# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 00:15:02 2020

@author: Sean Nichols

This is the module for class object aircraft, a class that serves as a flight
controller, utilizing the Arduino class described below

The aircraft_sim class object can also be used to simulate the arduino that is
controlled by this class

This module imports Arduino class from Python-Arduino_Proto-API-v2
for generic serial communications via digital/analog read/writes
https://github.com/vascop/Python-Arduino-Proto-API-v2/
"""

import time
import numpy as np
from inspect import signature

from arduino.arduino import Arduino
from aircraft_sim import aircraft_sim
from voice_control import audio_utils
import sensor_utils

min_output = 0
max_output = 255

class aircraft(object):
    
    def __init__(self, serial_port, motor_pins, ult_sensor, simulation=0):
        
        if not simulation:
            self.arduino = Arduino(serial_port)
            self.arduino.output(motor_pins + ult_sensor)
            self.simulation = 0
        else: # Test control logic with a simulated controller
            self.arduino = aircraft_sim()
            self.simulation = 1
        self.motors = motor_pins
        self.ult_sensor = ult_sensor
        
        self.base_altitude = self.check_altitude()
        self.crit_speed = self.find_crit_speed()
        
        # Define commands for aircraft
        command_dict = {'set altitude':self.set_altitude}
        self.command_dict = command_dict
     
    def check_altitude(self):

         return sensor_utils.get_distance(self.arduino, self.ult_sensor)
     
    def set_motors(self, speed):
        
        for motor in self.motors:
            self.arduino.analogWrite(motor, speed)
            
    def set_altitude(self, alt):
        '''
        Current method slingshots the aircraft towards the desired altitude;
        this needs to be optimized to correct acceleration as the altitude is
        approached
        
        Potential Solution:
            Record time and altitude for duration of adjustment, find velocity 
            at time altitude is reached and reverse acceleration according to 
            kinematics to yield a v_f of 0 after time t
             (can either minimize leveling time or power depleted)
        '''
        current_alt = self.check_altitude()
        altitude_gap = np.abs(current_alt - alt)
        init_acc = 0.05 # max percent diff from equilibrium
        
        if current_alt > alt:
            
            self.set_motors((1 - init_acc) *self.crit_speed)
            while current_alt > alt:
                
                # Update altitude
                current_alt = self.check_altitude()
                
                # Tune the motor speed to smooth approach
                new_acc = init_acc * (np.abs(current_alt - alt) / altitude_gap)
                self.set_motors((1 - new_acc) *self.crit_speed)
                
                time.sleep(0.1)
                print(current_alt)
        elif current_alt < alt:
            
            self.set_motors((1 + init_acc) *self.crit_speed)
            while current_alt < alt:
                
                # Update altitude
                current_alt = self.check_altitude()
                
                # Tune the motor speed to smooth approach
                new_acc = init_acc * (np.abs(current_alt - alt) / altitude_gap)
                self.set_motors((1 + new_acc) *self.crit_speed)
                
                time.sleep(0.1)
                print(current_alt)
        else:
            pass
        self.set_motors(self.crit_speed)
        
        print('Current Altitude is: {}'.format(current_alt))
        
    def find_crit_speed(self):
        '''
        In a real world scenario, this will never result in the perfect motor 
            speed that results in equilibrium. Better option is to characterize
            the motor to find an empirical acc vs. rpm curve, feed in on
            initialization
        '''
        
        print('Calibrating motors to find power at equilibrium')
        for ms in range(min_output, max_output):
            self.set_motors(ms)
            time.sleep(0.1)
            if self.check_altitude() > self.base_altitude:
                print('Critial Power at: {}%'.format(
                        str(round(ms / max_output * 100, 2))))
                
                # For simulation purposes, use the exact value
                if self.simulation:
                    ms = self.arduino.sim_crit_power
                    print('Using simulation critical power at:{}%'.format(
                            str(round(ms / max_output * 100, 2))))
                return ms
                break
            
    def command_aircraft(self):
        
        com = audio_utils.listen()
        
        # Search through the command dictionary to find appropriate function
        com_lib = list(self.command_dict.keys())
        closest_com = audio_utils.match_to_lib(com, com_lib)
        com_sig = signature(self.command_dict[closest_com])
        n_args = len(str(com_sig).split(','))
        
        command_args = audio_utils.collect_sounds(n_coms=n_args)
        command_args = [float(arg) for arg in command_args if arg.isdigit()]
        if len(command_args) == n_args:
            self.command_dict[closest_com](*command_args)
        else:
            print('Missing required arguments have been specified for: ' +
                  '"{}"'.format(closest_com))
        
    def shutdown(self):
        
        self.arduino.close()
        
        
        