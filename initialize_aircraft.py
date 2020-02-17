# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 15:14:10 2020

@author: Sean
"""

import time
from aircraft import aircraft

simulation = 1
a_com = 'COM3'
motors = [9, 10, 11, 12]
ultrasonic_sensor = [2, 3]

ac = aircraft(a_com, motors, ultrasonic_sensor, simulation=simulation)

# Testing acceleration
ac.set_motors(ac.crit_speed + 10)
for i in range(10):
    print(ac.check_altitude())
    time.sleep(0.5)
    
# Testing drag at equilibrium
print('Starting free fall')
ac.set_motors(ac.crit_speed)
for i in range(40):
    print(ac.check_altitude())
    time.sleep(0.5)
    
# Testing deceleration
print('Starting free fall')
ac.set_motors(0)
for i in range(40):
    print(ac.check_altitude())
    time.sleep(0.5)
    
# Testing set_altitude command
ac.set_altitude(20)
for i in range(20):
    print(ac.check_altitude())
    time.sleep(0.25)
    
#ac.command_aircraft()
