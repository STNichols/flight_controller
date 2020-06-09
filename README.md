# flight_controller
Autonomous flight controller to plug in with arduino.

The aircraft class object builds on the arduino class defined in the description below.
Additionally, for simulated flight the aircraft_sim class object may be used instead of the physical arduino/motors/sensors
to test aircraft behavior.

For an example of the simulated aircraft performing predetermined manuevers, refer to:
https://github.com/STNichols/flight_controller/blob/master/Flight_Test.ipynb

This module imports Arduino class from Python-Arduino_Proto-API-v2
for generic serial communications via digital/analog read/writes
https://github.com/vascop/Python-Arduino-Proto-API-v2/
