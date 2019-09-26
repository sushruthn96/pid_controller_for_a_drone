# pid_controller_for_a_drone

* PID controller for a quadcopter to track a land based moving object.

## Summary

* key_handling.py - Has the tuned PID code. Only Pitch and Roll axes are tuned keeping the yaw axis constant. 
* key_commands.py - Commands mapped to keyboard
* data_via_rosservice.py - Communicates with the pluto drone to get the sensor values for determining its x,y,z positions.
