"""
User interface for sending messages to UGV
"""
from pymavlink import mavutil
import time
import os

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

while True:
    cmd = input("Enter UGV command >  ")
    
    if cmd == "waypoints":
        # waypoints are structured as follows:
        # long, lat
        # long, lat
        file_path  = input("Enter waypoint file path >")
        if(os.path.exists(file_path)):
            with open(file_path, 'r') as f:
                waypoints = f.readlines()
                for waypoint in waypoints:
                    long, lat = waypoint.split(',')
                    master.mav.debug_float_array_send(
                        time.time(),
                        b"waypoints",
                        0,
                        bytearray([float(long), float(lat)])
                    )
    if cmd == "start":
        master.mav.named_value_int_send(int(time.time()), b"nav_start", 1)
    elif cmd == "clear":
       master.mav.named_value_int_send(int(time.time()), b"clear_wps", 1)
    elif cmd == "stop":
        master.mav.named_value_int_send(int(time.time()), b"nav_stop", 1)
    elif cmd == "return":
        master.mav.named_value_int_send(int(time.time()), b"return_home", 1)
    elif cmd == "help":
        print("\nwaypoints: prompts waypoint file input")
        print("start: starts navigation to waypoints")
        print("stop: stops navigation to waypoints")
        print("return: returns to home")
        print("clear: clears waypoints")
    else:
        print("\n[ERROR] Invalid command. Enter help for available commands.\n")
        
