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
       file_path  = input("Type in a file path")
        if(os.path.exists(file_path)):
            file = open(file_path,"read")
 
    if cmd == "start":
        master.mav.named_value_int_send(time.time(),"nav_start",1)
        
    if cmd == "clear":
       master.mav.named_value_int_send(time.time(),"clear_wps",1)

    if cmd == "stop"
        master.mav.named_value_int_send(time.time(),"nav_stop",1)
    
    if cmd == "return":
        master.mav.named_value_int_send(time.time(),"return_home",1)
    
    if cmd == "help":
        print("\nwaypoints: prompts waypoint file input")
        print("start: starts navigation to waypoints")
        print("stop: stops navigation to waypoints")
        print("return: returns to home")
        print("clear: clears waypoints")
    else:
        print("\n[ERROR] Invalid command. Enter help for available commands.\n")
        
