"""
User interface for sending messages to UGV
"""
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

while True:
    cmd = input("Enter UGV command >  ")

    if cmd == "start":
        None
    if cmd == "help":
        print("\nwaypoints: prompts waypoint file input")
        print("start: starts navigation to waypoints")
        print("stop: stops navigation to waypoints")
        print("return: returns to home")
        print("clear: clears waypoints")
    else:
        print("\n[ERROR] Invalid command. Enter help for available commands.\n")
        
