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
                wp_count = 1
                for waypoint in waypoints:
                    long, lat = waypoint.split(',')
                    # round everything to 7 decimal places
                    long = str(round(float(long), 7))
                    lat = str(round(float(lat), 7))
                    long = long.replace('.', '')
                    lat = lat.replace('.', '')
                    longs = [int(long[i]) for i in range(0, len(long))]
                    lats = [int(lat[i]) for i in range(0, len(lat))]
                    master.mav.debug_float_array_send(
                        int(time.time()),
                        b"long",
                        wp_count,
                        bytearray(longs)
                    )
                    master.mav.debug_float_array_send(
                        int(time.time()),
                        b"lat",
                        wp_count,
                        bytearray(lats)
                    )
                    wp_count += 1
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
        
