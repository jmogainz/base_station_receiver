"""
User interface for sending messages to UGV
"""
from pymavlink import mavutil
import time
import os

os.environ['MAVLINK20'] = 1
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

while True:
    cmd = input("Enter UGV command >  ")
    
    if cmd == "waypoints":
        # long, lat
        file_path  = input("Enter waypoint file path > ")
        if(os.path.exists(file_path)):
            with open(file_path, 'r') as f:
                waypoints = f.readlines()
                wp_count = 1
                for waypoint in waypoints:
                    long, lat = waypoint.split(',')

                    # round everything to 7 decimal places
                    long = str(round(float(long), 7))
                    lat = str(round(float(lat), 7))

                    # convert string to list of ascii values
                    longs = [ord(c) for c in long]
                    lats = [ord(c) for c in lat]

                    long_name = str(len(longs))
                    lat_name = str(len(lats))

                    long_name = long_name.encode('utf-8')
                    lat_name = lat_name.encode('utf-8')

                    longs.extend([0] * (58 - len(longs)))
                    lats.extend([0] * (58 - len(lats)))

                    # send sequence is long, lat
                    master.mav.debug_float_array_send(
                        int(time.time()),
                        long_name,
                        wp_count,
                        data = bytearray(longs)
                    )
                    master.mav.debug_float_array_send(
                        int(time.time()),
                        lat_name,
                        wp_count,
                        data = bytearray(lats)
                    )

                    wp_count += 1
                    
    elif cmd == "start":
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
        
