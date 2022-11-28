"""
User interface for sending messages to UGV
"""
from pymavlink import mavutil
import time
from pyrtcm import RTCMReader
from serial import Serial
import os
from multiprocessing import Process

os.putenv("MAVLINK20","1")
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

def heartbeat():
    while True:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)

def sendRTCM():
    stream = Serial("/dev/ttyACM0", baudrate=9600, timeout=3)
    while True:
        reader = RTCMReader(stream)
        (raw_data, parsed_data) = reader.read()
        if parsed_data:
            raw_data = raw_data + b'\x00' * (180 - len(raw_data))
            print(parsed_data)
            master.mav.gps_rtcm_data_send(0, len(raw_data), raw_data)

rtcm_proc_ = Process(target=sendRTCM)
rtcm_proc_.start()
hb_proc_ = Process(target=heartbeat)
hb_proc_.start()

while True:
    cmd = input("Enter UGV command >  ")
    
    if cmd == "waypoints":

        type_val = 0
        while True:
            type = input("Enter waypoint type >  ")
            if type == 'map':
                type_val = 1
            elif type == 'll':
                type_val = 2
            if type_val:
                break
            else:
                print("Invalid waypoint type. Try again.")

        file_path  = input("Enter waypoint file path > ")
        if(os.path.exists(file_path)):
            with open(file_path, 'r') as f:
                waypoints = f.readlines()
                for waypoint in waypoints:
                    long, lat = waypoint.split(',')

                    # round everything to 7 decimal places
                    long = str(round(float(long), 10))
                    lat = str(round(float(lat), 10))

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
                        type_val,
                        data = bytearray(longs)
                    )
                    master.mav.debug_float_array_send(
                        int(time.time()),
                        lat_name,
                        type_val,
                        data = bytearray(lats)
                    )
                    
    elif cmd == "start":
        master.mav.named_value_int_send(int(time.time()), b"start", 1)
    elif cmd == "clear":
       master.mav.named_value_int_send(int(time.time()), b"clear", 1)
    elif cmd == "stop":
        master.mav.named_value_int_send(int(time.time()), b"stop", 1)
    elif cmd == "return":
        master.mav.named_value_int_send(int(time.time()), b"return", 1)
    elif cmd == "heading":
        master.mav.named_value_int_send(int(time.time()), b"heading", 1)
        
    elif cmd == "help":
        print("\nwaypoints: prompts waypoint file input")
        print("start: starts navigation to waypoints")
        print("stop: stops navigation to waypoints")
        print("return: returns to home")
        print("clear: clears waypoints")
    else:
        print("\n[ERROR] Invalid command. Enter help for available commands.\n")
        

