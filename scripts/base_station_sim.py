"""
Simulate MAVLink base station over local UDP port.
"""

import os
import socket
import pickle
import readline

host = 'localhost'
send_port = 14551

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class Msg:
    def __init__(self, name='', data='', type='', value=0):
        self.name = name
        self.data = data
        self.type = type
        self.value = value

    def get_type(self):
        return self.type

while True:
    cmd = input("Enter UGV command >  ")

    if cmd == "waypoints":
        file_path  = input("Enter waypoint file path > ")
        if(os.path.exists(file_path)):
            with open(file_path, 'r') as f:
                waypoints = f.readlines()
                wp_count = 1
                for waypoint in waypoints:
                    long, lat = waypoint.split(',')

                    # round everything to 7 decimal places
                    long = str(round(float(long), 9))
                    lat = str(round(float(lat), 9))

                    # convert string to list of ascii values
                    longs = [ord(c) for c in long]
                    lats = [ord(c) for c in lat]

                    long_name = str(len(longs))
                    lat_name = str(len(lats))

                    long_name = long_name.encode('utf-8')
                    lat_name = lat_name.encode('utf-8')

                    longs.extend([0] * (58 - len(longs)))
                    lats.extend([0] * (58 - len(lats)))

                    msg = Msg(name=long_name, data=longs, type='DEBUG_FLOAT_ARRAY', value=1)
                    client.sendto(pickle.dumps(msg), (host, send_port))

                    msg = Msg(name=lat_name, data=lats, type='DEBUG_FLOAT_ARRAY', value=1)
                    client.sendto(pickle.dumps(msg), (host, send_port))

                    wp_count += 1
                    
    elif cmd == "start":
        msg = Msg(name="nav_start", data="", type='NAMED_VALUE_INT', value=1)
        client.sendto(pickle.dumps(msg), (host, send_port))
    elif cmd == "clear":
        msg = Msg(name="clear_wps", data=b"", type='NAMED_VALUE_INT', value=1)
        client.sendto(pickle.dumps(msg), (host, send_port))
    elif cmd == "stop":
        msg = Msg(name="nav_stop", data=b"", type='NAMED_VALUE_INT', value=1)
        client.sendto(pickle.dumps(msg), (host, send_port))
    elif cmd == "return":
        msg = Msg(name="return_home", data=b"", type='NAMED_VALUE_INT', value=1)
        client.sendto(pickle.dumps(msg), (host, send_port))
    elif cmd == "kill_receiver":
        msg = Msg(name="kill_server", data=b"", type='NAMED_VALUE_INT', value=1)
        client.sendto(pickle.dumps(msg), (host, send_port))
    elif cmd == "heading":
        msg = Msg(name="heading", data=b"", type='NAMED_VALUE_INT', value=1)
        client.sendto(pickle.dumps(msg), (host, send_port))
    elif cmd == "help":
        print("\nwaypoints: prompts waypoint file input")
        print("start: starts navigation to waypoints")
        print("stop: stops navigation to waypoints")
        print("return: returns to home")
        print("clear: clears waypoints")
    else:
        print("\n[ERROR] Invalid command. Enter help for available commands.\n")
