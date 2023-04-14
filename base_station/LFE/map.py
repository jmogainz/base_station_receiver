"""
User interface for sending messages to UGV
"""

import dash
import dash_leaflet as dl
from dash import html
from dash import dcc
from dash.dependencies import Input, Output, State
import dash_auth
import webbrowser
import time
from threading import Timer
import threading
from wsgiref.simple_server import make_server

# Define valid usernames and passwords
VALID_USERNAME_PASSWORD_PAIRS = {
    'justin': 'justin',
    'jacob': 'jacob',
    'scott': 'scott',
    'patrick': 'patrick',
    'harrison' : 'harrison'
}

# Define app and server
app = dash.Dash(__name__)
server = app.server

# Create authentication object
auth = dash_auth.BasicAuth(
    app,
    VALID_USERNAME_PASSWORD_PAIRS
)

# initialize locations
waypoints = []

# create front end
app.layout = html.Div([
    # Login form
    html.Div(id='login-form', children=[
        dcc.Input(id='username', type='text', placeholder='Username'),
        dcc.Input(id='password', type='password', placeholder='Password'),
        html.Button(id='login-button', n_clicks=0, children='Login')
    ], style={'display': 'inline-block', 'margin': '20px'}),

    # Map
    dl.Map(
        [dl.TileLayer(), dl.LayerGroup(id="layer")],
        id="map",
        style={'width': '1000px', 'height': '50vh', 'margin': "auto", "display": "block"},
        center=[33.4526221, -88.7872477],
        zoom=10
    ),
])

# Callback for handling login
@app.callback(Output('login-form', 'style'), Output('map', 'style'), Output('login-button', 'children'),
              [Input('login-button', 'n_clicks')], [State('username', 'value'), State('password', 'value')])
def login(n_clicks, username, password):
    if n_clicks > 0 and username in VALID_USERNAME_PASSWORD_PAIRS and VALID_USERNAME_PASSWORD_PAIRS[username] == password:
        login_style = {'display': 'none'}
        map_style = {'width': '1000px', 'height': '99vh', 'buffer-top': '-20px', 'margin': "auto", "display": "block"}
        button_text = 'Logged in as ' + username
        return login_style, map_style, button_text
    else:
        login_style = {'display': 'inline-block', 'margin': '20px'}
        map_style = {'display': 'none'}
        button_text = 'Login'
        return login_style, map_style, button_text

# Callback for storing and displaying markers
@app.callback([Output("layer", "children"), Output("map", "center"), Output("map", "zoom")], [Input("map", "click_lat_lng")])
def map_click(click_lat_lng):
    location = (click_lat_lng)
    # appends the waypoints vector with the coordinates
    waypoints.append(list(click_lat_lng))
    markers = [dl.Marker(position=point, children=dl.Tooltip("({:.3f}, {:.3f})".format(*point))) for point in waypoints]
    if len(waypoints) > 1:
        line = dl.Polyline(positions=waypoints, color="red", weight=5, opacity=0.7)
        return markers + [line], None, None
    return markers, None, None

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
            
def openBrowser():
    webbrowser.open_new(f"http://localhost:8050/")
            
# rtcm_proc_ = Process(target=sendRTCM)
# rtcm_proc_.start()
hb_proc_ = Process(target=heartbeat)
hb_proc_.start()

while True:
    cmd = input("Enter UGV command >  ")
    
    if cmd == "waypoints":
        # clear waypoints vector
        waypoints.clear()
        
        # start web app
        Timer(2, openBrowser).start()
        dash_thread = Process(target=app.run())
        dash_thread.start()

    elif cmd == "start":
        # type_val = 0
        # while True:
        #     type = input("Enter waypoint type >  ")
        #     if type == 'map':
        #         type_val = 1
        #     elif type == 'll':
        #         type_val = 2
        #     if type_val:
        #         break
        #     else:
        #         print("Invalid waypoint type. Try again.")
        
        # print contents of waypoints vector
        print(waypoints)
        type_val = 2

        for waypoint in waypoints:
            lat, long = waypoint[0], waypoint[1]

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
        
        master.mav.named_value_int_send(int(time.time()), b"start", 1)
    elif cmd == "clear":
       master.mav.named_value_int_send(int(time.time()), b"clear", 1)
    elif cmd == "stop":
        master.mav.named_value_int_send(int(time.time()), b"stop", 1)
    elif cmd == "return":
        master.mav.named_value_int_send(int(time.time()), b"return", 1)
    elif cmd == "heading":
        master.mav.named_value_int_send(int(time.time()), b"heading", 1)
    elif cmd == "location":
        master.mav.named_value_int_send(int(time.time()), b"location", 1)
    elif cmd == "calibration":
        master.mav.named_value_int_send(int(time.time()), b"calibration", 1)
        
    elif cmd == "help":
        print("\nwaypoints: prompts waypoint file input")
        print("start: starts navigation to waypoints")
        print("stop: stops navigation to waypoints")
        print("return: returns to home")
        print("heading: returns the car's current heading")
        print("location: returns the car's current location")
        print("calibaration: returns the calibration")
    else:
        print("\n[ERROR] Invalid command. Enter help for available commands.\n")
        

