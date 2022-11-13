from pymavlink import mavutil
import time

master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600)

timestamp = 0
while True:
    time.sleep(0.2)
    # master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    # send lat long waypoint
    master.mav.gps_input_send(timestamp, 1, 0, 0, 0, 2, 33, 88, 0, 1, 1, 
                                0, 0, 0, 0, 0, 0, 0)
    
    timestamp += 1
    
