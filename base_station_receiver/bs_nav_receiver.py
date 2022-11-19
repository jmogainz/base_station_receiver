#!/usr/bin/env python3

from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from copy import deepcopy
from robot_navigator import BasicNavigator, NavigationResult
import multiprocessing
from std_msgs.msg import Float64MultiArray
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import math
import serial
from navsat_conversions import LLtoUTM, UTMtoLL
import os
import sys
# import cv2
# from threading import timer

class BSNavReceiver(Node):

    def __init__(self):
        super().__init__('bs_nav_receiver')

        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 1)

        self.master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
        self.current_waypoints = []
        self.navigator = BasicNavigator()
        self.nav_running = False
        self.nav_launched = False
        self.initial_pose = PoseStamped()
        self.origin_lat = 0
        self.origin_long = 0
    
    def receiveCmds(self):
        i = 0
        while True:

            msg = self.master.recv_match(blocking=False)

            valid_msg = False
            try:
                print(f"Received message of type {msg.get_type()}")
                valid_msg = True
            except:
                pass
            
            if valid_msg:
                if msg.get_type() != 'BAD_DATA':
                    if msg.get_type() == 'DEBUG_FLOAT_ARRAY':
                        # longitude value is too large to be sent, it is split into list of digits
                        long = list(msg.data)
                        lat_msg = self.master.recv_match(type=['DEBUG_FLOAT_ARRAY'], blocking=True)
                        lat = list(lat_msg.data)

                        long_name = int(msg.name)
                        lat_name = int(lat_msg.name)

                        # get list values from 0 to long_name (length)
                        long = long[0:long_name]
                        lat = lat[0:lat_name]

                        # convert list of digits to string
                        long = ''.join(chr(int(e)) for e in long)
                        lat = ''.join(chr(int(e)) for e in lat)

                        long = float(long)
                        lat = float(lat)

                        # x, y = self.convert_to_map_coords(self.origin_lat, self.origin_long, lat, long)
                        # x, y, zone = LLtoUTM(lat, long)
                        x, y = self.convert_to_map_coords(self.origin_lat, self.origin_long, lat, long)
                        ros_pose = self.createPose(x, y, 0.0) 
                        self.current_waypoints.append(deepcopy(ros_pose))
                        self.get_logger().info("Received waypoints: %s" % len(self.current_waypoints))
                    if msg.get_type() == 'NAMED_VALUE_INT':
                        if msg.name == 'nav_start' and msg.value == 1:
                            self.get_logger().info("Received start navigation command")
                            if self.current_waypoints:
                                self.get_logger().info("All waypoints received, starting navigation")
                                if not self.nav_running:
                                    self.nav_running = True
                                    self.navigator.followWaypoints(self.current_waypoints)
                                else:
                                    self.get_logger().info("Navigation already launched")
                        if msg.name == 'nav_stop' and msg.value == 1:
                            self.get_logger().info("Stopping navigation")
                            self.navigator.cancelNav()
                        if msg.name == 'clear_wps' and msg.value == 1:
                            if not self.nav_running:
                                self.get_logger().info("Clearing waypoints")
                                self.current_waypoints.clear()
                            else:
                                self.get_logger().info("Cannot clear waypoints while navigating")
                        if msg.name == 'return_home' and msg.value == 1:
                            if not self.nav_running:
                                self.get_logger().info("Returning to home")
                                self.navigator.goToPose(self.initial_pose)
                            else:
                                self.get_logger().info("Cannot return home while navigating")
                            if msg.name == 'kill_server' and msg.value == 1:
                                self.get_logger().info("Killing server")
                                self.destroy_node()
                                sys.exit()
                    if msg.get_type() == 'GPS_RTCM_DATA':
                        self.get_logger().info("Received RTCM data")
                        # handle rtcm data in separate process so that it does not block
                        rtcm_process = multiprocessing.Process(target=self.handle_rtcm_data, args=(msg.data,))
                        rtcm_process.start()

            if not self.navigator.isNavComplete() and self.nav_running:
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(self.current_waypoints)))

            if self.nav_running and self.navigator.isNavComplete():
                result = self.navigator.getResult()
                if result == NavigationResult.SUCCEEDED:
                    print('Waypoints navigated successfully...')
                    self.nav_running = False
                    i = 0
                elif result == NavigationResult.CANCELED:
                    print('Waypoint navigation cancelled...')
                    self.nav_running = False
                    i = 0
                elif result == NavigationResult.FAILED:
                    print('Waypoint navigation failed...')
                    self.nav_running = False

    def handle_rtcm_data(rtcm_msg):
        # send rtcm_msg over uart to /dev/ttyUSB1
        uart_rtcm = serial.Serial('/dev/ttyUSB1', 38400)
        # set configuration
        uart_rtcm.bytesize = serial.EIGHTBITS
        uart_rtcm.parity = serial.PARITY_NONE
        uart_rtcm.stopbits = serial.STOPBITS_ONE
        uart_rtcm.write(rtcm_msg)
        uart_rtcm.close()

    def gps_callback(self, current_gps_msg):
        self.origin_lat = current_gps_msg.latitude
        self.origin_long = current_gps_msg.longitude
        self.initial_pose = self.createPose(0.0, 0.0, 1.0)
        self.get_logger().info("Initial pose and lat/long origin is recorded.")

        # set current gps location as datum in navsat_transform_node
        # datum_cmd = 'ros2 service call /datum robot_localization/srv/SetDatum \'{geo_pose: {position: {latitude: ' + str(current_gps_msg.latitude) + ', longitude: ' + str(current_gps_msg.longitude) + ', altitude: ' + str(current_gps_msg.altitude) + '}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\''
        # os.system(datum_cmd)

# self.navigator.setInitialPose(self.initial_pose)

    def createPose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # pose.header.frame_id = 'utm'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.z = yaw
        pose.pose.orientation.w = 0.0
        self.get_logger().info("x: %f" % x)
        self.get_logger().info("y: %f" % y)
        pose.pose.position.x = x
        pose.pose.position.y = y
        return pose

    def convert_to_map_coords(self, origin_lat, origin_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        azimuth = g['azi1']

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse

        # Convert to UTM coordinates
        return x, y

    # def capture_photo_and_send_photo(self):
        
    #     photo = cv2.VideoCapture(0)
    #     _, frame = photo.read()
    #     self.master.mav.mavlink_data_stream_type(5)
    #     self.master.mav.data_transmission_handshake()
    #     msg = self.master.recv_match(type=['DATA_TRANSMISSION_HANDSHAKE')
    #     self.master.mav.data_transmission_handshake()

    #     image_string = base64.b64decode(frame)

    #    print(image_string) 

    #     master.mav.encapsulated_data_send()

        


def main(args=None):
    rclpy.init(args=args)

    bs_nav_receiver = BSNavReceiver()

    # set the gps origin
    rclpy.spin_once(bs_nav_receiver)

    bs_nav_receiver.receiveCmds()

    # self.timer.start()
    
    # self.timer() = Timer(10,self.capture_photo_and_send_photo)



if __name__ == '__main__':
    main()
