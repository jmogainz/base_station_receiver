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
from diagnostic_msgs.msg import DiagnosticStatus
import math
import serial
from navsat_conversions import LLtoUTM, UTMtoLL
import os
import sys
import numpy as np
import time
from pyrtcm import RTCMReader

class BSNavReceiver(Node):

    def __init__(self):
        super().__init__('bs_nav_receiver')

        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 1)
        self.imu_calib_sub = self.create_subscription(DiagnosticStatus, '/imu/diagnostics', self.imu_calib_callback, 1)

        self.master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
        self.current_waypoints = []
        self.navigator = BasicNavigator()
        self.nav_running = False

        self.initial_pose = PoseStamped()
        self.origin_lat = 0
        self.origin_long = 0
        self.current_lat = 0
        self.current_long = 0
        
        self.queue = multiprocessing.Queue()
        
        self.imu_sys_cal = 0
        self.imu_gyro_cal = 0
        self.imu_accel_cal = 0
        self.imu_mag_cal = 0
    
    def receiveCmds(self):
        rtcm_process = multiprocessing.Process(target=self.handle_rtcm_data, args=(self.queue,))
        rtcm_process.start()

        i = 0
        heartbeat_timer_start = time.perf_counter()
        comms_live = True

        while True:
            heartbeat_timer_end = time.perf_counter()
            if (heartbeat_timer_end - heartbeat_timer_start > 60) and comms_live:
                self.get_logger().info("Heartbeat not received in 60 seconds, exiting.")
                self.navigator.cancelNav()
                self.get_logger().info("Returning to home")
                home_pose = self.createPose(0.0, 0.0, True) # create pose from current position
                self.navigator.goToPose(home_pose)
                comms_live = False

            msg = self.master.recv_match(blocking=False)

            try:
                print(f"Received message of type {msg.get_type()}")
            except:
                continue

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
                    if msg.array_id == 2:
                        x, y = self.convert_to_map_coords(self.origin_lat, self.origin_long, lat, long)
                    else:
                        x = long # meters
                        y = lat
                    
                    ros_pose = self.createPose(x, y) 
                    self.current_waypoints.append(deepcopy(ros_pose))
                    self.get_logger().info("Received waypoints: %s" % len(self.current_waypoints))
                if msg.get_type() == 'NAMED_VALUE_INT':
                    # print(msg.name)
                    if msg.name == 'start' and msg.value == 1:
                        self.get_logger().info("Received start navigation command")
                        if self.current_waypoints:
                            self.get_logger().info("All waypoints received, starting navigation")
                            if not self.nav_running:
                                self.nav_running = True
                                self.navigator.followWaypoints(self.current_waypoints)
                            else:
                                self.get_logger().info("Navigation already launched")
                    if msg.name == 'stop' and msg.value == 1:
                        self.get_logger().info("Stopping navigation")
                        self.navigator.cancelNav()
                    if msg.name == 'clear' and msg.value == 1:
                        if not self.nav_running:
                            self.get_logger().info("Clearing waypoints")
                            self.current_waypoints.clear()
                        else:
                            self.get_logger().info("Cannot clear waypoints while navigating")
                    if msg.name == 'heading' and msg.value == 1:
                        rclpy.spin_once(self.navigator)
                        qx = self.navigator.current_pose.orientation.x
                        qy = self.navigator.current_pose.orientation.y
                        qz = self.navigator.current_pose.orientation.z
                        qw = self.navigator.current_pose.orientation.w
                        roll, pitch, yaw = self.get_euler_from_quaternion(qx, qy, qz, qw)
                        self.get_logger().info("Current heading: %s" % yaw)
                        self.master.mav.named_value_int_send(int(time.time()), b"heading", yaw)
                    if msg.name == "location" and msg.value == 1:
                        rclpy.spin_once(self)
                        self.master.mav.named_value_int_send(int(time.time()), b"lat", self.current_lat)
                        self.master.mav.named_value_int_send(int(time.time()), b"long", self.current_long)
                    if msg.name == "cal" and msg.value == 1:
                        rclpy.spin_once(self)
                        print("Sending calibration values")
                        self.master.mav.named_value_int_send(int(time.time()), b"imu_sys", self.imu_sys_cal)
                        self.master.mav.named_value_int_send(int(time.time()), b"imu_gyro", self.imu_gyro_cal)
                        self.master.mav.named_value_int_send(int(time.time()), b"imu_accel", self.imu_accel_cal)
                        self.master.mav.named_value_int_send(int(time.time()), b"imu_mag", self.imu_mag_cal)
                    if msg.name == 'return' and msg.value == 1:
                        if not self.nav_running:
                            self.get_logger().info("Returning to home")
                            home_pose = self.createPose(0.0, 0.0, True) # create pose from current position
                            self.navigator.goToPose(home_pose)
                        else:
                            self.get_logger().info("Cannot return home while navigating")
                if msg.get_type() == 'HEARTBEAT':
                    heartbeat_timer_start = time.perf_counter()
                    comms_live = True
                    # print("Heartbeat received")
                if msg.get_type() == 'GPS_RTCM_DATA':
                    # handle rtcm data in separate process so that it does not block
                    self.queue.put(msg)

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

    def handle_rtcm_data(self, queue):
        uart_rtcm = serial.Serial('/dev/ttyTHS1', 38400)
        uart_rtcm.bytesize = serial.EIGHTBITS
        uart_rtcm.parity = serial.PARITY_NONE
        uart_rtcm.stopbits = serial.STOPBITS_ONE
        time.sleep(1) # wait for serial port to open
        while True:
            rtcm_msg = queue.get()
            rtcm_data = rtcm_msg.data
            rtcm_raw = bytes(rtcm_data[0:rtcm_msg.len])
            parsed = RTCMReader.parse(rtcm_raw)
            # print(parsed)
            # print(len(rtcm_raw), "\n")
            uart_rtcm.write(rtcm_raw)

    def gps_callback(self, current_gps_msg):
        # set origin if not set
        if not self.origin_lat and not self.origin_long:
            self.origin_lat = current_gps_msg.latitude
            self.origin_long = current_gps_msg.longitude
            self.get_logger().info("Initial lat/long origin is recorded.")
        else:
            self.current_lat = current_gps_msg.latitude
            self.current_long = current_gps_msg.longitude

        # set current gps location as datum in navsat_transform_node
        # datum_cmd = 'ros2 service call /datum robot_localization/srv/SetDatum \'{geo_pose: {position: {latitude: ' + str(current_gps_msg.latitude) + ', longitude: ' + str(current_gps_msg.longitude) + ', altitude: ' + str(current_gps_msg.altitude) + '}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}\''
        # os.system(datum_cmd)

        # self.navigator.setInitialPose(self.initial_pose)
        
    def imu_calib_callback(self, imu_diagnostics_msg):
        self.imu_sys_cal = imu_diagnostics_msg.values[1].value
        self.imu_gyro_cal = imu_diagnostics_msg.values[2].value
        self.imu_accel_cal = imu_diagnostics_msg.values[3].value
        self.imu_mag_cal = imu_diagnostics_msg.values[4].value

    def createPose(self, x, y, from_current=False):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # pose.header.frame_id = 'utm'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # absolute position in map frame
        self.get_logger().info("x: %f" % x)
        self.get_logger().info("y: %f" % y)
        pose.pose.position.x = x
        pose.pose.position.y = y 

        # loads in current pose
        rclpy.spin_once(self.navigator)

        # orientation needs to be calculated for proper y, a x axis (east is x, north is y)
        y = -y # reverse our previous transformation
        temp_x = x
        x = y
        y = temp_x

        # z orientation should be facing away from most recent waypoint
        if from_current:
            dx = x - self.navigator.current_pose.position.x
            dy = y - self.navigator.current_pose.position.y
            yaw = math.atan2(dy, dx)
            self.get_logger().info("Orientation from current location: %s" % yaw)
        else:
            if self.current_waypoints:
                last_waypoint = self.current_waypoints[-1]
                dx = x - last_waypoint.pose.position.x
                dy = y - last_waypoint.pose.position.y
                yaw = math.atan2(dy, dx)
            else:
                dx = x - self.navigator.current_pose.position.x
                dy = y - self.navigator.current_pose.position.y
                yaw = math.atan2(dy, dx)
            self.get_logger().info("Orientation from previous waypoint: %s" % yaw)

        qx, qy, qz, qw = self.get_quaternion_from_euler(0, 0, yaw) # x and y are 0 because we are only rotating around z axis
        pose.pose.orientation.z = z = qz
        pose.pose.orientation.w = w = qw

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

        # map frame is flipped in y direction, negate y
        y = -y
        # Convert to UTM coordinates
        return x, y

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return qx, qy, qz, qw
    
    def get_euler_from_quaternion(self, x, y, z, w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

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
