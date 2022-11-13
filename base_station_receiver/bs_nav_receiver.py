from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from copy import deepcopy
from robot_navigator import BasicNavigator, NavigationResult
from lotus_waypoint_follower import goToWaypoints
from std_msgs.msg import Float64MultiArray
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import math

class BSNavReceiver(Node):

    def __init__(self):
        super().__init__('bs__nav_receiver')

        self.waypoints_pub = self.create_publisher(Float64MultiArray, 'waypoints', 1)
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
        while True:
        # master.wait_heartbeat()
        # print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_system))

            msg = self.master.recv_match(blocking=True)
            
            if msg.get_type() != 'BAD_DATA':
                if msg.get_type() == 'DEBUG_FLOAT_ARRAY':
                    x, y = self.convert_to_map_coords(self.origin_lat, self.origin_long, msg.data[1], msg.data[0]) # data is sent as long, lat
                    ros_pose = self.createPose(x, y) 
                    self.current_wayoints.append(deepcopy(ros_pose))
                    rclpy.loginfo(self.get_logger(), "Received waypoints: %s" % len(self.current_waypoints))
                if msg.get_type() == 'NAMED_VALUE_INT':
                    if msg.name == 'nav_start' and msg.value == 1:
                        rclpy.loginfo(self.get_logger(), "All waypoints received, starting navigation")
                        if not self.nav_running:
                            self.nav_running = True
                            goToWaypoints(self.current_waypoints, self.navigator)
                        else:
                            rclpy.loginfo(self.get_logger(), "Navigation already launched")
                    if msg.name == 'nav_stop' and msg.value == 1:
                        rclpy.loginfo(self.get_logger(), "Stopping navigation")
                        self.navigator.cancelNav()
                    if msg.name == 'clear_wps' and msg.value == 1:
                        if not self.nav_running:
                            rclpy.loginfo(self.get_logger(), "Clearing waypoints")
                            self.current_waypoints.clear()
                        else:
                            rclpy.loginfo(self.get_logger(), "Cannot clear waypoints while navigating")
            
            if self.nav_running and self.navigator.isNavComplete():
                result = self.navigator.getResult()
                if result == NavigationResult.SUCCEEDED:
                    print('Waypoints navigated successfully...')
                    self.nav_running = False
                elif result == NavigationResult.CANCELED:
                    print('Waypoint navigation cancelled...')
                    self.nav_running = False
                elif result == NavigationResult.FAILED:
                    print('Waypoint navigation failed...')
                    self.nav_running = False

    def gps_callback(self, current_gps_msg):
        self.origin_lat = current_gps_msg.latitude
        self.origin_long = current_gps_msg.longitude
        self.createPose(0, 0)
        self.navigator.setInitialPose(0, 0)

    def createPose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.z = 1.0
        pose.pose.orientation.w = 0.0
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

        return x, y

def main(args=None):
    rclpy.init(args=args)

    bs_nav_receiver = BSNavReceiver()

    # set the gps origin
    rclpy.spin_once(bs_nav_receiver)

    bs_nav_receiver.receiveCmds()


if __name__ == '__main__':
    main()