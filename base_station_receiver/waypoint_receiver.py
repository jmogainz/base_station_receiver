import rclpy
from rclpy.node import Node

from follow_waypoints.lotus_waypoint_follower import goToWaypoints
from std_msgs.msg import Float64MultiArray
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
import math

class WaypointReceiver(Node):

    def __init__(self):
        super().__init__('waypoint_receiver')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'waypoints',
            self.receiver_callback,
            1)
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            1)
        self.subscription  # prevent unused variable warning

    def receiver_callback(self, waypoint_msg):
        ros_mapped_waypoints = []
        for waypoint in waypoint_msg.data:
            x, y = self.convert_to_map_coords(self.origin_lat, self.origin_long, 
                                                waypoint[0], waypoint[1])
            ros_mapped_waypoints.append([x, y])
        goToWaypoints(ros_mapped_waypoints)

    def gps_callback(self, current_gps_msg):
        self.origin_lat = current_gps_msg.latitude
        self.origin_long = current_gps_msg.longitude
            
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

    waypoint_receiver = WaypointReceiver()

    rclpy.spin(waypoint_receiver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()