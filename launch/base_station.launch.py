from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="base_station_receiver",
            executable="bs_nav_receiver",
            output="screen"
        ),
        Node(
            package="base_station_receiver",
            executable="bs_rtcm_receiver",
            output="screen"
        ),
    ])