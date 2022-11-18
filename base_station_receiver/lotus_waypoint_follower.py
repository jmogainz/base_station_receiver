#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from robot_navigator import BasicNavigator


def goToWaypoints(waypoints, navigator):
    navigator.followWaypoints(waypoints)
    


if __name__ == '__main__':
    navigator = BasicNavigator()
    inspection_route = [ # simulation points
        [5.0, 0.0],
        [-5.0, -5.0],
        [-5.0, 5.0]]
    inspection_points = []
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.orientation.z = 1.0
    pose.pose.orientation.w = 0.0
    for pt in inspection_route:
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        inspection_points.append(deepcopy(pose))
    goToWaypoints(inspection_route, navigator)