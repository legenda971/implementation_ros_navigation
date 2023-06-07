#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.server import Server
from test.cfg import DynamicParamConfig

class LaserToLocalGrid:

    def __init__(self):
        rospy.init_node('laser_to_local_grid')

        self.frame_id = rospy.get_param('~frame_id', '/robot0')
        self.resolution = rospy.get_param('~resolution', 0.05)  # in meters
        self.width = rospy.get_param('~width', 10)  # in meters
        self.height = rospy.get_param('~height', 10)  # in meters
        
        self.distance_error_mean = rospy.get_param('~distance_error_mean', 0)  # in meters
        self.distance_error_std = rospy.get_param('~distance_error_std', 0)  # in meters
        self.angle_error_mean = rospy.get_param('~angle_error_mean', 0)  # in meters
        self.angle_error_std = rospy.get_param('~angle_error_std', 0)  # in meters
        self.distance_dep_factor = rospy.get_param('~distance_dep_factor', 0)  # in meters
        self.distance_dep_error_std = rospy.get_param('~distance_dep_error_std', 0)  # in meters

        self.grid_pub = rospy.Publisher('~local_grid', OccupancyGrid, queue_size=1)
        self.cube_list_pub = rospy.Publisher('~cube_list_pub', MarkerArray, queue_size=1)
        rospy.Subscriber('/robot0/laser_0', LaserScan, self.scan_callback)
        
        self.reconfigure_server = Server(DynamicParamConfig, callback=self.server_callback)

    def server_callback(self, config, level):
        # copy the values from the config to instance
        for key in config.keys():
            setattr(self, key, config[key])
            
        return config

    def polar_to_cartesian(self, angle, radius):
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        return x, y
    
    def create_cube_marker(self, position, index, color=(1.0, 1.0, 0.0, 1.0)):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.1
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.position.x, marker.pose.position.y = position
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.id = index
        return marker
    
    def create_occupancy_grid(self):
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = self.frame_id
        grid.info.resolution = self.resolution
        grid.info.width = int(self.width / self.resolution)
        grid.info.height = int(self.height / self.resolution)
        grid.info.origin = Pose(Point(-self.width / 2, -self.height / 2, 0), Quaternion(0, 0, 0, 1))
        grid.data = np.zeros((grid.info.width, grid.info.height), dtype=np.int8).flatten().tolist()
        return grid
    
    def get_grid(self, ranges, angles):
        grid = self.create_occupancy_grid()
        for angle, range_value in zip(angles, ranges):
            if np.isinf(range_value):
                continue
            
            x, y = self.polar_to_cartesian(angle, range_value)
            i = int((x - grid.info.origin.position.x) / self.resolution)
            j = int((y - grid.info.origin.position.y) / self.resolution)
            
            if 0 <= i < grid.info.width and 0 <= j < grid.info.height:
                index = j *  grid.info.width + i
                grid.data[index] = 100
        return grid

    def scan_callback(self, scan):
        # Angle error
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
        angle_errors = np.random.normal(self.angle_error_mean, self.angle_error_std, len(angles))
        angles_with_error = angles + np.deg2rad(angle_errors)
        
        # Distance error
        ranges = np.array(scan.ranges)
        distance_errors = np.random.normal(self.distance_error_mean, self.distance_error_std, len(ranges))
        ranges_with_error = ranges + distance_errors
        
        # Distance dependent error
        if self.distance_dep_factor != 0:
            distance_dep_error = ranges * self.distance_dep_factor 
            ranges_with_error += np.random.normal(distance_dep_error, self.distance_dep_error_std, len(ranges))

        self.grid_pub.publish(self.get_grid(ranges_with_error, angles_with_error))

if __name__ == '__main__':
    laser_to_local_grid = LaserToLocalGrid()
    rospy.spin()
