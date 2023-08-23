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
        
        # Occupancy grid parameters
        self.resolution = rospy.get_param('~resolution', 0.05)  # in meters
        self.width = rospy.get_param('~width', 10)  # in meters
        self.height = rospy.get_param('~height', 10)  # in meters
        
        self.noise = rospy.get_param('~noise', True)
        
        # Nose parameters
        self.a = rospy.get_param('~a', -106.99)
        self.b = rospy.get_param('~b', 0.336)
        self.c = rospy.get_param('~c', -0.000256)

        self.grid_pub = rospy.Publisher(self.frame_id + '/local_grid', OccupancyGrid, queue_size=1)
        rospy.Subscriber(self.frame_id + '/laser_0', LaserScan, self.scan_callback)
        
        self.reconfigure_server = Server(DynamicParamConfig, callback=self.server_callback)
        
        self.noise_threshold = 0.25

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
        for idx, (angle, range_value) in enumerate(zip(angles, ranges)):
            # if self.noise:
            #     range_value = self.mixed_pixel_effect(ranges, idx)
            
            if np.isinf(range_value) or np.isnan(range_value):
                continue
            
            x, y = self.polar_to_cartesian(angle, range_value)
            i = int((x - grid.info.origin.position.x) / self.resolution)
            j = int((y - grid.info.origin.position.y) / self.resolution)
            
            if 0 <= i < grid.info.width and 0 <= j < grid.info.height:
                index = j *  grid.info.width + i
                grid.data[index] = 100
        return grid
    
    def mixed_pixel_effect(self, distances, i):
        beam_width = 0.1  # A parameter representing the width of the LiDAR beam
        weights = [np.exp(-0.5 * (angle / beam_width) ** 2) for angle in range(-2, 3)]
        weights /= sum(weights)  # Normalize to sum to 1

        # Convolve distances with beam effect
        neighbors = distances[max(0, i - 2):min(len(distances), i + 3)]
        mixed_distance = sum(w * d for w, d in zip(weights, neighbors))
        return mixed_distance


    def scan_callback(self, scan):
        # Extract distances
        distances = np.array(scan.ranges)

        # Compute variance for each distance
        
        variances = self.a + self.b * distances + self.c * distances**2

        # Simulate noise
        if self.noise:
            noisy_distances = []
            for distance, variance in zip(distances, variances):
                if np.isinf(distance) or np.isnan(distance):
                    noisy_distances.append(distance)
                    continue
                noise = np.random.normal(loc=0, scale=np.sqrt(np.abs(variance)))
                noisy_distances.append((distance * 1000 + noise) / 1000)  # in meters
            distances = noisy_distances
        
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment)
        
        self.grid_pub.publish(self.get_grid(distances, angles))
        
        

if __name__ == '__main__':
    laser_to_local_grid = LaserToLocalGrid()
    rospy.spin()
