#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2
from nav_msgs.msg import  Odometry

class PathSmoothnessMetric:
    def __init__(self):
        self.previous_yaw = None
        self.total_angular_difference = 0
        self.num_updates = 0

    def update_orientation(self, x, y, current_yaw):
        # If this is not the first yaw, calculate the difference with the previous yaw
        if self.previous_yaw is not None:
            angular_difference = abs(current_yaw - self.previous_yaw)
            # Normalize the angular difference to the range [0, pi]
            angular_difference = min(angular_difference, 2 * np.pi - angular_difference)
            self.total_angular_difference += angular_difference
            self.num_updates += 1

        self.previous_yaw = current_yaw

    def calculate_smoothness(self):
        # Smoothness is measured as the inverse of the average angular differences
        if not self.num_updates or not self.total_angular_difference:
            return 0
        # print("self.total_angular_difference / self.num_updates", self.total_angular_difference / self.num_updates)
        smoothness = 1.0 / (self.total_angular_difference / self.num_updates)
        return smoothness

    def report_smoothness(self):
        smoothness = self.calculate_smoothness()
        print('Path Smoothness:', smoothness, "Num updates:", self.num_updates, "Total angular difference:", self.total_angular_difference)
        
class NodePathSmoothnessMetric():
    def __init__(self):
        rospy.init_node('path_smoothness_metric_node')
        self.odom   = rospy.get_param('~odom', '/odom')
        rospy.Subscriber( self.odom, Odometry, self.odom_callback)
        
        self.metric = PathSmoothnessMetric()
        
    def odom_callback(self, msg):
        """
        msg: Odometry
        """        
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        
        yaw = atan2(siny_cosp, cosy_cosp)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        
        # update metrics    
        self.metric.update_orientation(x, y, yaw)
        self.metric.report_smoothness()

if __name__ == '__main__':
    laser_to_local_grid = NodePathSmoothnessMetric()
    rospy.spin()