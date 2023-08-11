#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PoseStamped, Twist, Quaternion
from std_msgs.msg import ColorRGBA  
from visualization_msgs.msg import Marker
from vfh import VFH
from math import pi

class NodeVFH():
    _goal = None
    _destination_angle = None
    _goal_reached = False
    
    def __init__(self):
        rospy.init_node('vfh_node')
        self.frame_id   = rospy.get_param('~frame_id', '/robot0')
        
        """
        VFH params
        """
        self.threshhold = rospy.get_param('~threshhold', 2)
        self.sector_num = rospy.get_param('~sector_num', 100)
        self.wide_sectors = rospy.get_param('~wide_sectors', 5)
        self.safe_radius = rospy.get_param('~safe_radius', 0.3)
        self.d_max = rospy.get_param('~d_max', 3)
        self.smoothing = rospy.get_param('~smoothing', 9)
        self.safe_sector = rospy.get_param('~safe_sector', 2)
        
        print(self.frame_id, self.threshhold, self.sector_num, self.wide_sectors, self.safe_radius, self.d_max, self.smoothing, self.safe_sector)
        
        """
        CMD control
        """
        self.max_turn_speed                     = rospy.get_param('~max_turn_speed', 0.5)
        self.max_linear_speed                   = rospy.get_param('~max_linear_speed', 0.5)
        self.min_angle_for_turn                 = rospy.get_param('~min_angle_for_turn', 0.02)
        self.max_angle_for_full_linear_speed    = rospy.get_param('~max_angle_for_full_linear_speed', pi/4)
        self.threshold_angle_for_linear_speed   = rospy.get_param('~threshold_angle_for_linear_speed', 0.1)
        
        
        self.vfh = VFH(threshhold=self.threshhold, sector_num=self.sector_num, wide_sectors=self.wide_sectors, 
                       safe_radius=self.safe_radius, d_max=self.d_max, smoothing=self.smoothing, 
                       safe_sector=self.safe_sector)
        
        # self.local_grid__topic_name = rospy.get_param(self.frame_id + '/local_grid_topic_name', '/local_grid')
        self.markerPublisherRobotCMD    = rospy.Publisher(self.frame_id + '/cmd_vel', Twist)
        self.his_viz_pub                = rospy.Publisher(self.frame_id + '/histogram', Marker, queue_size=1)
        
        rospy.Subscriber(self.frame_id + "/local_grid", OccupancyGrid, self.local_grid_callback)
        rospy.Subscriber(self.frame_id + '/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
    
        
        
    def local_grid_callback(self, msg):
        """
        msg: OccupancyGrid
        """
        result = self.vfh.calculate(msg, self._destination_angle)
        histogram = self.vfh.get_histogram()
        self.cmd(result)
                
        self.visual(histogram, msg, result)
    
    def cmd(self, angle):
        if (angle is None) or (self._destination_distance is None) or self._goal_reached:
            return
        
        turn_speed = 0.0
        linear_speed = 0.0
        if self._destination_distance > 0.05:
            # Smooth turning logic using a sigmoid function
            turn_speed = np.sign(angle) * min(abs(angle), self.max_turn_speed)
            turn_speed *= 1 / (1 + np.exp(-10 * (abs(angle) - self.min_angle_for_turn)))

            # Dynamic linear speed control based on distance to target
            linear_speed = self.max_linear_speed if abs(angle) < self.threshold_angle_for_linear_speed else 0.0

            if abs(angle) > self.max_angle_for_full_linear_speed:
                linear_speed = 0
        else:
            self._goal_reached = True
            rospy.loginfo("Reached goal")


        # Construct the Twist message
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = turn_speed

        # Publish the Twist message
        self.markerPublisherRobotCMD.publish(twist)
    
    def goal_callback(self, msg):
        """
        msg: PoseStamped
        """
        self._goal = msg
        self._goal_reached = False
    
    def odom_callback(self, msg):
        """
        msg: Odometry
        """
        if self._goal is None:
            return
        
        x = self._goal.pose.position.x - msg.pose.pose.position.x
        y = self._goal.pose.position.y - msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        self._destination_angle =  atan2(y, x) - yaw
        self._destination_distance = np.sqrt(x ** 2 + y ** 2)
        
    def get_marker(self, id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale = Point(1, 1, 1)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)
        marker.color = ColorRGBA(0, 0, 0, 0.5)
        
        return marker
        
    def visual(self, histogram, local_grid, method_angle):
        width, height = local_grid.info.width, local_grid.info.height
        grid_resolution = local_grid.info.resolution
        width_m, height_m = width * grid_resolution, height * grid_resolution
        angular_resolution = np.deg2rad(360.0 / self.sector_num)
        
        marker = self.get_marker(0)
        for i in range(len(histogram)):
            if method_angle and (i == self.vfh.angle2sector(method_angle)):
                marker.colors.append(ColorRGBA(0, 0, 1, 1))
            elif histogram[i] == 0:
                marker.colors.append(ColorRGBA(0, 1, 0, 1))
            else:
                marker.colors.append(ColorRGBA(1, 0, 0, 1))
                        
            marker.points.append(Point(0, 0, 0))
            for o in range(2):
                alfa = (i + o) * angular_resolution
                c = np.cos(alfa)
                s = np.sin(alfa)
                
                if abs(s) < abs(c):
                    x = np.sign(c) * width_m / 2.0
                    y = (1 if (c == 0) else (s/c)) * x
                else:
                    y = np.sign(s) * height_m / 2.0
                    x = (1 if (s == 0) else (c/s)) * y
                    
                marker.points.append(Point(x, y, 0))
        self.his_viz_pub.publish(marker)
        
if __name__ == '__main__':
    NodeVFH()
    rospy.spin()