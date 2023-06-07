#!/usr/bin/env python
import rospy
import tf
import numpy as np
from math import atan2, pi
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Twist
from std_msgs.msg import ColorRGBA  
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.server import Server
from test.cfg import DynamicParamConfig

class VFH:
    goal_ = None
    destination_angle_ = None
    wide_sectors = 10
    destination_sector = None
    
    def __init__(self):
        rospy.init_node('vfh')
        self.frame_id = rospy.get_param('~frame_id', '/robot0')
        self.local_grid__topic_name = rospy.get_param('~local_grid_topic_name', '/local_grid')
        self.markerPublisherRobotCMD = rospy.Publisher(self.frame_id + '/cmd_vel', Twist)
        self.threshhold = rospy.get_param('~threshhold', 0)
        
        self.sector_num = rospy.get_param('~sector_num', 100)

        rospy.Subscriber(self.local_grid__topic_name, OccupancyGrid, self.local_grid_callback)
        self.his_viz_pub = rospy.Publisher('~histogram', Marker, queue_size=1)
        
        rospy.Subscriber( self.frame_id + '/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        
        self.reconfigure_server = Server(DynamicParamConfig, callback=self.server_callback)
        
    def cmd(self, sector):
        if sector is None:
            return

        # calculate desired angle based on sector
        angle = sector * 2.0 * pi / self.sector_num

        # calculate angular difference between current orientation and desired angle
        angle_diff = angle - self.yaw
        if angle_diff > pi:
            angle_diff -= 2.0 * pi
        elif angle_diff < -pi:
            angle_diff += 2.0 * pi

        # scale turning speed based on angle difference
        turn_speed = max(-1.0, min(1.0, angle_diff))
        print(turn_speed, angle_diff, angle, self.yaw, sector)
        # Create Twist message
        twist = Twist()
        # If angle difference is small enough, move forward. Otherwise turn in place.
        temp =  0.5 if abs(angle_diff) < 0.1 else 0.0
        # print("twist.linear.x", temp)
        twist.linear.x = temp
        # Turn towards the desired angle
        twist.angular.z = turn_speed

        self.markerPublisherRobotCMD.publish(twist)
    
    
    def select_valley(self, valleys):
        if self.destination_angle_ is None:
            return None
        
        angle = self.destination_angle_
        if angle < 0:
            angle += 2 * pi
        destination_sector = int(angle // (2*pi / self.sector_num))
        self.destination_sector = destination_sector
        
        # if destination sector is in wide valley, then return it    
        wide_valleys = filter(lambda x: x[3] >= self.wide_sectors, valleys)
        # print("Destination sector: ", destination_sector)
        # print("Number of wide valleys: ", len(wide_valleys), wide_valleys)
        for valley in wide_valleys:
            if self.in_valley(destination_sector, valley):
                return destination_sector
        
        # print("Vallies: ", valleys)
        candidate_sectors = self.get_candidate_sectors(valleys)
        # print("Candidate sectors: ", candidate_sectors)
        # closest sector to destination sector
        distance = {sector: abs(sector - destination_sector) for sector in candidate_sectors}
        closest_sector = min(distance, key=distance.get)
        # print("Closest sector: ", closest_sector)
        return closest_sector

    def in_valley(self, sector, valley):
        l, r = valley[0], valley[1]
        if r >= l:
            return l <= sector and sector <= r
        else:
            return (l > sector and sector <= r) or (l <= sector and sector > r)

    def get_candidate_sectors(self, valleys):
        candidate_sectors = []
        if len(valleys) == 0:
            return candidate_sectors
        
        for valley in valleys:
            if valley[3] >= self.wide_sectors:
                r, l = valley[1], valley[0]
                candidate_sectors.append((valley[0] + 1) % self.sector_num)
                candidate_sectors.append((valley[1] - 1) % self.sector_num)
            else:
                candidate_sectors.append(int((valley[0] + valley[1]) // 2))
            
        return candidate_sectors
                
    def goal_callback(self, msg):
        self.goal_ = msg
        
    def odom_callback(self, msg):
        if self.goal_ is None:
            return
        
        x = self.goal_.pose.position.x - msg.pose.pose.position.x
        y = self.goal_.pose.position.y - msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        
        self.destination_angle_ =  atan2(y, x)
        self.yaw = yaw 
        
    def local_grid_callback(self, msg):
        self.local_grid_ = msg
        self.calculate_histogram()
        valleys = self.calculate_valleys(self.histogram_, self.threshhold)
        sector = self.select_valley(valleys)
        self.cmd(sector)
        self.vizual_histogram(sector)
        
    def server_callback(self, config, level):
        # copy the values from the config to instance
        for key in config.keys():
            print(key, config[key])
            setattr(self, key, config[key])
            
        return config

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
        
    def vizual_histogram(self, sector):
        width, height = self.local_grid_.info.width, self.local_grid_.info.height
        grid_resolution = self.local_grid_.info.resolution
        width_m, height_m = width * grid_resolution, height * grid_resolution
        angular_resolution = np.deg2rad(360.0 / self.sector_num)
        
        marker = self.get_marker(0)
        for i in range(len(self.histogram_)):
            if i == sector:
                marker.colors.append(ColorRGBA(0, 0, 1, 1))
            elif self.histogram_[i] <= self.threshhold:
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
            

    def calculate_histogram(self, smoothing=3):
        width, height = self.local_grid_.info.width, self.local_grid_.info.height
        grid_resolution = self.local_grid_.info.resolution
        angular_resolution = 360.0 / self.sector_num
        
        histogram = np.zeros(self.sector_num, dtype=np.float32)

        for i in range(width):
            for j in range(height):
                index = i + j * width
                if self.local_grid_.data[index] == 0:
                    continue
                                
                x = i * grid_resolution - width * grid_resolution / 2.0
                y = j * grid_resolution - height * grid_resolution / 2.0

                angle = np.rad2deg(np.arctan2(y, x)) % 360
                
                bin_index = int(angle // angular_resolution)
                distance = np.sqrt(x ** 2.0 + y ** 2.0)
                magnitude = 1 / distance

                histogram[bin_index] += magnitude

        self.histogram_ = np.convolve(histogram, np.ones(smoothing)/smoothing, mode='same')
        
    def calculate_valleys(self, histogram, threshold):
        valleys = []
        n = len(histogram)
        i = 0
        while i < n:
            idx_l = i % n
            if histogram[idx_l] <= threshold:
                idx_r = (i + 1) % n
                total_cost = histogram[idx_l]
                while histogram[idx_r] <= threshold:
                    total_cost += histogram[idx_r]
                    idx_r = (idx_r + 1) % n
                    i += 1
                size = (idx_r - idx_l) % n
                valleys.append((idx_l, idx_r, total_cost, size))
            i += 1
                
        return valleys


if __name__ == '__main__':
    laser_to_local_grid = VFH()
    rospy.spin()