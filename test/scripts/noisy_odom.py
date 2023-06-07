#!/usr/bin/env python
import rospy
import tf
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.server import Server
from test.cfg import DynamicParamConfig
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy

class NoisyOdom:
    def __init__(self):
        rospy.init_node('noisy_odom')
        self.sdt_position = rospy.get_param('~sdt_position', 0.5)
        self.sdt_orientation = rospy.get_param('~sdt_orientation', 0.5)
        self.odom_topic_name = rospy.get_param('~odom_topic_name', '/odom')
        self.noise_odom_topic_name = rospy.get_param('~noise_odom_topic_name', '/noise_odom')
        
        self.a1 = rospy.get_param('~a1', 0.05)
        self.a2 = rospy.get_param('~a2', 15.0*3.14/180.0)
        self.a3 = rospy.get_param('~a3', 0.05)
        self.a4 = rospy.get_param('~a4', 0.01)
        
        self.last_odom = None
        
        rospy.Subscriber(self.odom_topic_name, Odometry, self.callback)
        self.noise_odom_pub = rospy.Publisher(self.noise_odom_topic_name, Odometry, queue_size=1)
        
    def get_position(self, odo):
        x, y = odo.pose.pose.position.x, odo.pose.pose.position.y
        theta = euler_from_quaternion([odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, 
                                       odo.pose.pose.orientation.z, odo.pose.pose.orientation.w])[2]
        return x, y, theta
    
    def get_odometry(self, x, y, theta):
        odo = Odometry()
        odo.header.frame_id = self.noise_odom_topic_name
        odo.header.stamp = rospy.Time.now()
        
        odo.pose.pose.position.x = x
        odo.pose.pose.position.y = y
        odo.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, theta))
        return odo
        

    def callback(self, msg):
        x, y, theta = self.get_position(msg)
        rospy.loginfo("=================================")
        rospy.loginfo("x {}, y {}, theta {}".format(x, y, theta))
        if self.last_odom is None:
            self.last_odom = deepcopy(msg)
            return
        
        x2, y2, theta2 = self.get_position(self.last_odom)
        rospy.loginfo("x2 {}, y2 {}, theta2 {}".format(x2, y2, theta2))
        
        dx, dy, dtheta = x - x2, y - y2, theta - theta2
        trans = np.sqrt(dx**2 + dy**2)
        rot2 = np.arctan2(dy, dx) - theta2
        rot = dtheta - rot2
        rospy.loginfo("dx {}, dy {}, dtheta {}".format(dx, dy, dtheta))
        
        sd_rot = self.a1 * np.abs(rot) + self.a2 * trans
        sd_rot2 = self.a1 * np.abs(rot2) + self.a2 * trans
        sd_trans = self.a3*trans + self.a4*(abs(rot) + abs(rot2))
        rospy.loginfo("sd_rot {}, sd_rot2 {}, sd_trans {}".format(sd_rot, sd_rot2, sd_trans))

        
        trans +=  np.random.normal(0, sd_trans**2)
        rot += np.random.normal(0, sd_rot**2)
        rot2 += np.random.normal(0, sd_rot2**2)
        rospy.loginfo("trans {}, rot {}, rot2 {}".format(trans, rot, rot2))
        
        new_x = x2 + trans * np.cos(theta2 + rot2)
        new_y = y2 + trans * np.sin(theta2 + rot2)
        new_theta = rot2 + rot
        rospy.loginfo("x {}, y {}, theta {}".format(new_x, new_y, new_theta))
        
        self.last_odom = deepcopy(msg)
        br = tf.TransformBroadcaster()
        br.sendTransform((new_x, new_y, 0),
                         quaternion_from_euler(0, 0, new_theta),
                         msg.header.stamp,
                         "/map",
                         self.noise_odom_topic_name)
        
        self.noise_odom_pub.publish(self.get_odometry(new_x, new_y, new_theta))


if __name__ == '__main__':
    laser_to_local_grid = NoisyOdom()
    rospy.spin()