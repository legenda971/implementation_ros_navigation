#!/usr/bin/env python
import rospy
import numpy as np
from math import atan2
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Point

class TrajectoryTrack():
    previous_pose_position = None
    
    def __init__(self):
        rospy.init_node('trajectory_track_node')
        self.odom   = rospy.get_param('~odom', '/odom')
        self.frame_id = rospy.get_param('frame_id', '/map')
        self.threshold = rospy.get_param('threshold', 0.005)
        
        self.trajectory_path_pub = rospy.Publisher('~trajectory_path', Path, queue_size=10, latch=True)
        self.odom_sub = rospy.Subscriber(self.odom, Odometry, self.odom_callback)
        
        self.trajectory_path_msg = Path()
    
    
    def publish_trajectory_path(self, position):
        if (self.previous_pose_position is not None) and (self.has_position_changed(position)):
            self.add_pose_to_trajectory_path(position)
            self.trajectory_path_pub.publish(self.trajectory_path_msg)
        self.previous_pose_position = position


    def has_position_changed(self, position):
        """
        Checks if the position has changed more than a set threshold.

        :param position: Position object containing x, y, z coordinates.
        :return: True if the position has changed, False otherwise.
        """
        return (
            abs(self.previous_pose_position.x - position.x) > self.threshold or
            abs(self.previous_pose_position.y - position.y) > self.threshold or
            abs(self.previous_pose_position.z - position.z) > self.threshold
        )


    def create_pose_stamped_msg(self, position):
        """
        Creates a PoseStamped message from the given position.

        :param position: Position object containing x, y, z coordinates.
        :return: PoseStamped message.
        """
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.pose.position = position
        pose_stamped_msg.pose.orientation.w = 1.0
        return pose_stamped_msg


    def add_pose_to_trajectory_path(self, position):
        """
        Adds the given position to the trajectory path message.

        :param position: Position object containing x, y, z coordinates.
        """
        self.trajectory_path_msg.header.stamp = rospy.Time.now()
        self.trajectory_path_msg.header.frame_id = self.frame_id
        pose_stamped_msg = self.create_pose_stamped_msg(position)
        
        self.trajectory_path_msg.poses.append(pose_stamped_msg)

        # todo: for long distance trajectories
        # if len(self.trajectory_path_msg.poses) < self.max_poses:
        #     self.trajectory_path_msg.poses.append(pose_stamped_msg)
        # else:
        #     rospy.logdebug('Max number of poses reached, erasing oldest pose')
        #     self.trajectory_path_msg.poses = self.trajectory_path_msg.poses[1:]
        #     self.trajectory_path_msg.poses.append(pose_stamped_msg)

    
    def odom_callback(self, msg):
        self.publish_trajectory_path(msg.pose.pose.position)
        

if __name__ == '__main__':
    laser_to_local_grid = TrajectoryTrack()
    rospy.spin()