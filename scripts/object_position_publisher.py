#!/usr/bin/env python3
"""Print the ArUco-recognized object's pose in the world frame, refreshed in place.
Shows all zeros until the marker has been recognized at least once."""

import sys

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class ObjectPositionPublisher:
    def __init__(self):
        self.position = (0.0, 0.0, 0.0)
        self.rpy = (0.0, 0.0, 0.0)
        self.listener = tf.TransformListener()
        rospy.Subscriber('/aruco_ros/pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.print_pose)

    def pose_cb(self, msg):
        msg.header.stamp = rospy.Time(0)
        try:
            world_pose = self.listener.transformPose('world', msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        p = world_pose.pose.position
        q = world_pose.pose.orientation
        self.position = (p.x, p.y, p.z)
        self.rpy = euler_from_quaternion((q.x, q.y, q.z, q.w))

    def print_pose(self, _event):
        x, y, z = self.position
        r, p, y_ = self.rpy
        line = ('[Object world, m/rad]  pos: ({:+.3f}, {:+.3f}, {:+.3f})'
                '  rpy: ({:+.3f}, {:+.3f}, {:+.3f})').format(x, y, z, r, p, y_)
        sys.stdout.write('\r' + line + ' ')
        sys.stdout.flush()


if __name__ == '__main__':
    rospy.init_node('object_position_publisher')
    ObjectPositionPublisher()
    rospy.spin()
