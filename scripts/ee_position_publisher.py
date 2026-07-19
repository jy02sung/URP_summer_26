#!/usr/bin/env python3
"""Publish and display both palm-center positions for recording a grasp demo."""

import sys

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from tf.transformations import quaternion_matrix


class EndEffectorPositionPublisher:
    FRAMES = {
        # Gazebo merges the massless fixed grip-frame links into L/R_EE.
        # These offsets recover the palm-disc centers defined in the URDF.
        'left': ('dual_arm::L_EE', (0.0, -0.0325, 0.0)),
        'right': ('dual_arm::R_EE', (0.0, 0.0325, 0.0)),
    }

    def __init__(self):
        self.positions = {}
        self.pose_publishers = {
            side: rospy.Publisher('/dual_arm/ee_position/{}/pose'.format(side), PoseStamped,
                                  queue_size=1)
            for side in self.FRAMES
        }
        self.scalar_publishers = {
            (side, axis): rospy.Publisher('/dual_arm/ee_position/{}/{}'.format(side, axis),
                                          Float64, queue_size=1)
            for side in self.FRAMES for axis in ('x', 'y', 'z')
        }
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self.print_positions)

    def link_states_cb(self, message):
        indices = {name: index for index, name in enumerate(message.name)}
        for side, (link, offset) in self.FRAMES.items():
            index = indices.get(link)
            if index is None:
                continue
            pose = message.pose[index]
            rotation = quaternion_matrix((pose.orientation.x, pose.orientation.y,
                                          pose.orientation.z, pose.orientation.w))
            offset_world = rotation.dot((offset[0], offset[1], offset[2], 0.0))
            position = pose.position
            position.x += offset_world[0]
            position.y += offset_world[1]
            position.z += offset_world[2]
            self.positions[side] = position

            stamped = PoseStamped()
            stamped.header.stamp = rospy.Time.now()
            stamped.header.frame_id = 'world'
            stamped.pose = pose
            self.pose_publishers[side].publish(stamped)
            for axis in ('x', 'y', 'z'):
                self.scalar_publishers[(side, axis)].publish(Float64(getattr(pose.position, axis)))

    def print_positions(self, _event):
        if len(self.positions) != 2:
            return
        left, right = self.positions['left'], self.positions['right']
        line = ('[EE world, m]  L: ({:+.3f}, {:+.3f}, {:+.3f})'
                '  R: ({:+.3f}, {:+.3f}, {:+.3f})').format(
                    left.x, left.y, left.z, right.x, right.y, right.z)
        sys.stdout.write('\r' + line + ' ')
        sys.stdout.flush()


if __name__ == '__main__':
    rospy.init_node('ee_position_publisher')
    EndEffectorPositionPublisher()
    rospy.spin()
