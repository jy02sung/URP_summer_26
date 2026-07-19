#!/usr/bin/env python3
"""Keep recording force topics discoverable before the grasp command begins."""

import rospy
from std_msgs.msg import Float64


if __name__ == '__main__':
    rospy.init_node('force_graph_preview')
    publishers = [
        rospy.Publisher('/dual_arm/grip_force/left', Float64, queue_size=1),
        rospy.Publisher('/dual_arm/grip_force/right', Float64, queue_size=1),
        rospy.Publisher('/dual_arm/grip_force/target', Float64, queue_size=1),
    ]
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for publisher in publishers:
            publisher.publish(Float64(0.0))
        rate.sleep()
