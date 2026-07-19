#!/usr/bin/env python3
import math
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class SymmetricSqueeze:
    def __init__(self):
        self.left_y = None
        self.right_y = None
        self.max_velocity = 0.0
        self.box = None
        self.pub = rospy.Publisher('/dual_arm/squeeze_force_targets', Float64MultiArray,
                                   queue_size=1, latch=True)
        rospy.Subscriber('/dual_arm/left_ft_sensor', WrenchStamped, self.left_cb, queue_size=1)
        rospy.Subscriber('/dual_arm/right_ft_sensor', WrenchStamped, self.right_cb, queue_size=1)
        rospy.Subscriber('/dual_arm/joint_states', JointState, self.joint_cb, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size=1)

    def left_cb(self, msg):
        self.left_y = msg.wrench.force.y

    def right_cb(self, msg):
        self.right_y = msg.wrench.force.y

    def joint_cb(self, msg):
        if msg.velocity:
            self.max_velocity = max(self.max_velocity, max(abs(v) for v in msg.velocity))

    def model_cb(self, msg):
        if 'aruco_box_26' in msg.name:
            self.box = msg.pose[msg.name.index('aruco_box_26')]

    def send(self, left, right, common_x=0.0, common_z=0.0):
        self.pub.publish(Float64MultiArray(data=[left, right, common_x, common_z]))

    def run(self):
        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and (self.left_y is None or self.right_y is None or self.box is None):
            if time.monotonic() > deadline:
                raise RuntimeError('sensor/model state timeout')
            rospy.sleep(0.01)

        # Average the no-contact sensor bias.  External squeeze reaction is +Y on the
        # left sensor and -Y on the right sensor.
        left_samples, right_samples = [], []
        for _ in range(200):
            left_samples.append(self.left_y)
            right_samples.append(self.right_y)
            rospy.sleep(0.01)
        bias_left = sum(left_samples) / len(left_samples)
        bias_right = sum(right_samples) / len(right_samples)
        start_y = self.box.position.y
        desired_x, desired_z = 0.45, 1.225

        kp_force = 0.45
        k_balance = 0.80
        k_center_y = 400.0
        rate = rospy.Rate(100)
        trip = ''
        cmd_left = cmd_right = 0.0
        common_x = common_z = 0.0
        filtered_left = filtered_right = 0.0

        # Contact acquisition: a hand that reaches 0.5 N waits while only the other
        # continues inward.  This prevents the first contact from pushing the box.
        contact_start = time.monotonic()
        left_contact = right_contact = False
        while not rospy.is_shutdown() and time.monotonic()-contact_start < 25.0:
            raw_left=max(0.0,self.left_y-bias_left);raw_right=max(0.0,-(self.right_y-bias_right))
            filtered_left+=0.05*(raw_left-filtered_left);filtered_right+=0.05*(raw_right-filtered_right)
            left_contact=left_contact or filtered_left>=0.5
            right_contact=right_contact or filtered_right>=0.5
            if left_contact and right_contact:break
            if not left_contact:cmd_left=min(4.0,cmd_left+0.002)
            if not right_contact:cmd_right=min(4.0,cmd_right+0.002)
            dx=self.box.position.x-desired_x;dy=self.box.position.y;dz=self.box.position.z-desired_z
            common_x+=min(0.01,max(-0.01,min(5.0,max(-5.0,-500.0*dx))-common_x))
            common_z+=min(0.01,max(-0.01,min(2.0,max(-2.0,-200.0*dz))-common_z))
            if abs(dx)>0.003 or abs(dy)>0.0015 or abs(dz)>0.003 or self.max_velocity>0.5:
                trip='contact acquisition displacement'
                break
            self.send(cmd_left,cmd_right,common_x,common_z);rate.sleep()
        if not trip and not (left_contact and right_contact):trip='contact timeout'
        if trip:
            self.send(0.0,0.0)
            rospy.loginfo('symmetric squeeze: trip=%s L=%.3f R=%.3f cmdL=%.3f cmdR=%.3f max_v=%.3f',
                          trip,filtered_left,filtered_right,cmd_left,cmd_right,self.max_velocity)
            return 1

        start = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic() - start < 45.0:
            elapsed = time.monotonic() - start
            desired = min(10.0, 0.5+0.25*elapsed)  # both contacts established at 0.5 N
            raw_left = max(0.0, self.left_y - bias_left)
            raw_right = max(0.0, -(self.right_y - bias_right))
            filtered_left += 0.05*(raw_left-filtered_left)
            filtered_right += 0.05*(raw_right-filtered_right)
            measured_left, measured_right = filtered_left, filtered_right
            imbalance = measured_left - measured_right
            requested_left = desired + kp_force * (desired - measured_left) - k_balance * imbalance
            requested_right = desired + kp_force * (desired - measured_right) + k_balance * imbalance
            requested_left += k_center_y*self.box.position.y
            requested_right -= k_center_y*self.box.position.y
            # 2 N/s command slew prevents force-noise chatter at first contact.
            cmd_left += min(0.02,max(-0.02,requested_left-cmd_left))
            cmd_right += min(0.02,max(-0.02,requested_right-cmd_right))
            cmd_left = min(12.0, max(0.0, cmd_left))
            cmd_right = min(12.0, max(0.0, cmd_right))
            requested_x = min(5.0, max(-5.0, -500.0*(self.box.position.x-desired_x)))
            requested_z = min(2.0, max(-2.0, -200.0*(self.box.position.z-desired_z)))
            common_x += min(0.02,max(-0.02,requested_x-common_x))
            common_z += min(0.02,max(-0.02,requested_z-common_z))

            dx = self.box.position.x - desired_x
            dy = self.box.position.y - start_y
            dz = self.box.position.z - desired_z
            if abs(dx) > 0.008 or abs(dy) > 0.008 or abs(dz) > 0.008:
                trip = 'box displacement'
            elif measured_left > 12.0 or measured_right > 12.0:
                trip = 'force limit'
            elif self.max_velocity > 0.5:
                trip = 'velocity limit'
            if trip:
                break
            self.send(cmd_left, cmd_right, common_x, common_z)
            rate.sleep()

        if trip:
            self.send(0.0, 0.0)
        else:
            # Keep the final P-only force loop active for a five-second hold check.
            hold_start = time.monotonic()
            while not rospy.is_shutdown() and time.monotonic() - hold_start < 5.0:
                measured_left = max(0.0, self.left_y - bias_left)
                measured_right = max(0.0, -(self.right_y - bias_right))
                imbalance = measured_left - measured_right
                cmd_left = min(12.0, max(0.0, 10.0 + kp_force*(10.0-measured_left)-k_balance*imbalance))
                cmd_right = min(12.0, max(0.0, 10.0 + kp_force*(10.0-measured_right)+k_balance*imbalance))
                cmd_left = min(12.0, max(0.0, cmd_left+k_center_y*self.box.position.y))
                cmd_right = min(12.0, max(0.0, cmd_right-k_center_y*self.box.position.y))
                common_x = min(5.0, max(-5.0, -500.0*(self.box.position.x-desired_x)))
                common_z = min(2.0, max(-2.0, -200.0*(self.box.position.z-desired_z)))
                self.send(cmd_left, cmd_right, common_x, common_z)
                rate.sleep()
            if (abs(self.box.position.x-desired_x)>0.003 or abs(self.box.position.y)>0.003 or
                    abs(self.box.position.z-desired_z)>0.005):
                trip='final box displacement'
                self.send(0.0,0.0)
        measured_left = max(0.0, self.left_y - bias_left)
        measured_right = max(0.0, -(self.right_y - bias_right))
        rospy.loginfo('symmetric squeeze: trip=%s L=%.3f R=%.3f cmdL=%.3f cmdR=%.3f max_v=%.3f',
                      trip or 'none', measured_left, measured_right, cmd_left, cmd_right, self.max_velocity)
        return 1 if trip else 0


if __name__ == '__main__':
    rospy.init_node('symmetric_squeeze')
    try:
        raise SystemExit(SymmetricSqueeze().run())
    except Exception as exc:
        rospy.logerr('%s', exc)
        raise
