#!/usr/bin/env python
import rospy
import tf

import math

from iarc7_safety.SafetyClient import SafetyClient

from iarc7_msgs.msg import OrientationThrottleStamped
from iarc7_msgs.msg import BoolStamped

from geometry_msgs.msg import TwistStamped


def constrain(x, l, h):
    return min(h, max(x, l))

if __name__ == '__main__':
    rospy.init_node('waypoints_test', anonymous=True)

    safety_client = SafetyClient('motors_test')
    safety_client.form_bond()

    command_pub = rospy.Publisher('uav_direction_command', OrientationThrottleStamped, queue_size=0)
    arm_pub = rospy.Publisher('uav_arm', BoolStamped, queue_size=0)

    rate = rospy.Rate(10)
    throttle = 0
    while not rospy.is_shutdown():

        # Exit immediately if fatal
        if safety_client.is_fatal_active():
            break;

        # Turn off motors if put into safety mode
        if safety_client.is_safety_active():
            arm = BoolStamped()
            arm.data = False
            arm_pub.publish(arm)

            command = OrientationThrottleStamped()
            command.header.stamp = rospy.Time.now()
            command_pub.publish(command)

        else:
            command = OrientationThrottleStamped()
            command.header.stamp = rospy.Time.now()

            # Three second throttle ramp and three seconds off
            throttle = (throttle + 1) % 150
            if throttle <= 100:
                command.throttle = float(throttle)/100.0
                command.data.pitch = 0.0
                command.data.roll = 0.0
                command.data.yaw = 0.0
                command_pub.publish(command)
            else:
                command.throttle = 0.0
                command.data.pitch = 0.0
                command.data.roll = 0.0
                command.data.yaw = 0.0
                command_pub.publish(command)

            arm = BoolStamped()
            arm.data = True
            arm_pub.publish(arm)

        rate.sleep()

