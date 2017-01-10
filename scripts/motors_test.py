#!/usr/bin/env python
import rospy
import tf

import math

from iarc7_safety.SafetyClient import SafetyClient

from iarc7_msgs.msg import OrientationThrottleStamped
from iarc7_msgs.msg import BoolStamped

from std_srvs.srv import SetBool

from geometry_msgs.msg import TwistStamped

if __name__ == '__main__':
    rospy.init_node('waypoints_test', anonymous=True)

    rospy.wait_for_service('uav_arm')
    arm_service = rospy.ServiceProxy('uav_arm', SetBool)

    safety_client = SafetyClient('motors_test')
    safety_client.form_bond()

    command_pub = rospy.Publisher('uav_direction_command', OrientationThrottleStamped, queue_size=0)

    armed = False
    while armed == False :
        try:
            armed = arm_service(True)
        except rospy.ServiceException as exc:
            print("Could not arm: " + str(exc))

    rate = rospy.Rate(30)
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

        rate.sleep()

