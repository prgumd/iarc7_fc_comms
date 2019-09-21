#!/usr/bin/env python
import rospy
import tf

import math

from iarc7_msgs.msg import OrientationThrottleStamped
from iarc7_msgs.msg import BoolStamped
from iarc7_msgs.srv import Arm

from geometry_msgs.msg import TwistStamped

if __name__ == '__main__':
    rospy.init_node('waypoints_test', anonymous=True)

    rospy.wait_for_service('uav_arm')
    arm_service = rospy.ServiceProxy('uav_arm', Arm)
    rospy.logerr('done waiting')

    command_pub = rospy.Publisher('uav_direction_command', OrientationThrottleStamped, queue_size=0)

    rate = rospy.Rate(30)
    throttle = 0
    while not rospy.is_shutdown():
        armed = False
        while armed == False :
            try:
                rospy.logerr('Trying to arm')
                armed = arm_service(True)
            except rospy.ServiceException as exc:
                rospy.logerr('Could not arm: ' + str(exc))
        rospy.logerr('Armed')

        command = OrientationThrottleStamped()
        command.header.stamp = rospy.Time.now()

        while not rospy.is_shutdown():
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

            rate.sleep()

        while armed == True :
            try:
                rospy.logerr('Trying to disarm')
                armed = arm_service(False)
            except rospy.ServiceException as exc:
                rospy.logerr('Could not disarm: ' + str(exc))
        rospy.logerr('disarmed')

        rospy.Time.sleep(2.0)
