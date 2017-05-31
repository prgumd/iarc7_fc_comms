#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64
from iarc7_msgs.msg import OrientationThrottleStamped
from std_srvs.srv import SetBool

import numpy as np

last_scale_msg = None
have_new_scale_msg = False
def scale_callback(msg):
    global last_scale_msg, have_new_scale_msg
    last_scale_msg = msg
    have_new_scale_msg = True

rospy.init_node('thrust_test')
rospy.Subscriber('/scale', Float64, scale_callback)
pub = rospy.Publisher('/uav_direction_command', OrientationThrottleStamped, queue_size=5)
rospy.logerr('Publisher created')
rospy.wait_for_service('/uav_arm')
arm_service = rospy.ServiceProxy('/uav_arm', SetBool)
rospy.logerr('Service ready')
assert arm_service(True)
rospy.logerr('Motors armed')

command = OrientationThrottleStamped()
samples = np.linspace(0.0, 1.0, 100)
forces = np.ndarray(samples.shape)
for i, sample in enumerate(samples):
    if rospy.is_shutdown():
        break
    rospy.logerr('Testing %f'%sample)
    command.throttle = sample
    pub.publish(command)
    rospy.logerr('Set throttle to %f'%sample)
    rospy.sleep(0.5)
    rospy.logerr('Waiting for scale...')
    have_new_scale_msg = False
    while not have_new_scale_msg:
        if rospy.is_shutdown():
            break
    rospy.logerr('Received thrust of %f'%last_scale_msg.data)
    forces[i] = last_scale_msg.data
np.savetxt('thrust_data.csv',
           np.hstack(np.vstack(samples), np.vstack(forces)),
           delimiter=',')
rospy.logerr('Setting throttle to 0')
command.throttle = 0
pub.publish(command)
rospy.logerr('Disarming')
arm_service(False)
rospy.logerr('Disarmed')