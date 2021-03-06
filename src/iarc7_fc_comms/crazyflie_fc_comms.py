#!/usr/bin/env python
import rospy
import sys
import threading
import traceback
import math
from threading import Timer

from std_srvs.srv import SetBool

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped

from iarc7_msgs.msg import (OrientationThrottleStamped, FlightControllerStatus,
                            Float64Stamped, FlightControllerStatus,
                            OrientationAnglesStamped, LandingGearContactsStamped)

from iarc7_msgs.srv import Arm, ArmResponse

from iarc7_safety.SafetyClient import SafetyClient
from iarc7_safety.iarc_safety_exception import IARCFatalSafetyException

from sensor_msgs.msg import (Imu, Range, MagneticField)

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Only output errors from the logging framework
# Used in a all crazyflie demo scripts
logging.basicConfig(level=logging.ERROR)

class CrazyflieFcComms:

    def __init__(self):

        self._uav_command = None
        self._connected = False
        self._timestamp_offset = None
        self._armed = False
        self._cf = None
        self._commands_allowed = True
        self._uav_command = None
        self._vbat = None
        self._crash_detected = False
        self._crash_landed = False

        # safety 
        self._safety_client = SafetyClient('fc_comms_crazyflie')

        # Publishers for FC sensors
        self._battery_publisher = rospy.Publisher('fc_battery',
                                                  Float64Stamped,
                                                  queue_size=5)
        self._status_publisher = rospy.Publisher('fc_status',
                                                  FlightControllerStatus,
                                                  queue_size=5)
        self._imu_publisher = rospy.Publisher('fc_imu',
                                               Imu,
                                               queue_size=5)
        self._orientation_pub = rospy.Publisher('fc_orientation',
                                                 OrientationAnglesStamped,
                                                 queue_size=5)

        self._velocity_pub = rospy.Publisher('optical_flow_estimator/twist',
                                              TwistWithCovarianceStamped,
                                              queue_size=5)

        self._range_pub = rospy.Publisher('short_distance_lidar',
                                          Range,
                                          queue_size=5)

        self._switch_pub = rospy.Publisher('landing_gear_contact_switches',
                                            LandingGearContactsStamped,
                                            queue_size=5)

        self._mag_pub = rospy.Publisher('fc_mag',
                                        MagneticField,
                                        queue_size=5)

        # Subscriber for uav_angle values
        self._uav_angle_subscriber = rospy.Subscriber(
                'uav_direction_command',
                OrientationThrottleStamped,
                self._uav_command_handler)

        # Service to arm copter
        self._uav_arm_service = rospy.Service('uav_arm',
                                        Arm,
                                        self._arm_service_handler)
        self._current_height = 0.0
        self._over_height_counts = 0

    def run(self):
        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)

        scan_rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            # Use first Crazyflie found
            rospy.loginfo('Crazyflie FC Comms scanning for Crazyflies')
            crazyflies = cflib.crtp.scan_interfaces()
            if len(crazyflies) != 0:
                break
            rospy.logerr('Crazyflie FC Comms could not find Crazyflie. Trying again.')
            scan_rate.sleep()

        uri = crazyflies[0][0]

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connection_made)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        rospy.loginfo('Crazyflie FC Comms connecting to: {}'.format(uri))
        self._cf.open_link(uri)

        connected_check_rate = rospy.Rate(2)
        while not self._connected and not rospy.is_shutdown():
            connected_check_rate.sleep()
            pass

        rospy.loginfo('Crazyflie FC Comms reseting kalmann filter')
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        fast_log_stab   = LogConfig(name='high_update_rate', period_in_ms=20)
        medium_log_stab = LogConfig(name='medium_update_rate', period_in_ms=30)
        slow_log_stab   = LogConfig(name='slow_update_rate', period_in_ms=100)

        fast_log_stab.add_variable('acc.x', 'float')
        fast_log_stab.add_variable('acc.y', 'float')
        fast_log_stab.add_variable('acc.z', 'float')
        fast_log_stab.add_variable('stabilizer.roll', 'float')
        fast_log_stab.add_variable('stabilizer.pitch', 'float')
        fast_log_stab.add_variable('stabilizer.yaw', 'float')

        # kalman_states.ox and kalman_states.oy are also available
        # and provide position estimates
        medium_log_stab.add_variable('kalman_states.vx', 'float')
        medium_log_stab.add_variable('kalman_states.vy', 'float')
        medium_log_stab.add_variable('range.zrange', 'uint16_t')

        slow_log_stab.add_variable('pm.vbat', 'float')
        slow_log_stab.add_variable('mag.x', 'float')
        slow_log_stab.add_variable('mag.y', 'float')
        slow_log_stab.add_variable('mag.z', 'float')

        try:
            self._cf.log.add_config(fast_log_stab)
            self._cf.log.add_config(medium_log_stab)
            self._cf.log.add_config(slow_log_stab)

            fast_log_stab.data_received_cb.add_callback(self._receive_crazyflie_data)
            medium_log_stab.data_received_cb.add_callback(self._receive_crazyflie_data)
            slow_log_stab.data_received_cb.add_callback(self._receive_crazyflie_data)

            fast_log_stab.error_cb.add_callback(self._receive_crazyflie_error)
            medium_log_stab.error_cb.add_callback(self._receive_crazyflie_error)
            slow_log_stab.error_cb.add_callback(self._receive_crazyflie_error)

            fast_log_stab.start()
            medium_log_stab.start()
            slow_log_stab.start()
        except KeyError as e:
            rospy.logerr('Crazyflie FC Comms could not start logging,'
                  '{} not found'.format(str(e)))
        except AttributeError as e:
            rospy.logerr('Crazyflie FC Comms could not finishing configuring logging: {}'.format(str(e)))

        rospy.loginfo("Crazyflie FC Comms finished configured logs")

        # Publish a landing gear contact message so that the landing detector can start up
        switch_msg = LandingGearContactsStamped()
        switch_msg.front = True
        switch_msg.back = True
        switch_msg.left = True
        switch_msg.right = True
        self._switch_pub.publish(switch_msg)

        rospy.loginfo('Crazyflie FC Comms trying to form bond')

        # forming bond with safety client
        if not self._safety_client.form_bond():
            raise IARCFatalSafetyException('Crazyflie FC Comms could not form bond with safety client')

        rospy.loginfo('Crazyflie FC Comms done forming bond')

        # Update at the same rate as low level motion
        rate = rospy.Rate(60)

        while not rospy.is_shutdown() and self._connected:
            status = FlightControllerStatus()
            status.header.stamp = rospy.Time.now()

            status.armed = self._armed
            status.auto_pilot = True
            status.failsafe = False

            self._status_publisher.publish(status)

            if self._safety_client.is_fatal_active():
                raise IARCFatalSafetyException('Safety Client is fatal active')

            if self._safety_client.is_safety_active():
                rospy.logwarn_throttle(1.0, 'Crazyflie FC Comms activated safety response')
                self._commands_allowed = False
                self._cf.commander.send_setpoint(0, 0, 0, 0)

            if self._crash_detected:
                rospy.logwarn_throttle(1.0, 'Crazyflie FC Comms detected crash')
                self._commands_allowed = False
                if self._current_height > 0.15 and not self._crash_landed:
                    self._cf.commander.send_setpoint(0, 0, 0, 0.47 * 65535) # Max throttle is 65535
                else:
                    self._crash_landed = True
                    rospy.logwarn_throttle(1.0, 'Crazyflie FC Comms shut down motors')
                    self._cf.commander.send_setpoint(0, 0, 0, 0)

            if self._armed and self._commands_allowed:
                if self._uav_command is None:
                    # Keep the connection alive
                    self._cf.commander.send_setpoint(0, 0, 0, 0)
                else:
                    throttle = max(10000.0, self._uav_command.throttle * 65535.0) # Max throttle is 65535
                    self._cf.commander.send_setpoint(self._uav_command.data.roll * 180.0 / math.pi,
                                                     self._uav_command.data.pitch * 180.0 / math.pi,
                                                     -1.0 * self._uav_command.data.yaw * 180.0 / math.pi,
                                                     throttle)
            rate.sleep()

        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Give time for buffer to flush
        time.sleep(0.1)
        self._cf.close_link()

    # Always return success because making sure the crazyflie is
    # armed correctly isn't critical
    def _arm_service_handler(self, arm_request):
        if self._armed and not arm_request.data:
            self._cf.commander.send_setpoint(0, 0, 0, 0)
            self._armed = False
            return ArmResponse(success=True)

        elif not self._armed and arm_request.data:
            if self._vbat < 3.55 or self._vbat is None:
                rospy.logerr('Crazyflie FC Comms refusing to arm, battery to low or not received')
                return ArmResponse(success=False)
            else:
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                self._armed = True
                return ArmResponse(success=True)
        
        # State is the same as it was before. So simply return true.
        return ArmResponse(success=True)


    def _uav_command_handler(self, data):
        self._uav_command = data

    def _receive_crazyflie_data(self, timestamp, data, logconf):
        # Catch all exceptions because otherwise the crazyflie
        # library catches them all silently
        try:
            stamp = self._calculate_timestamp(timestamp)

            if logconf.name == 'high_update_rate':
                imu = Imu()

                imu.header.stamp = stamp
                imu.header.frame_id = 'quad'

                imu.linear_acceleration.x = data['acc.x'] * 9.81;
                imu.linear_acceleration.y = data['acc.y'] * 9.81;
                imu.linear_acceleration.z = data['acc.z'] * 9.81;
                imu.orientation_covariance[0] = -1;
                imu.angular_velocity_covariance[0] = -1;

                variance = 6.0
                imu.linear_acceleration_covariance[0] = variance;
                imu.linear_acceleration_covariance[4] = variance;
                imu.linear_acceleration_covariance[8] = variance;

                self._imu_publisher.publish(imu)

                orientation = OrientationAnglesStamped()
                orientation.header.stamp = stamp
                orientation.data.roll = data['stabilizer.roll'] * math.pi / 180.0
                orientation.data.pitch = data['stabilizer.pitch'] * math.pi / 180.0

                # Zero out the yaw for now. Crazyflie has a problem with yaw calculation
                # and having correct yaw is not critical for this target yet.
                orientation.data.yaw = -0.0 * data['stabilizer.yaw'] * math.pi / 180.0
                self._orientation_pub.publish(orientation)

                if abs(orientation.data.roll) > 1.5 or abs(orientation.data.pitch) > 1.5:
                    rospy.logwarn('Crazyflie FC Comms maximimum attitude angle exceeded')
                    self._crash_detected = True

            elif logconf.name == 'medium_update_rate':
                twist = TwistWithCovarianceStamped()
                twist.header.stamp = stamp - rospy.Duration.from_sec(0.020)
                twist.header.frame_id = 'heading_quad'
                twist.twist.twist.linear.x = data['kalman_states.vx']
                twist.twist.twist.linear.y = data['kalman_states.vy']
                twist.twist.twist.linear.z = 0.0

                variance = 0.0001
                twist.twist.covariance[0] = variance
                twist.twist.covariance[7] = variance

                self._velocity_pub.publish(twist)

                range_msg = Range()
                range_msg.header.stamp = stamp - rospy.Duration.from_sec(0.050)

                range_msg.radiation_type = Range.INFRARED
                range_msg.header.frame_id = '/short_distance_lidar'

                range_msg.field_of_view = 0.3
                range_msg.min_range = 0.01
                range_msg.max_range = 1.6

                self._current_height = data['range.zrange'] / 1000.0
                range_msg.range = self._current_height

                if range_msg.range > 2.0:
                    self._over_height_counts = self._over_height_counts + 1
                else:
                    self._over_height_counts = 0

                if self._over_height_counts > 10:
                    rospy.logwarn('Crazyflie FC Comms maximimum height exceeded')
                    self._crash_detected = True

                self._range_pub.publish(range_msg)

                switch_msg = LandingGearContactsStamped()
                switch_msg.header.stamp = stamp
                pressed = data['range.zrange'] < 50
                switch_msg.front = pressed
                switch_msg.back = pressed
                switch_msg.left = pressed
                switch_msg.right = pressed

                self._switch_pub.publish(switch_msg)

            elif logconf.name == 'slow_update_rate':
                battery = Float64Stamped()
                battery.header.stamp = stamp
                battery.data = data['pm.vbat']
                self._vbat = data['pm.vbat']
                self._battery_publisher.publish(battery)

                mag = MagneticField()
                mag.header.stamp = stamp
                mag.magnetic_field.x = data['mag.x']
                mag.magnetic_field.y = data['mag.y']
                mag.magnetic_field.z = data['mag.z']
                self._mag_pub.publish(mag)

            else:
                rospy.logerr('Crazyflie FC Comms received message block with unkown name')
        except Exception as e:
            rospy.logerr("Crazyflie FC Comms error receiving data: {}".format(str(e)))
            rospy.logerr(traceback.format_exc())


    def _calculate_timestamp(self, timestamp):
        if self._timestamp_offset is None:
            self._timestamp_offset = (rospy.Time.now(), timestamp)

        stamp = self._timestamp_offset[0] + rospy.Duration.from_sec((float(timestamp - self._timestamp_offset[1])/1000.0) - 0.025)
        return stamp


    def _receive_crazyflie_error(self, logconf, msg):
        rospy.logerr('Crazyflie FC Comms error: {}, {}'.format(logconf.name, msg))

    def _connection_made(self, uri):
        rospy.loginfo('Crazyflie FC Comms connected to {}'.format(uri))
        self._connected = True

    def _connection_failed(self, uri, msg):
        rospy.logerr('Crazyflie FC Comms connection {} failed: {}'.format(uri, msg))
        self._connected = False

    def _connection_lost(self, uri, msg):
        rospy.logerr('Crazyflie FC Comms connection to {} lost: {}'.format(uri, msg))
        self._connected = False

    def _disconnected(self, uri):
        rospy.loginfo('Crazyflie FC Comms disconnected from {}'.format(uri))
        self._connected = False

if __name__ == '__main__':
    rospy.init_node('crazyflie_fc_comms')

    craizeflie_fc_comms = CrazyflieFcComms()
    
    try:
        craizeflie_fc_comms.run()
    except Exception, e:
        rospy.logfatal("Error in Crazyflie FC Comms while running.")
        rospy.logfatal(str(e))
        rospy.logfatal(traceback.format_exc())
        raise
    finally:
        rospy.signal_shutdown("Crazyflie FC Comms shutdown")
