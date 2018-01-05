////////////////////////////////////////////////////////////////////////////
//
// PX4 flight controller node.
//
// Implements node behaviours specific to PX4 through mavros FC's
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <cmath>
#include <sstream>
#include <string>

#include "iarc7_fc_comms/PX4FcComms.hpp"
#include "iarc7_fc_comms/CommonConf.hpp"
#include "iarc7_fc_comms/MspConf.hpp"

namespace FcComms {

PX4FcComms::PX4FcComms(ros::NodeHandle& nh)
    : nh_(nh),
      fc_comms_status_(),
      mavros_current_state_()
{
    mavros_state_sub_ = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, &PX4FcComms::mavrosStateCallback, this);
    mavros_local_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    mavros_arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    mavros_set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");
}

void PX4FcComms::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
    mavros_current_state_ = *msg;

    if(mavros_current_state_.connected) {
        fc_comms_status_ = FcCommsStatus::kConnected;
    }
    else
    {
        // Don't set to disconnected if currently connecting
        // the connect() function set fc_comms_status_ to connecting
        // and it shouldn't be overwritten
        if(fc_comms_status_ != FcCommsStatus::kConnecting)
        {
            fc_comms_status_ = FcCommsStatus::kDisconnected;
        }
    }
}

FcCommsReturns PX4FcComms::safetyLand()
{
    return FcCommsReturns::kReturnOk;
}

// Scale the direction commands to rc values and put them in the rc values array.
// Send the rc values
FcCommsReturns PX4FcComms::processDirectionCommandMessage(
    const iarc7_msgs::OrientationThrottleStamped::ConstPtr&)
{
    /*// Constrain inputs
    double constrained_roll     = std::max(double(CommonConf::kMinAllowedRoll),
                                           std::min(double(CommonConf::kMaxAllowedRoll),
                                                    message->data.roll));
    double constrained_pitch    = std::max(double(CommonConf::kMinAllowedPitch),
                                           std::min(double(CommonConf::kMaxAllowedPitch),
                                                    message->data.pitch));
    double constrained_throttle = std::max(double(CommonConf::kMinAllowedThrottle),
                                           std::min(double(CommonConf::kMaxAllowedThrottle),
                                                    message->throttle));
    double constrained_yaw_rate = std::max(double(CommonConf::kMinAllowedYawRate),
                                           std::min(double(CommonConf::kMaxAllowedYawRate),
                                                    message->data.yaw));*/
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::setArm(bool )
{
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::postArm(bool )
{
    return FcCommsReturns::kReturnOk;
}

// Get the acceleration in m/s^2 and the angular velocities in rad/s
FcCommsReturns PX4FcComms::getIMU(double (&)[3], double(&)[3])
{
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::isAutoPilotAllowed(bool& )
{
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::getBattery(float& )
{
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::isArmed(bool& )
{
    return FcCommsReturns::kReturnOk;
}

// Get the attitude of the FC in the order roll pitch yaw in radians
FcCommsReturns PX4FcComms::getAttitude(double (&)[3])
{
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::calibrateAccelerometer()
{
    // PX4 has no need of accelerometer calibration
    return FcCommsReturns::kReturnOk;
}

// Disconnect from FC, should be called before destructor.
FcCommsReturns PX4FcComms::disconnect()
{
    // Disconnecting from mavros is not supported
    return FcCommsReturns::kReturnOk;
}


FcCommsReturns PX4FcComms::connect()
{
    ROS_INFO("FC_Comms PX4 beginning connection");
    fc_comms_status_ = FcCommsStatus::kConnecting;

    ros::Rate rate(30);
    // wait for mavros to indicate connection
    while(ros::ok() && fc_comms_status_ != FcCommsStatus::kConnected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("FC_Comms PX4 Connected to FC");

    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::handleComms()
{
    return FcCommsReturns::kReturnOk;
}

} // namespace FcComms
