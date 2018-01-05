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

#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"

namespace FcComms {

PX4FcComms::PX4FcComms()
{
    // Empty, nothing to do for now.
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
    return FcCommsReturns::kReturnOk;
}

// Disconnect from FC, should be called before destructor.
FcCommsReturns PX4FcComms::disconnect()
{
    ROS_INFO("Disconnecting from FC");

    ROS_INFO("Succesfull disconnection from FC");

    fc_comms_status_ = FcCommsStatus::kDisconnected;
    return FcCommsReturns::kReturnOk;
}


FcCommsReturns PX4FcComms::connect()
{
    ROS_INFO("FC_Comms beginning connection");
    fc_comms_status_ = FcCommsStatus::kConnecting;

    ROS_INFO("FC_Comms Connected to FC");
    fc_comms_status_ = FcCommsStatus::kConnected;

    return FcCommsReturns::kReturnOk;
}

FcCommsReturns PX4FcComms::handleComms()
{
    return FcCommsReturns::kReturnOk;
}

} // namespace FcComms
