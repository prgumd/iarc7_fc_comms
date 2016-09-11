////////////////////////////////////////////////////////////////////////////
//
// Common flight controller node.
//
// Contains declarations for objects common to all flight
// control communication nodes.
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include "CommonFcComms.hpp"
#include "std_msgs/Float32.h"
#include "iarc7_msgs/FlightControllerStatus.h"
#include "CommonConf.hpp"

CommonFcComms::CommonFcComms() : fc_comms_status_(kDisconnected)
{

}

// Use to connect to topics
CommonFcComms::FcCommsReturns CommonFcComms::init()
{
    // Should error check or something

    battery_publisher = nh_.advertise<std_msgs::Float32>("fc_battery", 50);
    status_publisher = nh_.advertise<iarc7_msgs::FlightControllerStatus>("fc_status", 50);

    subscribeControl();

    ROS_INFO("FC Comms registered and subscribed to topics");

    return kReturnOk;
}

// Main run loop of node.
CommonFcComms::FcCommsReturns CommonFcComms::run(int argc, char **argv)
{

    // Set up a 10hz Timer to call updateSensors
    ros::Timer timer = nh_.createTimer(ros::Duration(CommonConf::kFcSensorsUpdatePeriod), &CommonFcComms::updateSensors, this);

    // Wait till we get told to stop and service all callbacks
    ros::spin();

    // Disconnect from FC.
    disconnect();

    return kReturnOk;
}

// Push the sensor data to the appropriate topics.
void CommonFcComms::publishTopics()
{
    iarc7_msgs::FlightControllerStatus fc;
    getStatus(fc.armed, fc.auto_pilot, fc.failsafe);
    
    std_msgs::Float32 battery;
    getBattery(battery.data);

    status_publisher.publish(fc);
    battery_publisher.publish(battery);
}

// Update the sensors on the flight controller
void CommonFcComms::updateSensors(const ros::TimerEvent&)
{
    // Do different things based on the current connection status.
    switch(fc_comms_status_)
    {
        case kDisconnected:
            ROS_WARN("FC_Comms disconnected");
            connect();
            ROS_INFO("Back from connect");
            break;
        
        case kConnected:
            ROS_INFO("FC_comms updating FC sensors");
            handleComms();
            publishTopics();
            break;

        case kConnecting:
            break;

        default:
            ROS_ERROR("FC_Comms has undefined state.");
    }
}
