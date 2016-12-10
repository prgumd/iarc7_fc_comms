#ifndef COMMON_FC_COMMS_HPP
#define COMMON_FC_COMMS_HPP

////////////////////////////////////////////////////////////////////////////
//
// Common flight controller node.
//
// Contains declarations for objects common to all flight
// control communication nodes. Handles a lot of ROS specifics.
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "CommonConf.hpp"
#include "iarc7_msgs/FlightControllerStatus.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"
#include "std_msgs/Float32.h"


//mspfccomms actually handles the serial communication
namespace FcComms{

    template<class T>
    class CommonFcComms
    {
    public:
        // Used to make sure this class remains a singleton.
        static CommonFcComms<T>& getInstance();

        FcCommsReturns init();
        
        FcCommsReturns run(int argc, char **argv);

        //Delete copy constructors, assignment operator
        CommonFcComms(const CommonFcComms& rhs) = delete;
        CommonFcComms& operator=(const CommonFcComms& rhs) = delete;

    private:

        // Class can only be made by class
        CommonFcComms() = default;

        // This node's handle.
        ros::NodeHandle nh_; 

        // Publishers for FC sensors
        ros::Publisher battery_publisher;
        ros::Publisher status_publisher;

        // Publish to the FC sensor topics
        void publishTopics();

        // Callback to update the sensors on the FC
        void updateSensors(const ros::TimerEvent&);

        // Send FC angles
        void sendFcDirection(const iarc7_msgs::OrientationThrottleStamped::ConstPtr& message);

        // Send out the transform for the level_quad to quad
        void sendOrientationTransform(double (&attitude)[3]);

        // Just use the default constructor
        T flightControlImpl_;

        // Subscriber for uav_angle values
        ros::Subscriber uav_angle_subscriber;

        // Subscriber for uav_throttle valuess
        ros::Subscriber uav_throttle_subscriber;

        // Subscriber for uav_arm
        ros::Subscriber uav_arm_subscriber;

    };
}

using namespace FcComms;

template<class T>
CommonFcComms<T>& CommonFcComms<T>::getInstance()
{
    // Allow only one instance
    static CommonFcComms<T>* instance = nullptr;
    if (instance == nullptr) {
        instance = new CommonFcComms<T>;
    }
    return *instance;
}

// Use to connect to topics
template<class T>
FcCommsReturns CommonFcComms<T>::init()
{
    // Should error check or something

    battery_publisher = nh_.advertise<std_msgs::Float32>("fc_battery", 50);
    status_publisher = nh_.advertise<iarc7_msgs::FlightControllerStatus>("fc_status", 50);
    uav_angle_subscriber = nh_.subscribe("uav_direction_command", 100, &T::sendFcDirection, &flightControlImpl_);
    uav_arm_subscriber = nh_.subscribe("uav_arm", 100, &T::sendArmRequest, &flightControlImpl_);

    ROS_INFO("FC Comms registered and subscribed to topics");

    return FcCommsReturns::kReturnOk;
}

// Main run loop of node.
template<class T>
FcCommsReturns CommonFcComms<T>::run(int argc, char **argv)
{

    // Set up a 10hz Timer to call updateSensors
    ros::Timer timer = nh_.createTimer(ros::Duration(CommonConf::kFcSensorsUpdatePeriod), &CommonFcComms::updateSensors, this);

    // Wait till we get told to stop and service all callbacks
    ros::spin();

    // Disconnect from FC.
    //TODO handle if something went wrong disconnecting.
    #pragma GCC warning "TODO handle failure"
    (void)flightControlImpl_.disconnect();

    return FcCommsReturns::kReturnOk;
}

// Push the sensor data to the appropriate topics.
template<class T>
void CommonFcComms<T>::publishTopics()
{
    iarc7_msgs::FlightControllerStatus fc;
    #pragma GCC warning "TODO handle failure"
    bool temp_armed;
    bool temp_auto_pilot;
    bool temp_failsafe;
    flightControlImpl_.getStatus(temp_armed, temp_auto_pilot, temp_failsafe);

    fc.armed = temp_armed;
    fc.auto_pilot = temp_auto_pilot;
    fc.failsafe = temp_failsafe;
    
    std_msgs::Float32 battery;
    #pragma GCC warning "TODO handle failure"
    flightControlImpl_.getBattery(battery.data);

    ROS_DEBUG("Autopilot_enabled: %d", fc.auto_pilot);
    ROS_DEBUG("Armed: %d", fc.armed);
    status_publisher.publish(fc);
    ROS_DEBUG("Battery level: %f", battery.data);
    battery_publisher.publish(battery);


    double attitude[3];
    flightControlImpl_.getAttitude(attitude);
    ROS_DEBUG("Attitude: %f %f %f", attitude[0], attitude[1], attitude[2]);
    sendOrientationTransform(attitude);
}

// Update the sensors on the flight controller
template<class T>
void CommonFcComms<T>::updateSensors(const ros::TimerEvent&)
{

    ros::Time times = ros::Time::now();


    // Do different things based on the current connection status.
    switch(flightControlImpl_.getConnectionStatus())
    {
        case FcCommsStatus::kDisconnected:
            ROS_WARN("FC_Comms disconnected");
            // We don't care about the return value we just reconnect.
            (void)flightControlImpl_.connect();
            break;
        
        case FcCommsStatus::kConnected:
            #pragma GCC warning "TODO handle failure"
            flightControlImpl_.handleComms();
            publishTopics();

            //ROS_WARN("%f to run", (ros::Time::now() - times).toSec());
            break;

        case FcCommsStatus::kConnecting:
            break;

        default:
            ROS_ASSERT_MSG(false, "FC_Comms has undefined state.");
    }
}

// Send out the transform for the level_quad to quad
template<class T>
void CommonFcComms<T>::sendOrientationTransform(double (&attitude)[3])
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = CommonConf::kTfParentName;
  transformStamped.child_frame_id = CommonConf::kTfChildName;

  tf2::Quaternion q;

  // This assumes the values are returned in the form roll pitch yaw
  q.setRPY(attitude[0], attitude[1], attitude[2]);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

#endif
