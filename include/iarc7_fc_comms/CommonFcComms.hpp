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
#include "iarc7_msgs/BoolStamped.h"
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

        FcCommsReturns __attribute__((warn_unused_result)) init();

        FcCommsReturns __attribute__((warn_unused_result)) run();

        //Delete copy constructors, assignment operator
        CommonFcComms(const CommonFcComms& rhs) = delete;
        CommonFcComms& operator=(const CommonFcComms& rhs) = delete;

    private:

        // Class can only be made by class
        CommonFcComms() = default;

        // Publish to the FC sensor topics
        FcCommsReturns __attribute__((warn_unused_result)) publishTopics();

        // Attempt reconnection until timeout
        // Timeout defaults to zero (don't timeout)
        void reconnect(const ros::Duration& timeout = ros::Duration(0));

        // Callback to update the sensors on the FC
        void updateSensors(const ros::TimerEvent&);

        // Send out the transform for the level_quad to quad
        void sendOrientationTransform(double (&attitude)[3]);

        inline void uavDirectionCommandMessageHandler(
                const iarc7_msgs::OrientationThrottleStamped::ConstPtr& message) {
            last_direction_command_message_ptr_ = message;
            have_new_direction_command_message_ = true;
        }

        inline void uavArmMessageHandler(
                const iarc7_msgs::BoolStamped::ConstPtr& message) {
            last_arm_message_ptr_ = message;
            have_new_arm_message_ = true;
        }

        // This node's handle
        ros::NodeHandle nh_;

        // Publishers for FC sensors
        ros::Publisher battery_publisher;
        ros::Publisher status_publisher;

        // Just use the default constructor
        T flightControlImpl_;

        // Subscriber for uav_angle values
        ros::Subscriber uav_angle_subscriber;

        // Subscriber for uav_throttle valuess
        ros::Subscriber uav_throttle_subscriber;

        // Subscriber for uav_arm
        ros::Subscriber uav_arm_subscriber;

        iarc7_msgs::OrientationThrottleStamped::ConstPtr last_direction_command_message_ptr_;
        iarc7_msgs::BoolStamped::ConstPtr last_arm_message_ptr_;

        bool have_new_direction_command_message_ = false;
        bool have_new_arm_message_ = false;
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
    battery_publisher = nh_.advertise<std_msgs::Float32>("fc_battery", 50);
    if (!battery_publisher) {
        ROS_ERROR("CommonFcComms failed to create battery publisher");
        return FcCommsReturns::kReturnError;
    }

    status_publisher = nh_.advertise<iarc7_msgs::FlightControllerStatus>(
            "fc_status", 50);
    if (!status_publisher) {
        ROS_ERROR("CommonFcComms failed to create status publisher");
        return FcCommsReturns::kReturnError;
    }

    uav_angle_subscriber = nh_.subscribe("uav_direction_command",
                                         100,
                                         &CommonFcComms::uavDirectionCommandMessageHandler,
                                         this);
    if (!uav_angle_subscriber) {
        ROS_ERROR("CommonFcComms failed to create angle subscriber");
        return FcCommsReturns::kReturnError;
    }

    uav_arm_subscriber = nh_.subscribe("uav_arm",
                                       100,
                                       &CommonFcComms::uavArmMessageHandler,
                                       this);
    if (!uav_arm_subscriber) {
        ROS_ERROR("CommonFcComms failed to create arm subscriber");
        return FcCommsReturns::kReturnError;
    }

    ROS_INFO("FC Comms registered and subscribed to topics");
    return FcCommsReturns::kReturnOk;
}

// Main run loop of node.
template<class T>
FcCommsReturns CommonFcComms<T>::run()
{
    // Set up a timer to call updateSensors
    ros::Timer timer = nh_.createTimer(
            ros::Duration(CommonConf::kFcSensorsUpdatePeriod),
            &CommonFcComms::updateSensors,
            this);

    // Wait till we get told to stop and service all callbacks
    ros::spin();

    // Disconnect from FC.
    return flightControlImpl_.disconnect();
}

// Push the sensor data to the appropriate topics.
template<class T>
FcCommsReturns CommonFcComms<T>::publishTopics()
{
    FcCommsReturns status;

    iarc7_msgs::FlightControllerStatus fc;
    bool temp_armed;
    bool temp_auto_pilot;
    bool temp_failsafe;
    status = flightControlImpl_.getStatus(temp_armed, temp_auto_pilot, temp_failsafe);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to retrieve flight controller status");
        return status;
    }
    fc.armed = temp_armed;
    fc.auto_pilot = temp_auto_pilot;
    fc.failsafe = temp_failsafe;
    ROS_DEBUG("Autopilot_enabled: %d", fc.auto_pilot);
    ROS_DEBUG("Armed: %d", fc.armed);
    status_publisher.publish(fc);

    std_msgs::Float32 battery;
    status = flightControlImpl_.getBattery(battery.data);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to retrieve flight controller battery info");
        return status;
    }
    ROS_DEBUG("Battery level: %f", battery.data);
    battery_publisher.publish(battery);

    double attitude[3];
    status = flightControlImpl_.getAttitude(attitude);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to retrieve attitude from flight controller");
        return status;
    }
    ROS_DEBUG("Attitude: %f %f %f", attitude[0], attitude[1], attitude[2]);
    sendOrientationTransform(attitude);

    return FcCommsReturns::kReturnOk;
}

// Attempt to reconnect, blocking until successful
template<class T>
void CommonFcComms<T>::reconnect(const ros::Duration& timeout) {
    FcCommsReturns status;
    ros::Time start_time = ros::Time::now();
    do {
        status = flightControlImpl_.disconnect();
    } while (status != FcCommsReturns::kReturnOk
          && (timeout == ros::Duration(0) || ros::Time::now() - start_time < timeout));

    do {
        status = flightControlImpl_.connect();
    } while (status != FcCommsReturns::kReturnOk
          && (timeout == ros::Duration(0) || ros::Time::now() - start_time < timeout));
}

// Update the sensors on the flight controller
template<class T>
void CommonFcComms<T>::updateSensors(const ros::TimerEvent&)
{
    ros::Time times = ros::Time::now();

    FcCommsReturns status;

    // Do different things based on the current connection status.
    switch(flightControlImpl_.getConnectionStatus())
    {
        case FcCommsStatus::kDisconnected:
            ROS_WARN("FC_Comms disconnected");
            reconnect();
            break;

        case FcCommsStatus::kConnected:
            status = flightControlImpl_.handleComms();
            while (status != FcCommsReturns::kReturnOk) {
                reconnect();
                status = flightControlImpl_.handleComms();
            }

            while (have_new_direction_command_message_) {
                status = flightControlImpl_.processDirectionCommandMessage(
                             last_direction_command_message_ptr_);
                if (status == FcCommsReturns::kReturnOk) {
                    have_new_direction_command_message_ = false;
                } else {
                    reconnect();
                }
            }

            while (have_new_arm_message_) {
                status = flightControlImpl_.processArmMessage(
                             last_arm_message_ptr_);
                if (status == FcCommsReturns::kReturnOk) {
                    have_new_arm_message_ = false;
                } else {
                    reconnect();
                }
            }

            status = publishTopics();
            while (status != FcCommsReturns::kReturnOk) {
                reconnect();
                status = publishTopics();
            }

            ROS_DEBUG("Time to update FC sensors: %f", (ros::Time::now() - times).toSec());
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
