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
#include "iarc7_safety/SafetyClient.hpp"
#include "iarc7_msgs/BoolStamped.h"
#include "iarc7_msgs/FlightControllerStatus.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"
#include "std_msgs/Float32.h"
#include "std_srvs/SetBool.h"


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
        CommonFcComms();

        // Publish to the FC sensor topics
        FcCommsReturns __attribute__((warn_unused_result)) publishTopics();

        // Attempt reconnection
        void reconnect();

        // Update information from flight controller and send information
        void update();

        // Update flight controller armed information
        void updateArmed();

        // Update flight controller auto pilot status information
        void updateAutoPilotEnabled();

        // Update flight controller battery information
        void updateBattery();

        // Update flight controller attitude information
        void updateAttitude();

        // Send arm or direction message to flight controller
        void updateDirection();

        // Activate the safety response of the flight controller impl
        void activateFcSafety();

        // Publish the flight controller status
        void publishFcStatus();

        // Send out the transform for the level_quad to quad
        void sendOrientationTransform(double (&attitude)[3]);

        inline void uavDirectionCommandMessageHandler(
                const iarc7_msgs::OrientationThrottleStamped::ConstPtr& message) {
            last_direction_command_message_ptr_ = message;
            have_new_direction_command_message_ = true;
        }

        bool uavArmServiceHandler(
                std_srvs::SetBool::Request& request,
                std_srvs::SetBool::Response& response);

        // This node's handle
        ros::NodeHandle nh_;

        // Safety client
        Iarc7Safety::SafetyClient safety_client_;

        // Publishers for FC sensors
        ros::Publisher battery_publisher;
        ros::Publisher status_publisher;

        // Just use the default constructor
        T flightControlImpl_;

        // Subscriber for uav_angle values
        ros::Subscriber uav_angle_subscriber;

        // Subscriber for uav_throttle valuess
        ros::Subscriber uav_throttle_subscriber;

        // Service to arm copter
        ros::ServiceServer uav_arm_service;

        iarc7_msgs::OrientationThrottleStamped::ConstPtr last_direction_command_message_ptr_;

        bool have_new_direction_command_message_ = false;

        typedef void (CommonFcComms::*CommonFcCommsMemFn)();

        CommonFcCommsMemFn sequenced_updates[3] = {&CommonFcComms::updateArmed,
                                                   &CommonFcComms::updateAutoPilotEnabled,
                                                   &CommonFcComms::updateBattery
                                                  };

        uint32_t num_sequenced_updates = sizeof(sequenced_updates) / sizeof(CommonFcCommsMemFn);

        uint32_t current_sequenced_update = 0;

        bool fc_armed_ = false;

        bool fc_failsafe_ = false;

        bool fc_auto_pilot_enabled_ = false;
    };
}

using namespace FcComms;

template<class T>
CommonFcComms<T>::CommonFcComms() :
nh_(),
safety_client_(nh_, "fc_comms_msp"),
battery_publisher(),
status_publisher(),
flightControlImpl_(),
uav_angle_subscriber(),
uav_throttle_subscriber(),
uav_arm_service(),
last_direction_command_message_ptr_()
{
    //sequenced_updates[0] = this->updateFcStatus;
    //sequenced_updates[1] = this->updateFcStatus;
}

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
    ROS_ASSERT_MSG(safety_client_.formBond(), "fc_comms_msp: Could not form bond with safety client");

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

    uav_arm_service = nh_.advertiseService("uav_arm",
                                       &CommonFcComms::uavArmServiceHandler,
                                       this);
    if (!uav_arm_service) {
        ROS_ERROR("CommonFcComms failed to create arming service");
        return FcCommsReturns::kReturnError;
    }

    ROS_INFO("FC Comms registered and subscribed to topics");
    return FcCommsReturns::kReturnOk;
}

// Main run loop of node.
template<class T>
FcCommsReturns CommonFcComms<T>::run()
{

    ros::Rate rate(CommonConf::kFcSensorsUpdateRateHz);

    while(ros::ok())
    {
        update();
        ros::spinOnce();
        rate.sleep();
    }

    // Disconnect from FC.
    return flightControlImpl_.disconnect();
}

// Attempt to arm the flight controller
template<class T>
bool CommonFcComms<T>::uavArmServiceHandler(
                std_srvs::SetBool::Request& request,
                std_srvs::SetBool::Response& response)
{
    ROS_INFO("Uav arm service handler called");

    bool auto_pilot;
    FcCommsReturns status = flightControlImpl_.isAutoPilotAllowed(auto_pilot);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to find out if auto pilot is enabled");
        return false;
    }

    // If auto pilot is not enabled we have no power
    if(!auto_pilot)
    {
        ROS_INFO("Failed to arm or disarm the FC: auto pilot is disabled");
        response.success = false;
        response.message = "disabled";
        return true;
    }

    // Now attempt to arm or disarm
    status = flightControlImpl_.setArm(request.data);
    if (status != FcCommsReturns::kReturnOk)
    {
        ROS_ERROR("iarc7_fc_comms: Failed to send arm message");
        return false;
    }

    // Check to see if the craft actually armed
    ros::Time start_time = ros::Time::now();
    while((ros::Time::now() - start_time) < ros::Duration(CommonConf::kMaxArmDelay))
    {
        bool armed;
        FcCommsReturns status = flightControlImpl_.isArmed(armed);
        if (status != FcCommsReturns::kReturnOk) {
            ROS_ERROR("Failed to retrieve flight controller arm status");
            return false;
        }

        if(armed == request.data)
        {
            ROS_INFO("FC arm or disarm set succesfully");
            response.success = true;
            return true;
        }
    }

    ROS_INFO("Failed to arm or disarm the FC: timeout out");
    response.success = false;
    response.message = "timed out";
    return true;
}

// Update flight controller arming information
template<class T>
void CommonFcComms<T>::updateArmed()
{
    bool temp_armed;
    FcCommsReturns status = flightControlImpl_.isArmed(temp_armed);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to retrieve flight controller status");
    }
    else
    {
        ROS_DEBUG("Armed: %d", temp_armed);
        fc_armed_ = temp_armed;
    }
}

// Update flight controller auto pilot status information
template<class T>
void CommonFcComms<T>::updateAutoPilotEnabled()
{
    bool temp_auto_pilot;
    FcCommsReturns status = flightControlImpl_.isAutoPilotAllowed(temp_auto_pilot);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to find out if auto pilot is enabled");
    }
    else
    {
        ROS_DEBUG("Autopilot_enabled: %d", temp_auto_pilot);
        fc_auto_pilot_enabled_ = temp_auto_pilot;
    }
}

// Update flight controller battery information
template<class T>
void CommonFcComms<T>::updateBattery()
{
    std_msgs::Float32 battery;
    FcCommsReturns status = flightControlImpl_.getBattery(battery.data);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("iarc7_fc_comms: Failed to retrieve flight controller battery info");
    }
    else
    {
        ROS_DEBUG("iarc7_fc_comms: Battery level: %f", battery.data);
        battery_publisher.publish(battery);
    }
}

// Update flight controller attitude information
template<class T>
void CommonFcComms<T>::updateAttitude()
{
    double attitude[3];
    FcCommsReturns status = flightControlImpl_.getAttitude(attitude);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("iarc7_fc_comms: Failed to retrieve attitude from flight controller");
    }
    else
    {
        ROS_DEBUG("iarc7_fc_comms: Attitude: %f %f %f", attitude[0], attitude[1], attitude[2]);
        sendOrientationTransform(attitude);
    }
}

// Send direction message to flight controller
template<class T>
void CommonFcComms<T>::updateDirection()
{
    FcCommsReturns status{FcCommsReturns::kReturnOk};

    if(have_new_direction_command_message_)
    {
        status = flightControlImpl_.processDirectionCommandMessage(
                     last_direction_command_message_ptr_);
        if (status == FcCommsReturns::kReturnOk)
        {
            have_new_direction_command_message_ = false;
        }
        else
        {
            ROS_ERROR("iarc7_fc_comms: Failed to send direction message");
        }
    }
}

// Activate the safety response of the flight controller impl
template<class T>
void CommonFcComms<T>::activateFcSafety()
{
    FcCommsReturns status = flightControlImpl_.safetyLand();
    
    if (status == FcCommsReturns::kReturnOk) {
        ROS_WARN("iarc7_fc_comms: succesfully sent safety land request");
    } else {
        ROS_ERROR("iarc7_fc_comms: failed to send safety land request");
    }
}

// Attempt to reconnect, blocking until successful
template<class T>
void CommonFcComms<T>::reconnect() {
    FcCommsReturns status;
    ros::Time start_time = ros::Time::now();

    status = flightControlImpl_.disconnect();
    if(status == FcCommsReturns::kReturnOk)
    {
        status = flightControlImpl_.connect();
    }

    if(status == FcCommsReturns::kReturnOk)
    {
        ROS_INFO("iarc7_fc_comms: Succesful reconnection to flight controller");
    }
    else
    {
        ROS_ERROR("iarc7_fc_comms: Failed reconnection to flight controller");
    }
}

// Update information from flight controller and send information
template<class T>
void CommonFcComms<T>::update()
{
    ros::Time times = ros::Time::now();

    // Check the safety client before updating anything
    ROS_ASSERT_MSG(!safety_client_.isFatalActive(), "iarc7_fc_comms: fatal event from safety");

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
            if(status != FcCommsReturns::kReturnOk)
            {
                ROS_ERROR("iarc7_fc_comms: flight controller impl could not handle comms");
            }

            // Check if we need to have a safety response
            if(safety_client_.isSafetyActive())
            {
                activateFcSafety();
            }
            else
            {
                updateDirection();
                updateAttitude();
            }

            (this->*sequenced_updates[current_sequenced_update])();
            current_sequenced_update = (current_sequenced_update + 1) % num_sequenced_updates;

            publishFcStatus();

            ROS_DEBUG("iarc7_fc_comms: Time to update FC sensors: %f", (ros::Time::now() - times).toSec());
            break;

        case FcCommsStatus::kConnecting:
            break;

        default:
            ROS_ASSERT_MSG(false, "iarc7_fc_comms: FC_Comms has undefined state.");
    }
}

template<class T>
void CommonFcComms<T>::publishFcStatus()
{
    iarc7_msgs::FlightControllerStatus status_message;

    status_message.armed = fc_armed_;
    status_message.auto_pilot = fc_auto_pilot_enabled_;
    status_message.failsafe = fc_failsafe_;

    status_publisher.publish(status_message);
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
