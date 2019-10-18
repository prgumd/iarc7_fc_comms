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
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "iarc7_msgs/Arm.h"
#include "iarc7_msgs/BoolStamped.h"
#include "iarc7_msgs/FlightControllerStatus.h"
#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationAnglesStamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"
#include <ros_utils/ParamUtils.hpp>

#include <sensor_msgs/Imu.h>
#include "std_srvs/SetBool.h"

#include "CommonConf.hpp"

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

        // Calibrate the FC's accelerometer
        void calibrateAccelerometer();

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

        // Get acceleration data from the flight controller
        void updateAccelerations();

        // Activate the safety response of the flight controller impl
        void activateFcSafety();

        // Publish the flight controller status
        void publishFcStatus();

        // Send out the orientation data for the quad
        // Can also send transform based on flag
        void sendOrientation(double (&attitude)[3]);

        // Send out the accelerations from the FC
        void sendIMU(double (&accelerations)[3], double (&angular_velocities)[3]);

        inline void uavDirectionCommandMessageHandler(
                const iarc7_msgs::OrientationThrottleStamped::ConstPtr& message) {
            last_direction_command_message_ptr_ = message;
            have_new_direction_command_message_ = true;
        }

        inline void landingDetectedMessageHandler(
                const iarc7_msgs::BoolStamped::ConstPtr& message) {
            last_landing_detected_message_ptr_ = message;
        }

        bool uavArmServiceHandler(
                iarc7_msgs::Arm::Request& request,
                iarc7_msgs::Arm::Response& response);

        // This node's handle
        ros::NodeHandle nh_;

        // NodeHandle in this node's namespace
        ros::NodeHandle private_nh_;

        // Publishers for FC sensors
        ros::Publisher battery_publisher;
        ros::Publisher status_publisher;
        ros::Publisher imu_publisher;
        ros::Publisher orientation_pub_;

        // Just use the default constructor
        T flightControlImpl_;

        // Broadcaster to send transforms with
        tf2_ros::TransformBroadcaster transform_broadcaster_;

        // Subscriber for uav_angle values
        ros::Subscriber uav_angle_subscriber;

        // Subscriber for landing detection
        ros::Subscriber landing_detected_subscriber;

        // Service to arm copter
        ros::ServiceServer uav_arm_service;

        iarc7_msgs::OrientationThrottleStamped::ConstPtr last_direction_command_message_ptr_;

        iarc7_msgs::BoolStamped::ConstPtr last_landing_detected_message_ptr_;

        ros::Duration valid_landing_detected_message_delay_;

        ros::Duration orientation_timestamp_offset_;

        ros::Duration imu_timestamp_offset_;

        bool have_new_direction_command_message_ = false;

        typedef void (CommonFcComms::*CommonFcCommsMemFn)();

        std::vector<CommonFcCommsMemFn> sequenced_updates = {
                &CommonFcComms::updateArmed,
                &CommonFcComms::updateAutoPilotEnabled
            };

        uint32_t current_sequenced_update = 0;

        bool fc_armed_ = false;

        bool fc_failsafe_ = false;

        bool fc_auto_pilot_enabled_ = false;

        bool initial_heading_as_offset_ = false;

        bool calibrate_accelerometer_ = false;

        bool publish_orientation_transform_ = false;

        double initial_heading_offset_ = std::nan("");

        double imu_accel_x_var_ = 0.0f;
        double imu_accel_y_var_ = 0.0f;
        double imu_accel_z_var_ = 0.0f;

        double imu_gyro_x_var_ = 0.0f;
        double imu_gyro_y_var_ = 0.0f;
        double imu_gyro_z_var_ = 0.0f;
    };
}

using namespace FcComms;

template<class T>
CommonFcComms<T>::CommonFcComms() :
nh_(),
private_nh_("~"),
battery_publisher(),
status_publisher(),
imu_publisher(),
orientation_pub_(),
flightControlImpl_(),
transform_broadcaster_(),
uav_angle_subscriber(),
landing_detected_subscriber(),
uav_arm_service(),
last_direction_command_message_ptr_(),
last_landing_detected_message_ptr_(),
valid_landing_detected_message_delay_(),
orientation_timestamp_offset_(),
imu_timestamp_offset_(),
imu_accel_x_var_(),
imu_accel_y_var_(),
imu_accel_z_var_(),
imu_gyro_x_var_(),
imu_gyro_y_var_(),
imu_gyro_z_var_()
{
    if (ros_utils::ParamUtils::getParam<bool>(private_nh_,
                                              "publish_fc_battery")) {
        sequenced_updates.push_back(&CommonFcComms::updateBattery);
    }

    initial_heading_as_offset_ =
                            ros_utils::ParamUtils::getParam<bool>(
                            private_nh_, "initial_heading_as_offset");

    calibrate_accelerometer_ =
                            ros_utils::ParamUtils::getParam<bool>(
                            private_nh_, "calibrate_accelerometer");

    publish_orientation_transform_ =
                            ros_utils::ParamUtils::getParam<bool>(
                            private_nh_, "publish_orientation_transform");

    valid_landing_detected_message_delay_ = ros::Duration(
                        ros_utils::ParamUtils::getParam<double>(
                        private_nh_, "valid_landing_detected_message_delay"));

    orientation_timestamp_offset_ = ros::Duration(
                                  ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_orientation_timestamp_offset"));

    imu_timestamp_offset_ = ros::Duration(
                                  ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_timestamp_offset"));

    imu_accel_x_var_ = ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_accel_x_var");

    imu_accel_y_var_ = ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_accel_y_var");

    imu_accel_z_var_ = ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_accel_z_var");

    imu_gyro_x_var_ = ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_gyro_x_var");

    imu_gyro_y_var_ = ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_gyro_y_var");

    imu_gyro_z_var_ = ros_utils::ParamUtils::getParam<double>(
                                  private_nh_,
                                  "imu_gyro_z_var");
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
    ROS_DEBUG("fc_comms_msp: Forming bond with safety client");
    ROS_DEBUG("fc_comms_msp: Formed bond with safety client");

    uav_arm_service = nh_.advertiseService("uav_arm",
                                       &CommonFcComms::uavArmServiceHandler,
                                       this);
    if (!uav_arm_service) {
        ROS_ERROR("CommonFcComms failed to create arming service");
        return FcCommsReturns::kReturnError;
    }

    battery_publisher = nh_.advertise<iarc7_msgs::Float64Stamped>("fc_battery", 50);
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

    imu_publisher = nh_.advertise<sensor_msgs::Imu>(
            "fc_imu", 50);
    if (!imu_publisher) {
        ROS_ERROR("CommonFcComms failed to create imu publisher");
        return FcCommsReturns::kReturnError;
    }

    orientation_pub_ = nh_.advertise<iarc7_msgs::OrientationAnglesStamped>(
            "fc_orientation", 50);
    if (!orientation_pub_) {
        ROS_ERROR("CommonFcComms failed to create orientation publisher");
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

    landing_detected_subscriber = nh_.subscribe("landing_detected",
                                        100,
                                        &CommonFcComms::landingDetectedMessageHandler,
                                        this);

    ROS_DEBUG("FC Comms registered and subscribed to topics");

    if (calibrate_accelerometer_)
    {
        const ros::Time start_time = ros::Time::now();
        while (ros::ok()
               && last_landing_detected_message_ptr_ == nullptr
               && ros::Time::now()
                  < start_time
                    + ros::Duration(CommonConf::kLandingDetectedStartupTimeout)) {
            ros::spinOnce();
            ros::Duration(0.005).sleep();
        }

        if (last_landing_detected_message_ptr_ == nullptr)
        {
            ROS_ERROR("Landing detected message not received within the startup timeout");
            return FcCommsReturns::kReturnError;
        }
        else
        {
            ROS_DEBUG("FC Comms received initial landing detected message succesfully");
        }

    }

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
                iarc7_msgs::Arm::Request& request,
                iarc7_msgs::Arm::Response& response)
{
    ROS_DEBUG("Uav arm service handler called");

    bool auto_pilot;
    FcCommsReturns status = flightControlImpl_.isAutoPilotAllowed(auto_pilot);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("Failed to find out if auto pilot is enabled");
        return false;
    }

    // If auto pilot is not enabled we have no power
    if(!auto_pilot)
    {
        ROS_ERROR("Failed to arm or disarm the FC: auto pilot is disabled");
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
    while(ros::ok() && ((ros::Time::now() - start_time) < ros::Duration(CommonConf::kMaxArmDelay)))
    {
        ros::spinOnce();
        bool armed;
        FcCommsReturns status = flightControlImpl_.isArmed(armed);
        if (status == FcCommsReturns::kReturnOk) {
            if(armed == request.data)
            {
                ROS_DEBUG("FC arm or disarm set succesfully");

                status = flightControlImpl_.postArm(request.data);
                if(status != FcCommsReturns::kReturnOk) {
                    ROS_ERROR("iarc7_fc_comms: Post arm action failed");
                    return false;
                }

                response.success = true;
                return true;
            }
        }
        else
        {
            ROS_ERROR("Failed to retrieve flight controller arm status");
        }
        update();
    }

    ROS_ERROR("Failed to arm or disarm the FC: timed out");
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
    float voltage;
    FcCommsReturns status = flightControlImpl_.getBattery(voltage);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("iarc7_fc_comms: Failed to retrieve flight controller battery info");
    }
    else
    {
        iarc7_msgs::Float64Stamped battery_msg;
        battery_msg.header.stamp = ros::Time::now();
        battery_msg.data = voltage;
        ROS_DEBUG("iarc7_fc_comms: Battery level: %f", battery_msg.data);
        battery_publisher.publish(battery_msg);
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
        if(initial_heading_as_offset_) {
            if(std::isnan(initial_heading_offset_)) {
                initial_heading_offset_ = attitude[2];
                ROS_DEBUG("Initial heading: %f", initial_heading_offset_);
            }

            double yaw = attitude[2] - initial_heading_offset_;

            // Limit to 0 and 2*pi
            if(yaw < 0.0) {
                yaw += 2.0* M_PI;
            }
            else if(yaw >= 2.0 * M_PI) {
                yaw -= 2.0 * M_PI;
            }
            attitude[2] = yaw;
        }

        ROS_DEBUG("iarc7_fc_comms: Attitude: %f %f %f",
                  attitude[0],
                  attitude[1],
                  attitude[2]);

        sendOrientation(attitude);
    }
}

// Send direction message to flight controller
template<class T>
void CommonFcComms<T>::updateDirection()
{
    FcCommsReturns status{FcCommsReturns::kReturnOk};

    // If not armed send a zero state direction command
    // this ensures direction command state is such that
    // arming passes
    if(!fc_armed_) {
        iarc7_msgs::OrientationThrottleStamped::ConstPtr zero_state_direction_ptr(
            new iarc7_msgs::OrientationThrottleStamped());
        status = flightControlImpl_.processDirectionCommandMessage(
                     zero_state_direction_ptr);
        if (status != FcCommsReturns::kReturnOk)
        {
            ROS_ERROR("iarc7_fc_comms: Failed to send direction message");
        }
    }
    else if(have_new_direction_command_message_ && fc_armed_)
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

// Get the accelerations from the IMU on the flight controller
template<class T>
void CommonFcComms<T>::updateAccelerations()
{
    FcCommsReturns status{FcCommsReturns::kReturnOk};
    double accelerations[3];
    double angular_velocities[3];

    status = flightControlImpl_.getIMU(accelerations, angular_velocities);
    if (status != FcCommsReturns::kReturnOk) {
        ROS_ERROR("iarc7_fc_comms: Failed to retrieve IMU info from flight controller");
    }
    else
    {
        ROS_DEBUG("iarc7_fc_comms: Accelerations: %f %f %f",
                  accelerations[0],
                  accelerations[1],
                  accelerations[2]);

        ROS_DEBUG("iarc7_fc_comms: Angular Velocities: %f %f %f",
                  angular_velocities[0],
                  angular_velocities[1],
                  angular_velocities[2]);

        sendIMU(accelerations, angular_velocities);
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

    status = flightControlImpl_.disconnect();
    if(status == FcCommsReturns::kReturnOk)
    {
        status = flightControlImpl_.connect();
    }

    if(status == FcCommsReturns::kReturnOk)
    {
        ROS_DEBUG("iarc7_fc_comms: Succesful reconnection to flight controller");
        calibrateAccelerometer();
    }
    else
    {
        ROS_ERROR("iarc7_fc_comms: Failed reconnection to flight controller");
    }
}

// Send direction message to flight controller
template<class T>
void CommonFcComms<T>::calibrateAccelerometer()
{
    if(calibrate_accelerometer_)
    {
        if(last_landing_detected_message_ptr_->header.stamp
           > ros::Time::now() - valid_landing_detected_message_delay_)
        {
            if(last_landing_detected_message_ptr_->data)
            {
                FcCommsReturns status{FcCommsReturns::kReturnOk};

                status = flightControlImpl_.calibrateAccelerometer();

                if (status != FcCommsReturns::kReturnOk)
                {
                    ROS_ERROR("iarc7_fc_comms: Failed to calibrate accelerometer");
                }
            }
            else
            {
                ROS_WARN("Can't calibrate accelerometer, not on ground");
            }
        }
        else
        {
            ROS_ERROR("Skipping accelerometer calibration. No landing detected message within timeout");
        }
    }
}

// Update information from flight controller and send information
template<class T>
void CommonFcComms<T>::update()
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
            if(status != FcCommsReturns::kReturnOk)
            {
                ROS_ERROR("iarc7_fc_comms: flight controller impl could not handle comms");
            }

            updateDirection();
            updateAttitude();
            updateAccelerations();

            (this->*sequenced_updates[current_sequenced_update])();
            current_sequenced_update = (current_sequenced_update + 1) % sequenced_updates.size();

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

// Send out the orientation date from the quad
template<class T>
void CommonFcComms<T>::sendOrientation(double (&attitude)[3])
{
    iarc7_msgs::OrientationAnglesStamped orientation_msg;

    orientation_msg.header.stamp = ros::Time::now() + orientation_timestamp_offset_;

    orientation_msg.data.roll = attitude[0];
    orientation_msg.data.pitch = -1 * attitude[1];
    orientation_msg.data.yaw = attitude[2];

    orientation_pub_.publish(orientation_msg);

    if (publish_orientation_transform_) {
        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now() + orientation_timestamp_offset_;
        transformStamped.header.frame_id = CommonConf::kTfParentName;
        transformStamped.child_frame_id = CommonConf::kTfChildName;

        tf2::Quaternion q;
       // This assumes the values are returned in the form roll pitch yaw in radians
        q.setRPY(attitude[0], attitude[1], -attitude[2]);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        transform_broadcaster_.sendTransform(transformStamped);

        transformStamped.header.stamp = ros::Time::now() + orientation_timestamp_offset_;
        transformStamped.header.frame_id = CommonConf::kTfParentName;
        transformStamped.child_frame_id = "heading_quad";

        //tf2::Quaternion q;
       // This assumes the values are returned in the form roll pitch yaw in radians
        q.setRPY(0.0, 0.0, -attitude[2]);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        transform_broadcaster_.sendTransform(transformStamped);
    }
}

// Send out the accelerations from the quad FC
template<class T>
void CommonFcComms<T>::sendIMU(double (&accelerations)[3], double (&angular_velocities)[3])
{
  sensor_msgs::Imu imu;

  imu.header.stamp = ros::Time::now() + imu_timestamp_offset_;
  imu.header.frame_id = CommonConf::kTfChildName;

  imu.linear_acceleration.x = accelerations[0];
  imu.linear_acceleration.y = accelerations[1];
  imu.linear_acceleration.z = accelerations[2];
  imu.linear_acceleration_covariance[0] = imu_accel_x_var_;
  imu.linear_acceleration_covariance[4] = imu_accel_y_var_;
  imu.linear_acceleration_covariance[8] = imu_accel_z_var_;

  imu.angular_velocity.x = angular_velocities[0];
  imu.angular_velocity.y = angular_velocities[1];
  imu.angular_velocity.z = angular_velocities[2];
  imu.angular_velocity_covariance[0] = imu_gyro_x_var_;
  imu.angular_velocity_covariance[4] = imu_gyro_y_var_;
  imu.angular_velocity_covariance[8] = imu_gyro_z_var_;

  imu.orientation_covariance[0] = -1;

  imu_publisher.publish(imu);
}

#endif
