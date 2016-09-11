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

class CommonFcComms
{
public:

    // Used for the result of a command
    enum FcCommsReturns
    {
        kReturnOk,
        kReturnError
    };

    FcCommsReturns init();
    virtual FcCommsReturns run(int argc, char **argv);

protected:

    // Default constructor, only children can make an instance of this class.
    CommonFcComms();

    // Used for the current state of communication with flight controller
    enum FcCommsStatus
    {
        kDisconnected,
        kConnected,
        kConnecting
    };

    // This nodes handle.
    ros::NodeHandle nh_; 

    // Publishers for FC sensors
    ros::Publisher battery_publisher;
    ros::Publisher status_publisher;

    // Publish to the FC sensor topics
    virtual void publishTopics();

    // Connect to the FC using childs implementation
    virtual CommonFcComms::FcCommsReturns connect() = 0;

    // Disconnect from FC, should be called before destructor.
    virtual CommonFcComms::FcCommsReturns disconnect() = 0;

    // Give the child a chance to handle any communication tasks periodically
    virtual FcCommsReturns handleComms() = 0;

    // Get the FC Status using childs implementation
    virtual CommonFcComms::FcCommsReturns getStatus(uint8_t& armed, uint8_t& auto_pilot, uint8_t& failsafe) = 0;

    // Get the FC battery level using the childs implementation
    virtual CommonFcComms::FcCommsReturns getBattery(float& voltage) = 0;

    // Allow the child implementation to subscribe to its own control channel
    // So that it can use its on message types for the control style.
    virtual void subscribeControl() = 0;

    // State of communication with flight controller
    FcCommsStatus fc_comms_status_;

private:
    //Make constructors, copy constructors, assignment operator private
    CommonFcComms(const CommonFcComms& rhs);
    CommonFcComms& operator=(const CommonFcComms& rhs);

    // Callback to update the sensors on the FC
    void updateSensors(const ros::TimerEvent&);

};

#endif