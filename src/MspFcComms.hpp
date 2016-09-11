#ifndef MSP_FC_COMMS_HPP
#define MSP_FC_COMMS_HPP

////////////////////////////////////////////////////////////////////////////
//
// Msp flight controller node.
//
// Implements node behaviours specific to MSP FC's
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <string>
#include "CommonFcComms.hpp"
#include "serial/serial.h"
#include "iarc7_msgs/FlightControllerRx.h"


// Inherit from CommonFcComms to get the stuff any Fc is required to do.
class MspFcComms : public CommonFcComms
{
public:
    MspFcComms();
    ~MspFcComms();

protected:

    // Used to find and connect to the serial port
    virtual CommonFcComms::FcCommsReturns connect();

    // Disconnect from FC, should be called before destructor.
    virtual CommonFcComms::FcCommsReturns disconnect();

    // Subscribe to the flight controller control topic
    virtual void subscribeControl();

    // Handle periodically updating polled info.
    virtual CommonFcComms::FcCommsReturns handleComms();

    // Get the flight status of the FC.
    virtual CommonFcComms::FcCommsReturns getStatus(uint8_t& armed, uint8_t& auto_pilot, uint8_t& failsafe);

    // Get the battery voltage of the FC.
    virtual CommonFcComms::FcCommsReturns getBattery(float& voltage);

    // Subscriber for FC RC stick values
    ros::Subscriber rc_subscriber;

private:
    // Don't allow the copy constructor or assignment.
    MspFcComms(const CommonFcComms& rhs);
    MspFcComms& operator=(const CommonFcComms& rhs);

    // Find the FC from a list of serial ports using its hardware ID.
    static CommonFcComms::FcCommsReturns findFc(std::string& serial_port);

    // Connect to the serial port and identify FC.
    CommonFcComms::FcCommsReturns connectFc();

    // Send the flight controller RX values
    void sendFcRx(const iarc7_msgs::FlightControllerRx::ConstPtr& rx);

    // Send message using the MSP protocol
    template<typename T>
    CommonFcComms::FcCommsReturns sendMessage(T& message);

    // Serial object used to communicate with FC
    serial::Serial* fc_serial_;
};
#endif