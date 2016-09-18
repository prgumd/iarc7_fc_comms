////////////////////////////////////////////////////////////////////////////
//
// Msp flight controller node.
//
// Implements node behaviours specific to MSP FC's
// Also defines main entry point for the MSP FC node.
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <string>
#include "MspFcComms.hpp"
#include "CommonFcComms.hpp"
#include "serial/serial.h"
#include "MspConf.hpp"
#include "iarc7_msgs/FlightControllerRx.h"

using namespace FcComms;

MspFcComms::MspFcComms() : fc_serial_(nullptr)
{
    // Empty, nothing to do for now.
}

MspFcComms::~MspFcComms()
{
    delete fc_serial_;
}

void MspFcComms::sendFcAngles(float pitch, float yaw, float roll)
{
    // Send out the rx values using sendMessage.
}

// Disconnect from FC, should be called before destructor.
FcCommsReturns MspFcComms::disconnect()
{
    ROS_INFO("Disconnecting from FC");

    // Handle each connection state seperately.
    switch(fc_comms_status_)
    {
        case FcCommsStatus::kConnected:
            fc_serial_->close();
            break;

        case FcCommsStatus::kConnecting:
            if(fc_serial_->isOpen())
            {
                fc_serial_->close();
            }

        case FcCommsStatus::kDisconnected:
            break;

        default:
            ROS_ERROR("FC_Comms has undefined state.");
            return FcCommsReturns::kReturnError;
    }

    fc_comms_status_ = FcCommsStatus::kDisconnected;
    return FcCommsReturns::kReturnOk;
}


FcCommsReturns MspFcComms::connect()
{
    try
    {
        ROS_INFO("FC_Comms beginning connection");
        fc_comms_status_ = FcCommsStatus::kConnecting;

        // Find the flight controller by the hardware ID.
        std::string serial_port;
        if(findFc(serial_port) == FcCommsReturns::kReturnError)
        {
            fc_comms_status_= FcCommsStatus::kDisconnected;
            return FcCommsReturns::kReturnError;
        }

        // If connect is being called again be sure to free memory.
        if(fc_serial_ != nullptr)
        {
            delete fc_serial_;
        }

        // Make a serial port object
        fc_serial_ = new serial::Serial(serial_port, FcCommsMspConf::kBaudRate, serial::Timeout::simpleTimeout(1000));

        // Wait for the serial port to be open.
        if(fc_serial_->isOpen() == false)
        {
            ROS_WARN("Serial port not open.");
            fc_comms_status_= FcCommsStatus::kDisconnected;
            return FcCommsReturns::kReturnError;
        }

        ROS_INFO("FC_Comms Connected to FC");
        fc_comms_status_ = FcCommsStatus::kConnected;

        return FcCommsReturns::kReturnOk;
    }
    // Catch if there is an error making the connection.
    catch(const std::exception& e)
    {
        fc_comms_status_ = FcCommsStatus::kDisconnected;
        ROS_ERROR("Exception: %s", e.what());
        return FcCommsReturns::kReturnError;
    }
}

FcCommsReturns MspFcComms::findFc(std::string& serial_port)
{
    // List of serial ports
    std::vector<serial::PortInfo> devices = serial::list_ports();

    bool found(false);
    std::vector<serial::PortInfo>::iterator iter = devices.begin();
    while( iter != devices.end() && found == false)
    {
        serial::PortInfo device = *iter++;

        // If we've found something with the same hardware id as our FC
        if(device.hardware_id == FcCommsMspConf::kHardwareId)
        {
            ROS_INFO("FC_comms found target device.");
            serial_port = device.port;

            // Exit loop since we've found the port
            found = true;
        }
    }

    // FC not found
    if(found == false)
    {
        ROS_WARN("FC_comms did not find target device.");
        return FcCommsReturns::kReturnError;
    }

    return FcCommsReturns::kReturnOk;
}

FcCommsReturns MspFcComms::getStatus(uint8_t& armed, uint8_t& auto_pilot, uint8_t& failsafe)
{
    // Stubbed should send a message to get the flight controller status and update it.
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns MspFcComms::getBattery(float& voltage)
{
    // Stubbed should construct message and
    // return sendMessage<BatteryUpdate>();
    return FcCommsReturns::kReturnOk;
}

FcCommsReturns MspFcComms::handleComms()
{
    // Check Connection
    // Check that the serial port is still open.
    if(fc_serial_->isOpen() == false)
    {
        ROS_WARN("FC serial port closed");
        fc_comms_status_ = FcCommsStatus::kDisconnected;
        return FcCommsReturns::kReturnError;
    }

    return FcCommsReturns::kReturnOk;
}

template<typename T>
FcCommsReturns MspFcComms::sendMessage(T& message)
{
    if(fc_comms_status_ == FcCommsStatus::kConnected)
    {
        // Send a message
        // Check for errors
    }
    else
    {
        ROS_WARN("Attempted to send FC message without being connected: %s", message.kType);
    }
}
