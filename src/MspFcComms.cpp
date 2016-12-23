////////////////////////////////////////////////////////////////////////////
//
// Msp flight controller node.
//
// Implements node behaviours specific to MSP FC's
// Also defines main entry point for the MSP FC node.
//
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <cmath>
#include <string>

#include "iarc7_fc_comms/MspFcComms.hpp"
#include "iarc7_fc_comms/CommonConf.hpp"
#include "iarc7_fc_comms/MspConf.hpp"
#include "iarc7_fc_comms/MspCommands.hpp"

#include "serial/serial.h"

#include "iarc7_msgs/Float64Stamped.h"
#include "iarc7_msgs/OrientationThrottleStamped.h"

namespace FcComms
{

    MspFcComms::MspFcComms() : fc_serial_(nullptr)
    {
        // Empty, nothing to do for now.
    }

    MspFcComms::~MspFcComms()
    {
        delete fc_serial_;
    }
    // Scale the direction commands to rc values and put them in the rc values array.
    // Send the rc values
    void MspFcComms::sendFcDirection(const iarc7_msgs::OrientationThrottleStamped::ConstPtr& message)
    {
        // Constrain inputs
        double constrained_roll     = std::max(CommonConf::kMinAllowedRoll,
                                               std::min(CommonConf::kMaxAllowedRoll,
                                                        message->data.roll));
        double constrained_pitch    = std::max(CommonConf::kMinAllowedPitch,
                                               std::min(CommonConf::kMaxAllowedPitch,
                                                        message->data.pitch));
        double constrained_throttle = std::max(CommonConf::kMinAllowedThrottle,
                                               std::min(CommonConf::kMaxAllowedThrottle,
                                                        message->throttle));
        double constrained_yaw_rate = std::max(CommonConf::kMinAllowedYawRate,
                                               std::min(CommonConf::kMaxAllowedYawRate,
                                                        message->data.yaw));

        // Send out the rx values using sendMessage.
        double roll_output     = (constrained_roll * FcCommsMspConf::kMspRollScale)
                               + FcCommsMspConf::kMspMidPoint;
        double pitch_output    = (constrained_pitch * FcCommsMspConf::kMspPitchScale)
                               + FcCommsMspConf::kMspMidPoint;
        double throttle_output = constrained_throttle * FcCommsMspConf::kMspThrottleScale
                               + FcCommsMspConf::kMspThrottleStartPoint;
        double yaw_rate_output = (constrained_yaw_rate * FcCommsMspConf::kMspYawScale)
                               + FcCommsMspConf::kMspMidPoint;

        translated_rc_values_[0] = static_cast<uint16_t>(roll_output);
        translated_rc_values_[1] = static_cast<uint16_t>(pitch_output);
        translated_rc_values_[2] = static_cast<uint16_t>(throttle_output);
        translated_rc_values_[3] = static_cast<uint16_t>(yaw_rate_output);
        ROS_INFO("THROTTLE: %f", throttle_output);


        #pragma GCC warning "Handle return"
        (void)sendRc();
    }

    void MspFcComms::sendArmRequest(const iarc7_msgs::BoolStamped::ConstPtr& message)
    {
        // Try to arm, Values over 1800 arm the FC
        translated_rc_values_[4] = (message->data == true) ? 2000 : 1000;
        #pragma GCC warning "Handle return"
        (void)sendRc();
    }

    // Send the rc commands to the FC using the member array of rc values.
    FcCommsReturns MspFcComms::getRawRC(uint16_t (&rc_values)[18]) 
    {
        MSP_RC msp_getrawRC;
        sendMessage(msp_getrawRC);        
        msp_getrawRC.getRc(rc_values); 
    }

    // Debug function to print the raw rc values, not called from anywhere but can be used for debugging
    void MspFcComms::printRawRC()
    {
        uint16_t raw_values[18];
        getRawRC(raw_values);
        char RC_info[150];
        int j = 0;
        for (int i = 0 ; i < 18 ; i++) {
            j+=snprintf(&RC_info[j], (j >= 150 ? 0 : 150 - j), ", %d", raw_values[i]);
        }
        ROS_INFO(RC_info);
    }

    bool MspFcComms::isAutoPilotAllowed()
    {
        uint16_t autoRCvalues[18];
        getRawRC(autoRCvalues);
        if(autoRCvalues[8] > 1800)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    FcCommsReturns MspFcComms::sendRc()
    {
        MSP_SET_RAW_RC msp_rc;
        msp_rc.packRc(translated_rc_values_);
        #pragma GCC warning "Handle return"
        sendMessage(msp_rc);
    }

    FcCommsReturns MspFcComms::getBattery(float& voltage)
    {
        MSP_ANALOG analog;
        #pragma GCC warning "Handle return"
        sendMessage(analog);
        voltage = analog.getVolts();

        return FcCommsReturns::kReturnOk;
    }

    FcCommsReturns MspFcComms::getStatus(bool& armed, bool& auto_pilot, bool& failsafe)
    {
        // Adding the autopilot flag is probably going to require modifying the FC firmware
        // And could be quite a bit of work.
        #pragma GCC warning "Finish implementing auto_pilot and failsafe"
        MSP_STATUS status;
        sendMessage(status);
        armed = status.getArmed();

        MSP_RC rc_channels;
        sendMessage(rc_channels);
        auto_pilot = isAutoPilotAllowed(); 

        return FcCommsReturns::kReturnOk;
    }

    // Get the attitude of the FC in the order roll pitch yaw
    FcCommsReturns MspFcComms::getAttitude(double (&attitude)[3])
    {
        MSP_ATTITUDE att;
        (void)sendMessage(att);
        att.getAttitude(attitude);
        return FcCommsReturns::kReturnOk;
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
                ROS_ASSERT_MSG(false, "FC_Comms has undefined state.");

                // Needed as a placeholder, we aren't coming back though.
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
                ROS_ERROR("Connection to FC failed");
                return FcCommsReturns::kReturnError;
            }

            // If connect is being called again be sure to free memory.
            if(fc_serial_ != nullptr)
            {
                delete fc_serial_;
            }

            // Make a serial port object
            fc_serial_ = new serial::Serial(serial_port, FcCommsMspConf::kBaudRate,
                                            serial::Timeout::simpleTimeout(FcCommsMspConf::kSerialTimeoutMs));

            // Wait for the serial port to be open.
            if(fc_serial_->isOpen() == false)
            {
                ROS_WARN("Serial port not open.");
                fc_comms_status_= FcCommsStatus::kDisconnected;
                ROS_ERROR("Connection to FC failed");
                return FcCommsReturns::kReturnError;
            }

            ROS_INFO("FC_Comms Connected to FC");
            fc_comms_status_ = FcCommsStatus::kConnected;

            // Pause to allow the flight controller to come up
            ros::Duration(2.0).sleep();

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
            ROS_ERROR("FC_comms did not find target device.");
            return FcCommsReturns::kReturnError;
        }

        return FcCommsReturns::kReturnOk;
    }

    FcCommsReturns MspFcComms::handleComms()
    {   
        // Check Connection
        // Check that the serial port is still open.
        if(fc_serial_->isOpen() == false)
        {
            ROS_ERROR("FC serial port unexpectedly closed");
            fc_comms_status_ = FcCommsStatus::kDisconnected;
            return FcCommsReturns::kReturnError;
        }

        return FcCommsReturns::kReturnOk;
    }

    // Implementation to send a receive a response from the flight controller
    // Protocol specification here: http://www.stefanocottafavi.com/msp-the-multiwii-serial-protocol/
    template<typename T>
    FcCommsReturns MspFcComms::sendMessage(T& message)
    {
        if(fc_comms_status_ == FcCommsStatus::kConnected)
        {
            // Check length of data section
            if(message.data_length > FcCommsMspConf::kMspMaxDataLength)
            {
                ROS_ERROR("FC_Comms data section > kMspMaxDataLength was attempted.");
                return FcCommsReturns::kReturnError;
            }

            // Add the header, data_length, and message code
            uint8_t packet[FcCommsMspConf::kMspNonDataLength + message.data_length];

            std::copy(FcCommsMspConf::kMspSendHeader, FcCommsMspConf::kMspSendHeader + 3, packet);
            packet[3] = message.data_length;
            packet[4] = message.message_id;
            
            // Start off checksum calculation
            uint8_t checksum{message.data_length ^ message.message_id};

            #pragma GCC warning "Convert for loops in this function to some cleaner form of array copy"
            // Copy data into message and finish calculating checksum
            for(int i = 0; i < message.data_length; i++)
            {
                packet[FcCommsMspConf::kMspPacketDataOffset + i] = message.send[i];
                checksum ^= message.send[i];
            }

            // Add checksum to packet
            packet[FcCommsMspConf::kMspPacketDataOffset + message.data_length] = checksum;

            try
            {
                fc_serial_->write(packet, FcCommsMspConf::kMspNonDataLength + message.data_length);
            }
            // Catch if there is an error writing
            catch(const std::exception& e)
            {
                ROS_ERROR("FC_Comms error sending MSP packet");
                ROS_ERROR("Exception: %s", e.what());
                fc_comms_status_ = FcCommsStatus::kDisconnected;
                return FcCommsReturns::kReturnError;
            }

            #pragma GCC warning "It would be good to split this off to another function"

            try
            {
                // Now receive
                std::string header;
                if(fc_serial_->read(header, FcCommsMspConf::kMspHeaderSize) != FcCommsMspConf::kMspHeaderSize){
                    ROS_ERROR("Possible disconnection, wrong number of bytes received");
                    fc_comms_status_ = FcCommsStatus::kDisconnected;
                }
                
                // Header is of type std::string so we can use this type of comparison
                if(header != FcCommsMspConf::kMspReceiveHeader)
                {
                    ROS_ERROR("Invalid message header from FC.");
                    return FcCommsReturns::kReturnError;
                }

                // Read length of data section
                uint8_t data_length{0};
                if(fc_serial_->read(&data_length, 1) != 1){
                    ROS_ERROR("Possible disconnection, wrong number of bytes received");
                    fc_comms_status_ = FcCommsStatus::kDisconnected;
                }
                // Read rest of message
                // Resulting buffer length is data length + message id length + crc
                uint8_t message_length_no_header = data_length + 1 + 1;
                uint8_t buffer[message_length_no_header];
                #pragma GCC warning "TODO check how many bytes were received."
                uint8_t message_length_read = fc_serial_->read(&buffer[0], message_length_no_header);
                if(buffer[0] != packet[4])
                {
                    ROS_ERROR("Received packet id does not match the one sent previously");
                }
                // Log errors
                // Check that the lengths read are correct
                if(message_length_read != message_length_no_header)
                {
                    ROS_ERROR("FC_Comms not all bytes received, expected: %d, got: %d", message_length_no_header, message_length_read);
                    fc_comms_status_ = FcCommsStatus::kDisconnected;
                    return FcCommsReturns::kReturnError;
                }

                // Calculate checksum from received data
                // Only checksum up to message_length_read_1 to avoid xoring the checksum
                uint8_t crc = data_length;
                for(int i = 0; i < message_length_no_header - 1; i++)
                {
                    crc ^= buffer[i];
                }

                // Compare checksums
                if(crc != buffer[message_length_no_header-1])
                {
                    ROS_ERROR("FC_Comms CRC receive error, expected: %x, got: %x", crc, buffer[message_length_no_header - 1]);
                    return FcCommsReturns::kReturnError;
                }

                // Copy output buffer response to message
                std::copy(buffer+1, buffer+1+data_length, message.response);
            }
            // Catch if there is an error reading
            catch(const std::exception& e)
            {
                ROS_ERROR("FC_Comms error reading MSP packet");
                ROS_ERROR("Exception: %s", e.what());
                fc_comms_status_ = FcCommsStatus::kDisconnected;
                return FcCommsReturns::kReturnError;
            }
        }
        else
        {
            ROS_WARN("Attempted to send FC message without being connected, message id: %d", message.message_id);
        }

        ROS_DEBUG("FC_COMMS %s sent/received succesfully", message.string_name);
        return FcCommsReturns::kReturnOk;
    }
}
  