#ifndef MSP_CONF_HPP
#define MSP_CONF_HPP

////////////////////////////////////////////////////////////////////////////
//
// Configuration for MSP flight controller node.
//
////////////////////////////////////////////////////////////////////////////

namespace FcComms
{

    struct FcCommsMspConf
    {
        static const uint32_t kBaudRate{9600};

        // Needs space on end to bug in serial package
        static constexpr char const * const kHardwareId{"USB VID:PID=1a86:7523 "};

        // In seconds.
        static constexpr const float kFcSensorsUpdatePeriod{0.2};

        // Non Data MSP packet length in bytes
        static const uint8_t kMspNonDataLength = 6;

        // Non Data MSP packet length in bytes
        static const uint8_t kMspPacketDataOffset = 5;

        // Longest MSP data section
        static const uint8_t kMspMaxDataLength = 127;

        // Length of packet header
        static const uint8_t kMspHeaderSize = 3;

        // Header for receiving a message
        static constexpr char const * const kMspReceiveHeader{"$M>"};

        // Header for sending a message
        static constexpr char const * const kMspSendHeader{"$M<"};

    };

}

#endif // End MSP_CONF_HPP
