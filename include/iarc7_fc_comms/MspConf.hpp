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
        static const uint32_t kBaudRate{115200};

        static const uint32_t kSerialTimeoutMs{75};

        // Needs space on end to bug in serial package
        static constexpr char const * const kHardwareId{"USB VID:PID=10c4:ea60 SNR=0001"};

        // In seconds.
        static constexpr const float kFcSensorsUpdatePeriod{0.2};

        // Non Data MSP packet length in bytes
        static const uint8_t kMspNonDataLength = 6;

        // Non Data MSP packet length in bytes
        static const uint8_t kMspPacketDataOffset = 5;

        // Longest MSP data section
        static const uint8_t kMspMaxDataLength = 127;

        // Number of channels read from the flight controller
        static const uint8_t kMspReceivableChannels = 18;

        // Number of channels set on the flight controller
        static const uint8_t kMspSettableChannels = 8;

        static const uint8_t kMspAutoPilotAllowedChannel = 8;

        static const uint16_t kMspSwitchMidpoint = 1800;

        // Length of packet header
        static const uint8_t kMspHeaderSize = 3;

        // Header for receiving a message
        static constexpr char const * const kMspReceiveHeader{"$M>"};

        // Header for sending a message
        static constexpr char const * const kMspSendHeader{"$M<"};

        // MSP RC scaling factors
        static constexpr const double kMspMidPoint{1500.0};
        static constexpr const double kMspPitchScale{10.0}; // Should be filled in with values based on (kMspMax-kMspMidpoint) / max_angle
        static constexpr const double kMspRollScale{10.0};  // Should be filled in with values based on (kMspMax-kMspMidpoint) / max_angle
        static constexpr const double kMspYawScale{10.0};   // Should be filled in with values based on (kMspMax-kMspMidpoint) / max_angle
        static constexpr const double kMspThrottleStartPoint{950.0};
        static constexpr const double kMspThrottleScale{(2000.0-kMspThrottleStartPoint)};  // (Max-Min) * throttle + min where throttle is ranged from 0-1
        static constexpr const double kSafetyLandingThrottle{0.3};

    };

}

#endif // End MSP_CONF_HPP
