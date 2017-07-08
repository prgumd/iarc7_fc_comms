#ifndef MSP_CONF_HPP
#define MSP_CONF_HPP

#include <cmath>

////////////////////////////////////////////////////////////////////////////
//
// Configuration for MSP flight controller node.
//
////////////////////////////////////////////////////////////////////////////

namespace FcComms
{

    struct FcCommsMspConf
    {
        static const uint32_t kBaudRate{230400};

        static const uint32_t kSerialTimeoutMs{75};

        static constexpr double kConnectWaitPeriod{0.2};

        static constexpr double kAccelCalibWaitPeriod{2.0};

        static constexpr double kMspWaitForReplyTimeout{0.2};

        static constexpr char const * const kHardwareId
            {"USB VID:PID=0483:5740 SNR=206438515533"};

        // Uncomment the line below to specify a particular serial port
        // instead of using the usb hardware id
        static constexpr char const * const kSerialPort
            {""};
            //{"/dev/ttyTHS2"};

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
        static constexpr const double kMspStickStartPoint{1000.0};
        static constexpr const double kMspStickMidPoint{1500.0};
        static constexpr const double kMspStickEndPoint{2000.0};
        static constexpr const double kMspMaxAngleRadians{40.0 * M_PI / 180.0};
        static constexpr const double kMspPitchScale{(kMspStickEndPoint - kMspStickMidPoint)/kMspMaxAngleRadians};
        static constexpr const double kMspRollScale{kMspPitchScale};
        static constexpr const double kMspYawScale{0.0};// Should be filled in with values based on (kMspMax-kMspMidpoint) / max rate of rotation
        static constexpr const double kMspThrottleScale{(kMspStickEndPoint-kMspStickStartPoint)};  // (Max-Min) * throttle + min where throttle is ranged from 0-1
        static constexpr const double kSafetyLandingThrottle{(1200.0 - kMspStickStartPoint)/kMspThrottleScale};

    };

}

#endif // End MSP_CONF_HPP
