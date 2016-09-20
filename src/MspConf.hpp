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
        static constexpr const char* kHardwareId = "USB VID:PID=1a86:7523 ";

        // In seconds.
        static constexpr const float kFcSensorsUpdatePeriod{0.2};
    };
}

#endif // End MSP_CONF_HPP
