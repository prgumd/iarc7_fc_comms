#ifndef COMMON_CONF_HPP
#define COMMON_CONF_HPP

////////////////////////////////////////////////////////////////////////////
//
// Configuration Common to all flight controllers.
//
////////////////////////////////////////////////////////////////////////////

namespace FcComms
{

    // Used for the result of a command
    enum class FcCommsReturns
    {
        kReturnOk,
        kReturnError
    };

    // Used for the current state of communication with flight controller
    enum class FcCommsStatus
    {
        kDisconnected,
        kConnected,
        kConnecting
    };
    
    struct CommonConf
    {
        // In seconds.
        static constexpr const float kFcSensorsUpdatePeriod{0.2};

        static constexpr const char* kTfParentName{"level_quad"};
        static constexpr const char* kTfChildName{"quad"};

    };
}

#endif // End COMMON_CONF_HPP
