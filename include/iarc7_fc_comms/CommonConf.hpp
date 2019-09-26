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
    static constexpr const float kFcSensorsUpdateRateHz{200};

    static constexpr const double kMaxArmDelay = 0.1;
    static constexpr const double kLandingDetectedStartupTimeout = 5.0;

    static constexpr const double kMinAllowedRoll = -0.175;
    static constexpr const double kMaxAllowedRoll = 0.175;
    static constexpr const double kMinAllowedPitch = -0.175;
    static constexpr const double kMaxAllowedPitch = 0.175;
    static constexpr const double kMinAllowedThrottle = 0.0; // Do not set higher than 0 or arming will fail randomly
    static constexpr const double kMaxAllowedThrottle = 1.0;
    static constexpr const double kMinAllowedYawRate = -0.5;
    static constexpr const double kMaxAllowedYawRate = 0.5;

    static constexpr const char* kTfParentName{"level_quad"};
    static constexpr const char* kTfChildName{"quad"};

    // Variance of acceleration measurements (in m/2^2)
    static constexpr const double kAccelerationVariance[3] = {100, 100, 4};
};

} // namespace FcComms

#endif // End COMMON_CONF_HPP
