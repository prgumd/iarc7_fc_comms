#ifndef COMMON_CONF_HPP
#define COMMON_CONF_HPP

////////////////////////////////////////////////////////////////////////////
//
// Configuration Common to all flight controllers.
//
////////////////////////////////////////////////////////////////////////////

namespace FcComms
{
    struct CommonConf
    {
        // In seconds.
        static constexpr const float kFcSensorsUpdatePeriod{0.2};
    };
}

#endif // End COMMON_CONF_HPP
