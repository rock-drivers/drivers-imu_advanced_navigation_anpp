#ifndef ADVANCED_NAVIGATION_ANPP_STATUS_HPP
#define ADVANCED_NAVIGATION_ANPP_STATUS_HPP

#include <cstdint>
#include <imu_advanced_navigation_anpp/Constants.hpp>

namespace imu_advanced_navigation_anpp
{
    struct Status
    {
        uint16_t system_status;

        bool orientation_initialized;
        bool navigation_initialized;
        bool heading_initialized;
        bool utc_initialized;

        GNSS_STATUS gnss_status;
    };
}

#endif

