#ifndef ADVANCED_NAVIGATION_ANPP_STATUS_HPP
#define ADVANCED_NAVIGATION_ANPP_STATUS_HPP

#include <cstdint>
#include <base/Time.hpp>
#include <imu_advanced_navigation_anpp/Constants.hpp>

namespace imu_advanced_navigation_anpp
{
    struct Status
    {
        /** The status timestamp
         */
        base::Time time;

        /** The system status as a bitfield of SYSTEM_STATUS
         *
         * @meta bitfield /imu_advanced_navigation_anpp/SYSTEM_STATUS
         */
        uint16_t system_status;

        bool orientation_initialized;
        bool navigation_initialized;
        bool heading_initialized;
        bool utc_initialized;

        GNSS_STATUS gnss_status;
    };
}

#endif

