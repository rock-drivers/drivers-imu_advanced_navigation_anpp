#ifndef ADVANCED_NAVIGATION_ANPP_STATUS_HPP
#define ADVANCED_NAVIGATION_ANPP_STATUS_HPP

#include <cstdint>
#include <base/Time.hpp>
#include <imu_advanced_navigation_anpp/Constants.hpp>
#include <imu_advanced_navigation_anpp/NorthSeekingInitializationStatus.hpp>

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

        /** The filter's status as a bitfield of FILTER_STATUS
         *
         * @meta bitfield /imu_advanced_navigation_anpp/FILTER_STATUS
         */
        uint16_t filter_status;

        /** The status of the GNSS solution
         */
        GNSS_STATUS gnss_solution_status;

        /** The status of non-positioning GNSS resolution (doppler, heading,
         * ...)
         *
         * @meta bitfield /imu_advanced_navigation_anpp/GNSS_EXTRA_STATUS
         */
        uint16_t gnss_extra_status;

        /** North seeking process status
         *
         * If the north seeking status message is enabled, it is reported here
         *
         * @meta bitfield /imu_advanced_navigation_anpp/NORTH_SEEKING_INITIALIZATION_FLAGS
         */
        NorthSeekingInitializationStatus north_seeking;

        Status()
            : gnss_extra_status(0) {}

        bool isOrientationInitialized() const
        {
            return filter_status & FILTER_ORIENTATION_INITIALIZED;
        }

        bool isHeadingInitialized() const
        {
            return filter_status & FILTER_HEADING_INITIALIZED;
        }

        bool isNavigationInitialized() const
        {
            return filter_status & FILTER_NAVIGATION_INITIALIZED;
        }

        bool isUTCInitialized() const
        {
            return filter_status & FILTER_UTC_INITIALIZED;
        }
    };
}

#endif

