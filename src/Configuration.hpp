#ifndef ADVANCED_NAVIGATION_ANPP_CONFIGURATION_HPP
#define ADVANCED_NAVIGATION_ANPP_CONFIGURATION_HPP

#include <imu_advanced_navigation_anpp/Constants.hpp>
#include <base/Time.hpp>
#include <base/Eigen.hpp>

namespace imu_advanced_navigation_anpp
{
    struct Configuration
    {
        bool utc_synchronization;
        base::Time packet_timer_period;

        base::Vector3d gnss_antenna_offset;

        VEHICLE_TYPES vehicle_type;
        bool enabled_internal_gnss;
        bool enabled_atmospheric_altitude;
        bool enabled_velocity_heading;
        bool enabled_reversing_detection;
        bool enabled_motion_analysis;
    };
}

#endif


