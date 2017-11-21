#ifndef ADVANCED_NAVIGATION_ANPP_CURRENT_CONFIGURATION_HPP
#define ADVANCED_NAVIGATION_ANPP_CURRENT_CONFIGURATION_HPP

#include <imu_advanced_navigation_anpp/Configuration.hpp>

namespace imu_advanced_navigation_anpp
{
    struct CurrentConfiguration : public Configuration
    {
        MAGNETIC_CALIBRATION_STATUS magnetic_calibration_status;
        base::Vector3d hard_iron_bias;
        base::Matrix3d soft_iron_transformation;
    };
}

#endif

