#ifndef ADVANCED_NAVIGATION_ANPP_NORTH_SEEKING_INITIALIZATION_STATUS_HPP
#define ADVANCED_NAVIGATION_ANPP_NORTH_SEEKING_INITIALIZATION_STATUS_HPP

#include <base/Time.hpp>
#include <base/Angle.hpp>
#include <base/Float.hpp>

namespace imu_advanced_navigation_anpp
{
    struct NorthSeekingInitializationStatus
    {
        base::Time time;

        /** The process status
         *
         * @meta bitfield /imu_advanced_navigation_anpp/NORTH_SEEKING_INITIALIZATION_FLAGS
         */
        uint16_t flags;

        /** The process progress in each quadrant, from 0 to 1
         */
        float progress[4];

        /** The current rotation angle
         */
        base::Angle current_rotation_angle;

        base::Vector3d gyroscope_bias;
        float gyroscope_bias_solution_error;

        NorthSeekingInitializationStatus()
            : flags(0)
            , progress { base::unknown<float>(), base::unknown<float>(), base::unknown<float>(), base::unknown<float>() }
            , gyroscope_bias(base::unknown<double>(), base::unknown<double>(), base::unknown<double>())
            , gyroscope_bias_solution_error(base::unknown<float>()) {}
    };
}

#endif

