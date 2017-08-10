#ifndef ADVANCED_NAVIGATION_ANPP_TYPES_HPP
#define ADVANCED_NAVIGATION_ANPP_TYPES_HPP

#include <base/Time.hpp>
#include <base/Eigen.hpp>

namespace imu_advanced_navigation_anpp
{
    struct DeviceInformation
    {
        uint32_t software_version;
        uint32_t device_id;
        uint32_t hardware_revision;
        uint32_t serial_number_part0;
        uint32_t serial_number_part1;
        uint32_t serial_number_part2;
    } __attribute__((packed));

    enum SYSTEM_STATUS
    {
        SYSTEM_FAILURE                    = 0x0001,
        SYSTEM_ACCELEROMETER_FAILURE      = 0x0002,
        SYSTEM_GYROSCOPE_FAILURE          = 0x0004,
        SYSTEM_MAGNETOMETER_FAILURE       = 0x0008,
        SYSTEM_PRESSURE_SENSOR_FAILURE    = 0x0010,
        SYSTEM_GNSS_SENSOR_FAILURE        = 0x0020,
        SYSTEM_ACCELEROMETER_OVER_RANGE   = 0x0040,
        SYSTEM_GYROSCOPE_OVER_RANGE       = 0x0080,
        SYSTEM_MAGNETOMETER_OVER_RANGE    = 0x0100,
        SYSTEM_PRESSURE_SENSOR_OVER_RANGE = 0x0200,
        SYSTEM_MIN_TEMPERATURE_ALARM      = 0x0400,
        SYSTEM_MAX_TEMPERATURE_ALARM      = 0x0800,
        SYSTEM_LOW_VOLTAGE_ALARM          = 0x1000,
        SYSTEM_HIGH_VOLTAGE_ALARM         = 0x2000,
        SYSTEM_GNSS_ANTENNA_DISCONNECTED  = 0x4000,
        SYSTEM_DATA_OUTPUT_OVERFLOW_ALARM = 0x8000
    };

    enum GNSS_STATUS
    {
        GNSS_NO_FIX                  = 0x0000,
        GNSS_2D                      = 0x0010,
        GNSS_3D                      = 0x0020,
        GNSS_SBAS                    = 0x0030,
        GNSS_DGPS                    = 0x0040,
        GNSS_OMNISTAR                = 0x0050,
        GNSS_RTK_FLOAT               = 0x0060,
        GNSS_RTK_FIXED               = 0x0070,
    };

    enum FILTER_STATUS
    {
        FILTER_ORIENTATION_INITIALIZED      = 0x0001,
        FILTER_NAVIGATION_INITIALIZED       = 0x0002,
        FILTER_HEADING_INITIALIZED          = 0x0004,
        FILTER_UTC_INITIALIZED              = 0x0008,

        FILTER_GNSS_FIX_STATUS_MASK         = 0x0070,

        FILTER_EVENT_1                      = 0x0080,
        FILTER_EVENT_2                      = 0x0100,
        FILTER_INTERNAL_GNSS_ENABLED        = 0x0200,
        FILTER_MAGNETIC_HEADING_ENABLED     = 0x0400,
        FILTER_VELOCITY_HEADING_ENABLED     = 0x0800,
        FILTER_ATMOSPHERIC_ALTITUDE_ENABLED = 0x1000,
        FILTER_EXTERNAL_POSITION_ACTIVE     = 0x2000,
        FILTER_EXTERNAL_VELOCITY_ACTIVE     = 0x4000,
        FILTER_EXTERNAL_HEADING_ACTIVE      = 0x8000,
    };

    enum VEHICLE_TYPES
    {
        VEHICLE_UNCONSTRAINED         = 0,
        VEHICLE_BICYCLE_OR_MOTORCYCLE = 1,
        VEHICLE_CAR                   = 2,
        VEHICLE_HOVERCRAFT            = 3,
        VEHICLE_SUBMARINE             = 4,
        VEHICLE_3D_UNDERWATER         = 5,
        VEHICLE_FIXED_WING_PLANE      = 6,
        VEHICLE_3D_AIRCRAFT           = 7,
        VEHICLE_HUMAN                 = 8,
        VEHICLE_BOAT                  = 9,
        VEHICLE_LARGE_SHIP            = 10,
        VEHICLE_STATIONARY            = 11,
        VEHICLE_STUNT_PLANE           = 12,
        VEHICLE_RACE_CAR              = 13
    };

    enum MAGNETIC_CALIBRATION_STATUS
    {
        MAGNETIC_CALIBRATION_NOT_COMPLETED,
        MAGNETIC_CALIBRATION_2D_COMPLETED,
        MAGNETIC_CALIBRATION_3D_COMPLETED,
        MAGNETIC_CALIBRATION_CUSTOM_COMPLETED,
        MAGNETIC_CALIBRATION_2D_IN_PROGRESS,
        MAGNETIC_CALIBRATION_3D_IN_PROGRESS,
        MAGNETIC_CALIBRATION_ERROR_2D_EXCESSIVE_ROLL,
        MAGNETIC_CALIBRATION_ERROR_2D_EXCESSIVE_PITCH,
        MAGNETIC_CALIBRATION_ERROR_SENSOR_OVER_RANGE,
        MAGNETIC_CALIBRATION_ERROR_SENSOR_TIME_OUT,
        MAGNETIC_CALIBRATION_ERROR_SENSOR_SYSTEM_ERROR,
        MAGNETIC_CALIBRATION_ERROR_SENSOR_INTERFERENCE_ERROR
    };

    struct Status
    {
        uint16_t system_status;

        bool orientation_initialized;
        bool navigation_initialized;
        bool heading_initialized;
        bool utc_initialized;

        GNSS_STATUS gnss_status;
    };

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

    struct CurrentConfiguration : public Configuration
    {
        MAGNETIC_CALIBRATION_STATUS magnetic_calibration_status;
        base::Vector3d hard_iron_bias;
        base::Matrix3d soft_iron_transformation;
    };
}

#endif

