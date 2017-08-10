#include <iostream>
#include <iomanip>
#include <imu_advanced_navigation_anpp/Driver.hpp>

using namespace std;
using namespace imu_advanced_navigation_anpp;

#define ENUM_TO_STRING(name) \
    case name: return #name;


static const char* SYSTEM_STATE_BITFIELD_NAMES[] = {
    "FAILURE",
    "ACCELEROMETER_FAILURE",
    "GYROSCOPE_FAILURE",
    "MAGNETOMETER_FAILURE",
    "PRESSURE_SENSOR_FAILURE",
    "GNSS_SENSOR_FAILURE",
    "ACCELEROMETER_OVER_RANGE",
    "GYROSCOPE_OVER_RANGE",
    "MAGNETOMETER_OVER_RANGE",
    "PRESSURE_SENSOR_OVER_RANGE",
    "MIN_TEMPERATURE_ALARM",
    "MAX_TEMPERATURE_ALARM",
    "LOW_VOLTAGE_ALARM",
    "HIGH_VOLTAGE_ALARM",
    "GNSS_ANTENNA_DISCONNECTED",
    "DATA_OUTPUT_OVERFLOW_ALARM"
};

std::string systemStateToString(int16_t system_state)
{
    if (!system_state)
        return "OK";

    std::string result;
    for (int i = 0; i < 16; ++i)
    {
        if (system_state & (1 << i))
        {
            if (!result.empty())
                result += ", ";
            result += SYSTEM_STATE_BITFIELD_NAMES[i];
        }
    }
    return result;
}


std::string enumToString(VEHICLE_TYPES type)
{
    switch(type)
    {
        ENUM_TO_STRING(VEHICLE_UNCONSTRAINED);
        ENUM_TO_STRING(VEHICLE_BICYCLE_OR_MOTORCYCLE);
        ENUM_TO_STRING(VEHICLE_CAR);
        ENUM_TO_STRING(VEHICLE_HOVERCRAFT);
        ENUM_TO_STRING(VEHICLE_SUBMARINE);
        ENUM_TO_STRING(VEHICLE_3D_UNDERWATER);
        ENUM_TO_STRING(VEHICLE_FIXED_WING_PLANE);
        ENUM_TO_STRING(VEHICLE_3D_AIRCRAFT);
        ENUM_TO_STRING(VEHICLE_HUMAN);
        ENUM_TO_STRING(VEHICLE_BOAT);
        ENUM_TO_STRING(VEHICLE_LARGE_SHIP);
        ENUM_TO_STRING(VEHICLE_STATIONARY);
        ENUM_TO_STRING(VEHICLE_STUNT_PLANE);
        ENUM_TO_STRING(VEHICLE_RACE_CAR);
        default:
            throw std::invalid_argument("given an invalid vehicle type ID");
    };
}

std::string enumToString(MAGNETIC_CALIBRATION_STATUS status)
{
    switch(status)
    {
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_NOT_COMPLETED)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_2D_COMPLETED)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_3D_COMPLETED)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_CUSTOM_COMPLETED)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_2D_IN_PROGRESS)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_3D_IN_PROGRESS)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_ERROR_2D_EXCESSIVE_ROLL)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_ERROR_2D_EXCESSIVE_PITCH)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_ERROR_SENSOR_OVER_RANGE)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_ERROR_SENSOR_TIME_OUT)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_ERROR_SENSOR_SYSTEM_ERROR)
        ENUM_TO_STRING(MAGNETIC_CALIBRATION_ERROR_SENSOR_INTERFERENCE_ERROR)
        default:
            throw std::invalid_argument("given an invalid magnetic calibration status ID");
    }
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr
            << "Usage: imu_advanced_navigation_anpp_ctl URI COMMAND [args]\n"
            << "Known commands:\n"
            << "  info\n"
            << "  reset-cold\n"
            << "  reset-hot\n"
            << "  reset-factory\n";
        return 1;
    }

    string uri = argv[1];
    string cmd = argv[2];

    Driver driver;
    driver.openURI(uri);

    if (cmd == "info")
    {
        DeviceInformation info = driver.readDeviceInformation();
        std::cout
            << "Device Info:\n"
            << "  SW Version:    " << info.software_version << "\n"
            << "  Device ID/Rev: " << info.device_id << "/" << info.hardware_revision << "\n"
            << "  Serial Nr:     " << setfill('0') << setw(8) << info.serial_number_part0 << info.serial_number_part1 << info.serial_number_part2 << "\n";

        base::Time now = base::Time::now();
        auto time   = driver.readTime();
        auto status = driver.readStatus();
        std::cout
            << "\n"
            << "Local  Time: " << now << "\n"
            << "Device Time: " << time << " (" << (now - time).toMicroseconds() << "us)" << "\n";
        std::cout << "\n"
            << "System Status: " << systemStateToString(status.system_status) << "\n";
        std::cout << "\n"
            << "Filter Status:\n"
            << "  Orientation Initialized: " << (status.orientation_initialized ? "yes" : "no") << "\n"
            << "  Navigation Initialized:  " << (status.navigation_initialized ? "yes" : "no") << "\n"
            << "  Heading Initialized:     " << (status.heading_initialized ? "yes" : "no") << "\n"
            << "  UTC Initialized:         " << (status.utc_initialized ? "yes" : "no") << "\n";

        auto conf = driver.readConfiguration();
        std::cout
            << "\n"
            << "Base Packet Period: " << conf.packet_timer_period.toMicroseconds() << "us\n"
            << "  UTC Sync: " << (conf.utc_synchronization ? "yes" : "no") << "\n"
            << "GNSS Antenna Offset:\n"
            << "  x=" << conf.gnss_antenna_offset.x()
                << " y=" << conf.gnss_antenna_offset.y()
                << " z=" << conf.gnss_antenna_offset.z() << "\n"
            << "Filter Configuration:\n"
            << "  Vehicle Type:         " << enumToString(conf.vehicle_type) << "\n"
            << "  Internal GNSS:        " << (conf.enabled_internal_gnss ? "yes" : "no") << "\n"
            << "  Dual Antenna Heading: " << (conf.enabled_dual_antenna_heading ? "yes" : "no") << "\n"
            << "  Atmospheric Altitude: " << (conf.enabled_atmospheric_altitude ? "yes" : "no") << "\n"
            << "  Velocity Heading:     " << (conf.enabled_velocity_heading ? "yes" : "no") << "\n"
            << "  Reversing Detection:  " << (conf.enabled_reversing_detection ? "yes" : "no") << "\n"
            << "  Motion Analysis:      " << (conf.enabled_motion_analysis ? "yes" : "no") << "\n"
            << "Magnetic Calibration:\n"
            << "  Status: " << enumToString(conf.magnetic_calibration_status) << "\n";

        return 0;
    }
    else if (cmd == "reset-cold")
    {
        driver.reset(RESET_COLD);
    }
    else if (cmd == "reset-hot")
    {
        driver.reset(RESET_HOT);
    }
    else if (cmd == "reset-factory")
    {
        driver.reset(RESET_FACTORY);
    }
    else if (cmd == "gps")
    {
    }
    else
    {
        std::cerr << "Unknown command '" << cmd << "'\n";
        return 1;
    }


    return 0;
}
