#ifndef ADVANCED_NAVIGATION_ANPP_DEVICEINFORMATION_HPP
#define ADVANCED_NAVIGATION_ANPP_DEVICEINFORMATION_HPP

#include <cstdint>

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
}

#endif
