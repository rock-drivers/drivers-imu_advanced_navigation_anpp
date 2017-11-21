#include <imu_advanced_navigation_anpp/Exceptions.hpp>

using namespace imu_advanced_navigation_anpp;

#define ENUM_TO_STRING(name) \
    case name: return #name;

std::string AcknowledgeFailure::resultToString(ACK_RESULTS result)
{
    switch(result)
    {
        ENUM_TO_STRING(ACK_FAILED_PACKET_VALIDATION_CRC);
        ENUM_TO_STRING(ACK_FAILED_PACKET_VALIDATION_SIZE);
        ENUM_TO_STRING(ACK_FAILED_OUT_OF_RANGE);
        ENUM_TO_STRING(ACK_FAILED_SYSTEM_FLASH_FAILURE);
        ENUM_TO_STRING(ACK_FAILED_UNKNOWN_PACKET);
        default:
            return "UNKNOWN";
    }
}

