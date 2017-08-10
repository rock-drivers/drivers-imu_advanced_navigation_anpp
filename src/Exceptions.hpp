#ifndef ADVANCED_NAVIGATION_ANPP_EXCEPTIONS_HPP
#define ADVANCED_NAVIGATION_ANPP_EXCEPTIONS_HPP

#include <stdexcept>

namespace imu_advanced_navigation_anpp
{
    /** Acknowledge result codes */
    enum ACK_RESULTS
    {
        ACK_SUCCESS                       = 0,
        ACK_FAILED_PACKET_VALIDATION_CRC  = 1,
        ACK_FAILED_PACKET_VALIDATION_SIZE = 2,
        ACK_FAILED_OUT_OF_RANGE           = 3,
        ACK_FAILED_SYSTEM_FLASH_FAILURE   = 4,
        ACK_FAILED_SYSTEM_NOT_READY       = 5,
        ACK_FAILED_UNKNOWN_PACKET         = 6
    };

    struct AcknowledgeFailure : public std::runtime_error
    {
        uint8_t const packet_id;
        ACK_RESULTS const result;

        AcknowledgeFailure(uint8_t packet_id, ACK_RESULTS result)
            : std::runtime_error("received failure after having sent a configuration change")
            , packet_id(packet_id)
            , result(result) {}
    };
}

#endif

