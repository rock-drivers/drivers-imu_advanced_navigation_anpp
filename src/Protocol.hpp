#ifndef ADVANCED_NAVIGATION_ANPP_HEADER_HPP
#define ADVANCED_NAVIGATION_ANPP_HEADER_HPP

#include <cstdint>
#include <algorithm>
#include <stdexcept>
#include <type_traits>

namespace advanced_navigation_anpp
{
    /** Implementation of the ANPP protocol itself
     *
     * A iodrivers_base driver that uses this protocol is provided as Driver
     *
     * The general structure is that there is one struct per packet. Thse
     * structs represent the fixed-size part of the packet - some packets have
     * variable sizes. Structs that are meant to be sent to the device have a
     * marshal() method that returns the packet payload as a set of bytes.
     * Structs that are meant to be received have an unmarshal() static method
     * that return the struct from the payload data.
     */
    namespace protocol
    {
        /** Helper function to read a 64 bit value from a byte stream */
        template<typename T, typename InputIterator>
        inline T read16(InputIterator it)
        {
            uint16_t b0 = *it;
            uint16_t b1 = *(++it);
            uint16_t r = b0 | b1 << 8;
            return reinterpret_cast<T const&>(r);
        }

        /** Helper function to read a 64 bit value from a byte stream */
        template<typename T, typename InputIterator>
        inline T read32(InputIterator it)
        {
            uint32_t b0 = *it;
            uint32_t b1 = *(++it);
            uint32_t b2 = *(++it);
            uint32_t b3 = *(++it);
            uint32_t r = b0 | b1 << 8 | b2 << 16 | b3 << 24;
            return reinterpret_cast<T const&>(r);
        }

        /** Helper function to read a 32 bit value from a byte stream */
        template<typename T, typename InputIterator>
        inline T read64(InputIterator it)
        {
            uint64_t b0 = *it;
            uint64_t b1 = *(++it);
            uint64_t b2 = *(++it);
            uint64_t b3 = *(++it);
            uint64_t b4 = *(++it);
            uint64_t b5 = *(++it);
            uint64_t b6 = *(++it);
            uint64_t b7 = *(++it);
            uint64_t r = b0 | b1 << 8 | b2 << 16 | b3 << 24 | b4 << 32 | b5 << 40 | b6 << 48 | b7 << 56;
            return reinterpret_cast<T const&>(r);
        }

        /** ID of packets in the protocol */
        enum PACKET_IDS
        {
            ID_ACK                                = 0,
            ID_REQUEST                            = 1,
            ID_BOOT_MODE                          = 2,
            ID_DEVICE_INFO                        = 3,
            ID_RESTORE_FACTORY_SETTINGS           = 4,
            ID_RESET                              = 5,

            ID_SYSTEM_STATE                       = 20,
            ID_UNIX_TIME                          = 21,
            ID_STATUS                             = 23,
            ID_POSITION_STD_DEV                   = 24,
            ID_VELOCITY_STD_DEV                   = 25,
            ID_QUATERNION_STD_DEV                 = 27,
            ID_RAW_SENSORS                        = 28,
            ID_RAW_GNSS                           = 29,
            ID_SATELLITES                         = 30,
            ID_POSITION_GEODETIC                  = 31,
            ID_VELOCITY_NED                       = 35,
            ID_VELOCITY_BODY                      = 36,
            ID_ACCELERATION_BODY                  = 37,
            ID_ORIENTATION_QUATERNION             = 40,
            ID_VELOCITY_ANGULAR                   = 42,
            ID_ACCELERATION_ANGULAR               = 43,
            ID_LOCAL_MAGNETIC_FIELD               = 50,
            ID_GEOID_HEIGHT                       = 54,

            ID_PACKET_TIMER_PERIOD                = 180,
            ID_PACKETS_PERIOD                     = 181,
            ID_BAUD_RATES                         = 182,
            ID_INSTALLATION_ALIGNMENT             = 185,
            ID_FILTER_BASIC_OPTIONS               = 186,
            ID_FILTER_ADVANCED_OPTIONS            = 187,
            ID_MAGNETIC_CALIBRATION_VALUES        = 189,
            ID_MAGNETIC_CALIBRATION_CONFIGURATION = 190,
            ID_MAGNETIC_CALIBRATION_STATUS        = 191
        };

        /** Generic packet header
         *
         * The data in the packet is stored in little endian byte order
         */
        struct Header
        {
            /** Longitudinal redundancy check for the header
             *
             * <code>
             * LRC = ((packet_id + packet_length + crc[0] + crc[1])^0xFF) + 1
             * </code>
             */
            uint8_t header_checksum;
            uint8_t packet_id;
            uint8_t payload_length;
            /** Least significant byte of a crc-CCITT with starting value 0xFFFF
             * calculated over the packet data only
             */
            uint8_t payload_checksum_lsb;
            /** Most significant byte of a crc-CCITT with starting value 0xFFFF
             * calculated over the packet data only
             */
            uint8_t payload_checksum_msb;

            /** Construct an uninitialized header
             *
             * The header and payload checksums won't validate
             */
            Header();

            /** Initialize a header by filling all the fields based on data in
             * the packet
             */
            Header(uint8_t packet_id, uint8_t const* begin, uint8_t const* end);

            /** Check if the data in the header validates the header checksum
             */
            bool isValid() const;

            /** Check if the data in the header validates the packet data
             */
            bool isPacketValid(uint8_t const* begin, uint8_t const* end) const;

            /** Compute the checksum of the header
             */
            uint8_t computeHeaderChecksum() const;
        } __attribute__((packed));

        /** The maximum packet size, header included
         */
        static const int MAX_PACKET_SIZE = 256 + sizeof(Header);

        /** Compute the CRC as expected by the protocol
         */
        uint16_t crc(uint8_t const* begin, uint8_t const* end);

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

        /** Acknowledgment packet */
        struct Acknowledge
        {
            uint8_t acked_packet_id;
            uint8_t acked_payload_checksum_lsb;
            uint8_t acked_payload_checksum_msb;
            uint8_t result;

            /** Tests whether this acknowledgment matches the given packet
             * header
             */
            bool isMatching(Header const& header) const;

            /** True if this indicates a success */
            bool isSuccess() const;

            /** True if this acknowledge is a failure-to-validate-packet error
             */
            bool isPacketValidationFailure() const;

            /** True if this acknowledge indicates a protocol error on the
             * driver side
             *
             * Packet validation failures are not reported as they might be a
             * communication error as well. Check against sent packets with
             * isMatching to make the difference
             */
            bool isProtocolError() const;

            /** True if this acknowledge indicates a system error on the IMU
             * side
             */
            bool isSystemError() const;

            /** True if this ack indicates that the system is not ready */
            bool isNotReady() const;

            /** Initializes an Acknowledge from raw data
             */
            template<typename RandomInputIterator>
            static Acknowledge unmarshal(RandomInputIterator begin, RandomInputIterator end)
            {
                if (end - begin != sizeof(Acknowledge))
                    throw std::length_error("Acknowledge::unmarshal buffer size not the expected size");
                return Acknowledge{ begin[0], begin[1], begin[2], begin[3] };
            }
        } __attribute__((packed));

        /** Request packet
         *
         * This is a variable-length packet made only of the IDs of the
         * requested packets. There is no static part, hence the empty struct
         */
        struct Request
        {
            template<typename OutputIterator, typename InputIterator>
            OutputIterator marshal(OutputIterator out, InputIterator begin, InputIterator end)
            {
                return std::copy(begin, end, out);
            }

        } __attribute__((packed));

        /** Boot modes */
        enum BOOT_MODES
        {
            BOOT_TO_BOOTLOADER,
            BOOT_TO_PROGRAM
        };

        /** Boot mode packet */
        struct BootMode
        {
            uint8_t boot_mode;

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                *out = boot_mode;
                return out + 1;
            }

            template<typename RandomInputIterator>
            static BootMode unmarshal(RandomInputIterator begin, RandomInputIterator end)
            {
                if (end - begin != sizeof(BootMode))
                    throw std::length_error("BootMode::unmarshal: buffer size is not expected size");
                return BootMode{*begin};
            }
        } __attribute__((packed));

        /** Device information */
        struct DeviceInformation
        {
            uint32_t software_version;
            uint32_t device_id;
            uint32_t hardware_revision;
            uint32_t serial_number_part0;
            uint32_t serial_number_part1;
            uint32_t serial_number_part2;

            template<typename RandomInputIterator>
            static DeviceInformation unmarshal(RandomInputIterator begin, RandomInputIterator end)
            {
                if (end - begin != sizeof(DeviceInformation))
                    throw std::length_error("DeviceInformation::unmarshal: buffer size is not expected size");

                DeviceInformation info;
                info.software_version    = read32<uint32_t>(begin);
                info.device_id           = read32<uint32_t>(begin + 4);
                info.hardware_revision   = read32<uint32_t>(begin + 8);
                info.serial_number_part0 = read32<uint32_t>(begin + 12);
                info.serial_number_part1 = read32<uint32_t>(begin + 16);
                info.serial_number_part2 = read32<uint32_t>(begin + 20);
                return info;
            }
        } __attribute__((packed));

        struct RestoreFactorySettings
        {
            uint8_t verification_sequence[4] = { 0x1C, 0x9E, 0x42, 0x85 };

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                std::copy(verification_sequence, verification_sequence + 4, out);
                return out + 4;
            }
        } __attribute__((packed));

        struct HotStartReset
        {
            uint8_t verification_sequence[4] = { 0x7E, 0x7A, 0x05, 0x21 };

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                std::copy(verification_sequence, verification_sequence + 4, out);
                return out + 4;
            }
        } __attribute__((packed));

        struct ColdStartReset
        {
            uint8_t verification_sequence[4] = { 0xB7, 0x38, 0x5D, 0x9A };

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                std::copy(verification_sequence, verification_sequence + 4, out);
                return out + 4;
            }
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

        enum FILTER_STATUS
        {
            FILTER_ORIENTATION_INITIALIZED      = 0x0001,
            FILTER_NAVIGATION_INITIALIZED       = 0x0002,
            FILTER_HEADING_INITIALIZED          = 0x0004,
            FILTER_UTC_INITIALIZED              = 0x0008,

            FILTER_GNSS_STATUS_MASK             = 0x0070,
            FILTER_GNSS_NO_FIX                  = 0x0000,
            FILTER_GNSS_2D                      = 0x0010,
            FILTER_GNSS_3D                      = 0x0020,
            FILTER_GNSS_SBAS                    = 0x0030,
            FILTER_GNSS_DGPS                    = 0x0040,
            FILTER_GNSS_OMNISTAR                = 0x0050,
            FILTER_GNSS_RTK_FLOAT               = 0x0060,
            FILTER_GNSS_RTK_FIXED               = 0x0070,

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

        struct SystemState
        {
            /** Bitfield of SYSTEM_STATUS */
            uint16_t system_status;
            /** Bitfield of FILTER_STATUS */
            uint16_t filter_status;
            uint32_t unix_time_seconds;
            uint32_t unix_time_microseconds;
            double   lat_lon_z[3];
            float   velocity_ned[3];
            float   body_acceleration_xyz[3];
            float   g;
            float   rpy[3];
            float   angular_velocity[3];
            float   lat_lon_z_stddev[3];

            template<typename InputIterator>
            static SystemState unmarshal(InputIterator begin, InputIterator end)
            {
                SystemState state;
                state.system_status            = read16<uint16_t>(begin);
                state.filter_status            = read16<uint16_t>(begin + 2);
                state.unix_time_seconds        = read32<uint32_t>(begin + 4);
                state.unix_time_microseconds   = read32<uint32_t>(begin + 8);
                state.lat_lon_z[0]             = read64<double>(begin + 12);
                state.lat_lon_z[1]             = read64<double>(begin + 20);
                state.lat_lon_z[2]             = read64<double>(begin + 28);
                state.velocity_ned[0]          = read32<float>(begin + 36);
                state.velocity_ned[1]          = read32<float>(begin + 40);
                state.velocity_ned[2]          = read32<float>(begin + 44);
                state.body_acceleration_xyz[0] = read32<float>(begin + 48);
                state.body_acceleration_xyz[1] = read32<float>(begin + 52);
                state.body_acceleration_xyz[2] = read32<float>(begin + 56);
                state.g                        = read32<float>(begin + 60);
                state.rpy[0]                   = read32<float>(begin + 64);
                state.rpy[1]                   = read32<float>(begin + 68);
                state.rpy[2]                   = read32<float>(begin + 72);
                state.angular_velocity[0]      = read32<float>(begin + 76);
                state.angular_velocity[1]      = read32<float>(begin + 80);
                state.angular_velocity[2]      = read32<float>(begin + 84);
                state.lat_lon_z_stddev[0]      = read32<float>(begin + 88);
                state.lat_lon_z_stddev[1]      = read32<float>(begin + 92);
                state.lat_lon_z_stddev[2]      = read32<float>(begin + 96);
                return state;
            }
        } __attribute__((packed));
    }
}

#endif
