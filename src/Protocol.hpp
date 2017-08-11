#ifndef ADVANCED_NAVIGATION_ANPP_HEADER_HPP
#define ADVANCED_NAVIGATION_ANPP_HEADER_HPP

#include <cstdint>
#include <algorithm>
#include <stdexcept>
#include <cstring>
#include <map>
#include <type_traits>
#include <base/Timeout.hpp>
#include <iodrivers_base/Exceptions.hpp>

#include <imu_advanced_navigation_anpp/Constants.hpp>
#include <imu_advanced_navigation_anpp/DeviceInformation.hpp>
#include <imu_advanced_navigation_anpp/Exceptions.hpp>

namespace imu_advanced_navigation_anpp
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
            static_assert(sizeof(T) == 2, "return type is not a 2-byte data type");
            uint16_t b0 = *it;
            uint16_t b1 = *(++it);
            uint16_t r = b0 | b1 << 8;
            return reinterpret_cast<T const&>(r);
        }

        /** Helper function to read a 64 bit value from a byte stream */
        template<typename T, typename InputIterator>
        inline T read32(InputIterator it)
        {
            static_assert(sizeof(T) == 4, "return type is not a 4-byte data type");
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
            static_assert(sizeof(T) == 8, "return type is not a 8-byte data type");
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

        /** Helper function to write a 16 bit value into a byte stream */
        template<typename T, typename Out>
        inline void write16(Out out, T sample)
        {
            static_assert(sizeof(T) == 2, "sample is not a 2-byte data type");
            uint16_t value = reinterpret_cast<uint16_t&>(sample);
            out[0] = value & 0xFF;
            out[1] = (value >> 8) & 0xFF;
        }

        /** Helper function to read a 64 bit value from a byte stream */
        template<typename T, typename Out>
        inline void write32(Out out, T sample)
        {
            static_assert(sizeof(T) == 4, "sample is not a 4-byte data type");
            uint32_t value = reinterpret_cast<uint32_t&>(sample);
            out[0] = (value >> 0) & 0xFF;
            out[1] = (value >> 8) & 0xFF;
            out[2] = (value >> 16) & 0xFF;
            out[3] = (value >> 24) & 0xFF;
        }

        /** Helper function to read a 32 bit value from a byte stream */
        template<typename T, typename Out>
        inline void write64(Out out, T sample)
        {
            static_assert(sizeof(T) == 8, "sample is not a 8-byte data type");
            uint64_t value = reinterpret_cast<uint64_t&>(sample);
            out[1] = (value >> 8) & 0xFF;
            out[2] = (value >> 16) & 0xFF;
            out[3] = (value >> 24) & 0xFF;
            out[4] = (value >> 32) & 0xFF;
            out[5] = (value >> 40) & 0xFF;
            out[6] = (value >> 48) & 0xFF;
            out[7] = (value >> 56) & 0xFF;
        }

        static constexpr int PACKET_ID_COUNT = 256;

        /** Generic packet header
         *
         * The data in the packet is stored in little endian byte order
         */
        struct Header
        {
            static constexpr int SIZE = 5;

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

            /** The length of the whole packet (header + payload) */
            size_t getPacketLength() const;

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
        static constexpr int MAX_PACKET_SIZE = 256 + sizeof(Header);

        /** Compute the CRC as expected by the protocol
         */
        uint16_t crc(uint8_t const* begin, uint8_t const* end);

        /** Acknowledgment packet */
        struct Acknowledge
        {
            static constexpr uint8_t ID = 0;
            static constexpr int SIZE = 4;

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
            static constexpr uint8_t ID = 1;
            static constexpr int MIN_SIZE = 0;

            template<typename OutputIterator, typename InputIterator>
            OutputIterator marshal(OutputIterator out, InputIterator begin, InputIterator end) const
            {
                return std::copy(begin, end, out);
            }

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out, uint8_t packet_id) const
            {
                return std::copy(&packet_id, &packet_id + 1, out);
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
            static constexpr uint8_t ID = 2;
            static constexpr int SIZE = 1;

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
        struct DeviceInformation : public imu_advanced_navigation_anpp::DeviceInformation
        {
            static constexpr uint8_t ID = 3;
            static constexpr int SIZE = 24;

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
            static constexpr uint8_t ID = 4;
            static constexpr int SIZE = 4;

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
            static constexpr uint8_t ID = 5;
            static constexpr int SIZE = 4;

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
            static constexpr uint8_t ID = 5;
            static constexpr int SIZE = 4;

            uint8_t verification_sequence[4] = { 0xB7, 0x38, 0x5D, 0x9A };

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                std::copy(verification_sequence, verification_sequence + 4, out);
                return out + 4;
            }
        } __attribute__((packed));

        struct SystemState
        {
            static constexpr uint8_t ID = 20;
            static constexpr int SIZE = 100;

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
                if (end - begin != sizeof(SystemState))
                    throw std::length_error("SystemState::unmarshal buffer size not the expected size");

                SystemState state;
                state.system_status            = read16<uint16_t>(begin);
                state.filter_status            = read16<uint16_t>(begin + 2);
                state.unix_time_seconds        = read32<uint32_t>(begin + 4);
                state.unix_time_microseconds   = read32<uint32_t>(begin + 8);

                state.g                        = read32<float>(begin + 60);

                for (int i = 0; i < 3; ++i)
                {
                    state.lat_lon_z[i]             = read64<double>(begin + 12 + 8 * i);
                    state.velocity_ned[i]          = read32<float>(begin + 36 + 4 * i);
                    state.body_acceleration_xyz[i] = read32<float>(begin + 48 + 4 * i);
                    state.rpy[i]                   = read32<float>(begin + 64 + 4 * i);
                    state.angular_velocity[i]      = read32<float>(begin + 76 + 4 * i);
                    state.lat_lon_z_stddev[i]      = read32<float>(begin + 88 + 4 * i);
                }
                return state;
            }
        } __attribute__((packed));

        struct UnixTime
        {
            static constexpr uint8_t ID = 21;
            static constexpr int SIZE = 8;

            uint32_t seconds;
            uint32_t microseconds;

            template<typename InputIterator>
            static UnixTime unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != sizeof(UnixTime))
                    throw std::length_error("SystemState::unmarshal buffer size not the expected size");
                return UnixTime{read32<uint32_t>(begin), read32<uint32_t>(begin + 4)};
            }
        } __attribute__((packed));

        struct Status
        {
            static constexpr uint8_t ID = 23;
            static constexpr int SIZE = 4;

            /** Bitfield of SYSTEM_STATUS */
            uint16_t system_status;
            /** Bitfield of FILTER_STATUS */
            uint16_t filter_status;

            template<typename InputIterator>
            static Status unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != sizeof(Status))
                    throw std::length_error("SystemState::unmarshal buffer size not the expected size");
                return Status{read16<uint16_t>(begin), read16<uint16_t>(begin + 2)};
            }
        } __attribute__((packed));

        struct GeodeticPositionStandardDeviation
        {
            static constexpr uint8_t ID = 24;
            static constexpr int SIZE = 12;

            float   lat_lon_z_stddev[3];

            template<typename InputIterator>
            static GeodeticPositionStandardDeviation unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != sizeof(GeodeticPositionStandardDeviation))
                    throw std::length_error("SystemState::unmarshal buffer size not the expected size");
                return GeodeticPositionStandardDeviation {
                    read32<float>(begin + 0),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8)
                };
            }
        } __attribute__((packed));

        struct NEDVelocityStandardDeviation
        {
            static constexpr uint8_t ID = 25;
            static constexpr int SIZE = 12;

            float ned[3];

            template<typename InputIterator>
            static NEDVelocityStandardDeviation unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != sizeof(NEDVelocityStandardDeviation))
                    throw std::length_error("NEDVelocityStandardDeviation::unmarshal buffer size not the expected size");
                return NEDVelocityStandardDeviation {
                    read32<float>(begin + 0),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8)
                };
            }
        } __attribute__((packed));

        struct EulerOrientationStandardDeviation
        {
            static constexpr uint8_t ID = 26;
            static constexpr int SIZE = 12;

            float rpy[3];

            template<typename InputIterator>
            static EulerOrientationStandardDeviation unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != sizeof(EulerOrientationStandardDeviation))
                    throw std::length_error("EulerOrientationStandardDeviation::unmarshal buffer size not the expected size");
                return EulerOrientationStandardDeviation {
                    read32<float>(begin + 0),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8)
                };
            }
        } __attribute__((packed));

        struct RawSensors
        {
            static constexpr uint8_t ID = 28;
            static constexpr int SIZE = 48;

            float accelerometers_xyz[3];
            float gyroscopes_xyz[3];
            float magnetometers_xyz[3];
            float imu_temperature_C;
            float pressure;
            float pressure_temperature_C;

            template<typename InputIterator>
            static RawSensors unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != sizeof(RawSensors))
                    throw std::length_error("RawSensors::unmarshal buffer size not the expected size");

                RawSensors out;
                for (int i = 0; i < 3; ++i)
                {
                    out.accelerometers_xyz[i] = read32<float>(begin + 0 + 4 * i);
                    out.gyroscopes_xyz[i]      = read32<float>(begin + 12 + 4 * i);
                    out.magnetometers_xyz[i]   = read32<float>(begin + 24 + 4 * i);
                }

                out.imu_temperature_C = read32<float>(begin + 36);
                out.pressure = read32<float>(begin + 40);
                out.pressure_temperature_C = read32<float>(begin + 44);
                return out;
            }
        } __attribute__((packed));

        struct RawGNSS
        {
            static constexpr uint8_t ID   = 29;
            static constexpr int SIZE = 74;

            uint32_t unix_time_seconds;
            uint32_t unix_time_microseconds;
            double   lat_lon_z[3];
            float    velocity_ned[3];
            float    lat_lon_z_stddev[3];
            float    pitch;
            float    yaw;
            float    pitch_stddev;
            float    yaw_stddev;
            /** Bitfield described by RAW_GNSS_STATUS */
            uint16_t status;

            template<typename InputIterator>
            static RawGNSS unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("RawGNSS::unmarshal buffer size not the expected size");

                RawGNSS out;
                out.unix_time_seconds = read32<uint32_t>(begin + 0);
                out.unix_time_microseconds = read32<uint32_t>(begin + 4);
                for (int i = 0; i < 3; ++i)
                {
                    out.lat_lon_z[i] = read64<double>(begin + 8 + 8 * i);
                    out.velocity_ned[i] = read32<float>(begin + 32 + 4 * i);
                    out.lat_lon_z_stddev[i] = read32<float>(begin + 44 + 4 * i);
                }

                out.pitch = read32<float>(begin + 56);
                out.yaw = read32<float>(begin + 60);
                out.pitch_stddev = read32<float>(begin + 64);
                out.yaw_stddev = read32<float>(begin + 68);
                out.status = read16<uint16_t>(begin + 72);
                return out;
            }
        } __attribute__((packed));

        enum RAW_GNSS_STATUS
        {
            RAW_GNSS_FIX_STATUS_MASK                = 0x07,
            RAW_GNSS_NO_FIX                         = 0x00,
            RAW_GNSS_2D                             = 0x01,
            RAW_GNSS_3D                             = 0x02,
            RAW_GNSS_SBAS                           = 0x03,
            RAW_GNSS_DGPS                           = 0x04,
            RAW_GNSS_OMNISTAR                       = 0x05,
            RAW_GNSS_RTK_FLOAT                      = 0x06,
            RAW_GNSS_RTK_FIXED                      = 0x07,

            RAW_GNSS_HAS_DOPPLER_VELOCITY           = 0x08,
            RAW_GNSS_HAS_TIME                       = 0x10,
            RAW_GNSS_EXTERNAL                       = 0x20,
            RAW_GNSS_HAS_TILT                       = 0x40,
            RAW_GNSS_HAS_HEADING                    = 0x80,
            RAW_GNSS_HAS_FLOATING_AMBIGUITY_HEADING = 0x100
        };

        struct Satellites
        {
            static constexpr uint8_t ID   = 30;
            static constexpr int SIZE = 13;

            float hdop;
            float vdop;
            uint8_t gps_satellite_count;
            uint8_t glonass_satellite_count;
            uint8_t beidou_satellite_count;
            uint8_t galileo_satellite_count;
            uint8_t sbas_satellite_count;

            template<typename InputIterator>
            static Satellites unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("Satellites::unmarshal buffer size not the expected size");

                return Satellites {
                    read32<float>(begin + 0),
                    read32<float>(begin + 4),
                    begin[8],
                    begin[9],
                    begin[10],
                    begin[11],
                    begin[12]
                };
            }
        } __attribute__((packed));

        enum SATELLITE_SYSTEM
        {
            SATELLITE_SYSTEM_UNKNOWN,
            SATELLITE_SYSTEM_GPS,
            SATELLITE_SYSTEM_GLONASS,
            SATELLITE_SYSTEM_BEIDOU,
            SATELLITE_SYSTEM_GALILEO,
            SATELLITE_SYSTEM_SBAS,
            SATELLITE_SYSTEM_QZSS,
            SATELLITE_SYSTEM_STARFIRE,
            SATELLITE_SYSTEM_OMNISTAR
        };

        /** Bitfield values to represent supported frequencies */
        enum SATELLITE_FREQUENCIES
        {
            SATELLITE_FREQUENCY_L1CA = 0x01,
            SATELLITE_FREQUENCY_L1C  = 0x02,
            SATELLITE_FREQUENCY_L1P  = 0x04,
            SATELLITE_FREQUENCY_L1M  = 0x08,
            SATELLITE_FREQUENCY_L2C  = 0x10,
            SATELLITE_FREQUENCY_L2P  = 0x20,
            SATELLITE_FREQUENCY_L2M  = 0x40,
            SATELLITE_FREQUENCY_L5   = 0x80
        };

        /** Satellite info as returned by DetailedSatellites */
        struct SatelliteInfo
        {
            static constexpr int SIZE = 7;

            /** The satellite system as represented by SATELLITE_SYSTEM */
            uint8_t system;
            /** The satellite ID number */
            uint8_t prn;
            /** Satellite frequencies
             *
             * Bitfield represented by the SATELLITE_FREQUENCIES enum
             */
            uint8_t frequencies;
            /** Elevation in degrees */
            uint8_t elevation;
            /** Azimuth in degrees */
            uint16_t azimuth;
            /** Signal to noise ratio in dB */
            uint8_t snr;

            template<typename InputIterator>
            static SatelliteInfo unmarshal(InputIterator begin, InputIterator end)
            {
                return SatelliteInfo{
                    begin[0], begin[1], begin[2], begin[3],
                    read16<uint16_t>(begin + 4), begin[6]
                };
            }
        } __attribute__((packed));

        struct DetailedSatellites
        {
            static constexpr uint8_t ID = 31;
            static constexpr int MIN_SIZE = 0;

            template<typename InputIterator>
            static void unmarshal(InputIterator begin, InputIterator end, std::vector<SatelliteInfo>& info)
            {
                if ((end - begin) % SatelliteInfo::SIZE != 0)
                    throw std::length_error("Satellites::unmarshal buffer is not a multiple of the SatelliteInfo size");

                for (; begin != end; begin += SatelliteInfo::SIZE)
                {
                    info.push_back(SatelliteInfo::unmarshal(begin, begin + SatelliteInfo::SIZE));
                }
            }
        } __attribute__((packed));

        struct GeodeticPosition
        {
            static constexpr uint8_t ID = 32;
            static constexpr int SIZE = 24;

            double lat_lon_z[3];

            template<typename InputIterator>
            static GeodeticPosition unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != GeodeticPosition::SIZE)
                    throw std::length_error("GeodeticPosition::unmarshal unexpected buffer size");

                GeodeticPosition out;
                for (int i = 0; i < 3; ++i)
                    out.lat_lon_z[i] = read64<double>(begin + 8 * i);
                return out;
            }
        } __attribute__((packed));

        struct NEDVelocity
        {
            static constexpr uint8_t ID = 35;
            static constexpr int SIZE = 12;

            float ned[3];

            template<typename InputIterator>
            static NEDVelocity unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("NEDVelocity::unmarshal buffer is not of the expected size");

                return NEDVelocity { {
                    read32<float>(begin),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8) } };
            }
        } __attribute__((packed));

        struct BodyVelocity
        {
            static constexpr uint8_t ID = 36;
            static constexpr int SIZE = 12;

            float xyz[3];

            template<typename InputIterator>
            static BodyVelocity unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("BodyVelocity::unmarshal buffer is not of the expected size");

                return BodyVelocity { {
                    read32<float>(begin),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8) } };
            }
        } __attribute__((packed));

        /** Acceleration with the G force removed */
        struct Acceleration
        {
            static constexpr uint8_t ID = 37;
            static constexpr int SIZE = 12;

            float xyz[3];

            template<typename InputIterator>
            static Acceleration unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("Acceleration::unmarshal buffer is not of the expected size");

                return Acceleration { {
                    read32<float>(begin),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8) } };
            }
        } __attribute__((packed));

        struct BodyAcceleration
        {
            static constexpr uint8_t ID = 38;
            static constexpr int SIZE = 16;

            float xyz[3];
            float g;

            template<typename InputIterator>
            static BodyAcceleration unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("BodyAcceleration::unmarshal buffer is not of the expected size");

                return BodyAcceleration {
                    {
                        read32<float>(begin),
                        read32<float>(begin + 4),
                        read32<float>(begin + 8)
                    },
                    read32<float>(begin + 12)
                };
            }
        } __attribute__((packed));

        struct QuaternionOrientation
        {
            static constexpr uint8_t ID = 40;
            static constexpr int SIZE = 16;

            float im;
            float xyz[3];

            template<typename InputIterator>
            static QuaternionOrientation unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("QuaternionOrientation::unmarshal buffer is not of the expected size");

                return QuaternionOrientation {
                    read32<float>(begin + 0),
                    {
                        read32<float>(begin + 4),
                        read32<float>(begin + 8),
                        read32<float>(begin + 12)
                    }
                };
            }
        } __attribute__((packed));

        struct AngularVelocity
        {
            static constexpr uint8_t ID = 42;
            static constexpr int SIZE = 12;

            float xyz[3];

            template<typename InputIterator>
            static AngularVelocity unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("AngularVelocity::unmarshal buffer is not of the expected size");

                return AngularVelocity { {
                    read32<float>(begin),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8) } };
            }
        } __attribute__((packed));

        struct AngularAcceleration
        {
            static constexpr uint8_t ID = 43;
            static constexpr int SIZE = 12;

            float xyz[3];

            template<typename InputIterator>
            static AngularAcceleration unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("AngularAcceleration::unmarshal buffer is not of the expected size");

                return AngularAcceleration { {
                    read32<float>(begin),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8) } };
            }
        } __attribute__((packed));

        struct LocalMagneticField
        {
            static constexpr uint8_t ID = 50;
            static constexpr int SIZE = 12;

            float xyz[3];

            template<typename InputIterator>
            static LocalMagneticField unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) != SIZE)
                    throw std::length_error("LocalMagneticField::unmarshal buffer is not of the expected size");

                return LocalMagneticField { {
                    read32<float>(begin),
                    read32<float>(begin + 4),
                    read32<float>(begin + 8) } };
            }
        } __attribute__((packed));

        struct NorthSeekingInitializationStatus
        {
            static constexpr uint8_t ID = 71;
            static constexpr int SIZE = 28;

            /** Status as one of NORTH_SEEKING_INITIALIZATION_FLAGS */
            uint16_t flags;
            uint16_t reserved = 0;
            uint8_t progress[4];
            float   current_rotation_angle;
            float   gyroscope_bias_solution_xyz[3];
            float   gyroscope_bias_solution_error;

            template<typename InputIterator>
            static NorthSeekingInitializationStatus unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("MagneticCalibrationStatus::unmarshal buffer size not the expected size");

                NorthSeekingInitializationStatus out;
                out.flags = read16<uint16_t>(begin);
                std::copy_n(begin + 4, 4, out.progress);
                out.current_rotation_angle = read32<float>(begin + 8);
                for (int i = 0; i < 3; ++i)
                    out.gyroscope_bias_solution_xyz[i] = read32<float>(begin + 12 + 4 * i);
                out.gyroscope_bias_solution_error = read32<float>(begin + 24);
                return out;
            }
        } __attribute__((packed));

        struct PacketTimerPeriod
        {
            static constexpr uint8_t ID = 180;
            static constexpr int SIZE = 4;

            uint8_t  permanent;
            uint8_t  utc_synchronization;
            uint16_t period;

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                out[0] = permanent;
                out[1] = utc_synchronization;
                write16(out + 2, period);
                return out + SIZE;
            }

            template<typename InputIterator>
            static PacketTimerPeriod unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("PacketTimerPeriod::unmarshal buffer size not the expected size");

                PacketTimerPeriod out;
                out.permanent = 0;
                out.utc_synchronization = *(begin + 1);
                out.period = read16<uint16_t>(begin + 2);
                return out;
            }
        } __attribute__((packed));

        struct PacketPeriods
        {
            static constexpr uint8_t ID = 181;
            static constexpr int MIN_SIZE = 2;
            static constexpr int PERIOD_SIZE = 5;

            typedef std::map<uint8_t, uint32_t> Periods;

            uint8_t permanent;
            uint8_t clear_existing;

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                out[0] = permanent;
                out[1] = clear_existing;
                return out + MIN_SIZE;
            }

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out, uint8_t packet_id, uint32_t period) const
            {
                out[0] = permanent;
                out[1] = clear_existing;
                std::pair<uint8_t, uint32_t> pair(packet_id, period);
                return marshal(out, &pair, &pair + 1);
            }

            template<typename OutputIterator, typename InputIterator>
            OutputIterator marshal(OutputIterator out, InputIterator begin, InputIterator end) const
            {
                out[0] = permanent;
                out[1] = clear_existing;
                for (out += 2; begin != end; ++begin, out += PERIOD_SIZE)
                {
                    out[0] = begin->first;
                    write32(out + 1, begin->second);
                }
                return out;
            }

            template<typename InputIterator>
            static std::map<uint8_t, uint32_t> unmarshal(InputIterator begin, InputIterator end)
            {
                if ((end - begin) < MIN_SIZE)
                    throw std::length_error("PacketPeriods::unmarshal buffer too small");
                else if ((end - begin - MIN_SIZE) % PERIOD_SIZE != 0)
                    throw std::length_error("PacketPeriods::unmarshal expected period list to be a multiple of 5");
                begin += MIN_SIZE;
                std::map<uint8_t, uint32_t> result;
                for (; begin != end; begin += PERIOD_SIZE)
                    result[*begin] = read32<uint32_t>(begin + 1);
                return result;
            }
        } __attribute__((packed));

        struct BaudRates
        {
            static constexpr uint8_t ID = 182;
            static constexpr int SIZE = 17;

            uint8_t permanent;
            uint32_t primary_port;
            uint32_t gpio;
            uint32_t auxiliary_rs232;
            uint32_t reserved;

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                out[0] = permanent;
                write32(out + 1, primary_port);
                write32(out + 5, gpio);
                write32(out + 9, auxiliary_rs232);
                write32(out + 13, static_cast<uint32_t>(0));
                return out + 17;
            }

            template<typename InputIterator>
            static BaudRates unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("BaudRates::unmarshal buffer size not the expected size");

                return BaudRates {
                    0,
                    read32<uint32_t>(begin + 1),
                    read32<uint32_t>(begin + 5),
                    read32<uint32_t>(begin + 9),
                    static_cast<uint32_t>(0)
                };
            }
        } __attribute__((packed));

        struct Alignment
        {
            static constexpr uint8_t ID = 185;
            static constexpr int SIZE = 73;

            uint8_t permanent;
            float dcm[9];
            float gnss_antenna_offset_xyz[3];
            float odometer_offset_xyz[3];
            float external_data_offset_xyz[3];

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                out[0] = permanent;
                for (int i = 0; i < 9; ++i)
                    write32(out + 1 + 4 * i, dcm[i]);
                for (int i = 0; i < 3; ++i)
                {
                    write32(out + 37 + 4 * i, gnss_antenna_offset_xyz[i]);
                    write32(out + 49 + 4 * i, odometer_offset_xyz[i]);
                    write32(out + 61 + 4 * i, external_data_offset_xyz[i]);
                }
                return out + SIZE;
            }

            template<typename InputIterator>
            static Alignment unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("Alignment::unmarshal buffer size not the expected size");

                Alignment out;
                out.permanent = 0;
                for (int i = 0; i < 9; ++i)
                    out.dcm[i] = read32<float>(begin + 1 + 4 * i);
                for (int i = 0; i < 3; ++i)
                {
                    out.gnss_antenna_offset_xyz[i]  = read32<float>(begin + 37 + 4 * i);
                    out.odometer_offset_xyz[i]      = read32<float>(begin + 49 + 4 * i);
                    out.external_data_offset_xyz[i] = read32<float>(begin + 61 + 4 * i);
                }
                return out;
            }
        } __attribute__((packed));

        struct FilterOptions
        {
            static constexpr uint8_t ID   = 186;
            static constexpr int SIZE = 17;

            uint8_t permanent;
            /** The vehicle type to tune the filter dynamics
             *
             * See VEHICLE_TYPES
             */
            uint8_t vehicle_type;
            uint8_t enabled_internal_gnss;
            uint8_t enabled_dual_antenna_heading;
            uint8_t enabled_atmospheric_altitude;
            uint8_t enabled_velocity_heading;
            uint8_t enabled_reversing_detection;
            uint8_t enabled_motion_analysis;
            uint8_t reserved_1[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                return std::copy(&permanent, reserved_1 + 9, out);
            }

            template<typename InputIterator>
            static FilterOptions unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("FilterOptions::unmarshal buffer size not the expected size");

                FilterOptions out;
                out.permanent = 0;
                std::copy(begin + 1, end, &out.vehicle_type);
                return out;
            }
        } __attribute__ ((packed));

        struct MagneticCalibrationValues
        {
            static constexpr uint8_t ID   = 189;
            static constexpr int SIZE = 49;

            uint8_t permanent;
            float hard_iron_bias_xyz[3];
            float soft_iron_transformation[9];

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                out[0] = permanent;
                for (int i = 0; i < 3; ++i)
                    write32(out + 1 + 4 * i, hard_iron_bias_xyz[i]);
                for (int i = 0; i < 9; ++i)
                    write32(out + 13 + 4 * i, soft_iron_transformation[i]);
                return out + SIZE;
            }

            template<typename InputIterator>
            static MagneticCalibrationValues unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("MagneticCalibrationValues::unmarshal buffer size not the expected size");

                MagneticCalibrationValues out;
                out.permanent = 0;
                for (int i = 0; i < 3; ++i)
                    out.hard_iron_bias_xyz[i] = read32<float>(begin + 1 + 4 * i);
                for (int i = 0; i < 9; ++i)
                    out.soft_iron_transformation[i] = read32<float>(begin + 13 + 4 * i);
                return out;
            }
        } __attribute__((packed));

        struct MagneticCalibrationConfiguration
        {
            static constexpr uint8_t ID = 190;
            static constexpr int SIZE = 1;

            /** Action as one of MAGNETIC_CALIBRATION_ACTIONS */
            uint8_t action;

            template<typename OutputIterator>
            OutputIterator marshal(OutputIterator out) const
            {
                out[0] = action;
                return out + SIZE;
            }
        } __attribute__((packed));

        enum MAGNETIC_CALIBRATION_ACTIONS
        {
            MAGNETIC_CALIBRATION_CANCEL,
            MAGNETIC_CALIBRATION_START_2D,
            MAGNETIC_CALIBRATION_START_3D,
            MAGNETIC_CALIBRATION_RESET
        };

        struct MagneticCalibrationStatus
        {
            static constexpr uint8_t ID = 191;
            static constexpr int SIZE = 3;

            /** Status as one of MAGNETIC_CALIBRATION_STATUS */
            uint8_t status;
            uint8_t progress;
            uint8_t error;

            template<typename InputIterator>
            static MagneticCalibrationStatus unmarshal(InputIterator begin, InputIterator end)
            {
                if (end - begin != SIZE)
                    throw std::length_error("MagneticCalibrationStatus::unmarshal buffer size not the expected size");

                MagneticCalibrationStatus out;
                std::copy(begin, end, &out.status);
                return out;
            }
        } __attribute__((packed));

        template<typename Packet, typename Driver>
        inline Header writePacket(Driver& driver, Packet const& packet)
        {
            static_assert(Packet::SIZE + Header::SIZE < 300, "packet and header size are bigger than the expected buffer size");
            uint8_t marshalled[300];
            uint8_t* marshalled_end = packet.marshal(marshalled + Header::SIZE);
            Header const* header =
                new(marshalled) Header(Packet::ID, marshalled + Header::SIZE, marshalled_end);
            driver.writePacket(marshalled, marshalled_end - marshalled);
            return *header;
        }

        template<typename Packet, typename Driver>
        inline Packet waitForPacket(Driver& driver, base::Time const& _timeout)
        {
            uint8_t marshalled[MAX_PACKET_SIZE * 10];
            driver.resetPollSynchronization();

            base::Timeout timeout(_timeout);
            do
            {
                base::Time left = timeout.timeLeft();
                if (left.toMicroseconds() < 0)
                    left = base::Time();
                int packet_size = driver.readPacket(marshalled, sizeof(marshalled), left);

                Header const& header = reinterpret_cast<Header const&>(*marshalled);
                if (header.packet_id == Packet::ID)
                    return Packet::unmarshal(marshalled + Header::SIZE, marshalled + packet_size);
            }
            while (!timeout.elapsed());
            throw iodrivers_base::TimeoutError(
                    iodrivers_base::TimeoutError::NONE,
                    "failed to get an expected response from the device");
        }

        template<typename Driver>
        inline ACK_RESULTS waitForAck(Driver& driver, Header const& header, base::Time const& _timeout)
        {
            base::Timeout timeout(_timeout);
            do
            {
                base::Time left = timeout.timeLeft();
                if (left.toMicroseconds() < 0)
                    left = base::Time();
                Acknowledge ack = waitForPacket<Acknowledge>(driver, left);

                if (ack.isMatching(header))
                    return static_cast<ACK_RESULTS>(ack.result);
            }
            while (!timeout.elapsed());
            throw iodrivers_base::TimeoutError(
                    iodrivers_base::TimeoutError::NONE,
                    "failed to get an ack matching the given packet header");
        }

        template<typename Driver>
        inline void validateAck(Driver& driver, Header const& header, base::Time const& _timeout)
        {
            ACK_RESULTS result = waitForAck(driver, header, _timeout);
            if (result != ACK_SUCCESS)
                throw AcknowledgeFailure(header.packet_id, result);
        }

        template<typename Packet, typename Driver>
        inline Packet query(Driver& driver)
        {
            uint8_t marshalled[MAX_PACKET_SIZE];
            uint8_t* marshalled_end = Request().marshal(marshalled + Header::SIZE, Packet::ID);
            new(marshalled) Header(Request::ID, marshalled + Header::SIZE, marshalled_end);
            driver.writePacket(marshalled, marshalled_end - marshalled);

            return waitForPacket<Packet>(driver, driver.getReadTimeout());
        }

        template<typename Driver>
        inline Header writePacketPeriod(Driver& driver, uint8_t packet_id, int period, bool clear_existing)
        {
            uint8_t marshalled[MAX_PACKET_SIZE];
            PacketPeriods packet;
            packet.permanent = 0;
            packet.clear_existing = clear_existing ? 1 : 0;
            uint8_t* marshalled_end = packet.marshal(marshalled + Header::SIZE, packet_id, period);
            Header const* header =
                new(marshalled) Header(PacketPeriods::ID, marshalled + Header::SIZE, marshalled_end);
            driver.writePacket(marshalled, marshalled_end - marshalled);
            return *header;
        }
    }
}

#endif
