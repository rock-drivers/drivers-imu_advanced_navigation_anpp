#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <advanced_navigation_anpp/Protocol.hpp>
#include <list>
#include <stdexcept>
#include <cstring>

using namespace std;
using namespace advanced_navigation_anpp;
using namespace advanced_navigation_anpp::protocol;
using ::testing::ElementsAre;

TEST(protocol_Header, it_is_invalid_when_constructed)
{
    Header header;
    ASSERT_FALSE(header.isValid());
    uint8_t packet[0];
    ASSERT_FALSE(header.isPacketValid(packet, packet));
}

static const int PAYLOAD_SIZE = 7;
static const uint8_t payload[7] = { '0', '1', '2', '3', '4', '5', '6' };

TEST(protocol_Header, it_initializes_a_valid_packet_from_data)
{
    Header header(5, payload, payload + PAYLOAD_SIZE);
    ASSERT_TRUE(header.isValid());
    ASSERT_TRUE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
    ASSERT_EQ(5, header.packet_id);
    ASSERT_EQ(PAYLOAD_SIZE, header.payload_length);
}

TEST(protocol_Header, isValid_returns_true_if_the_header_checksum_matches)
{
    Header header;
    header.header_checksum = 0;
    header.packet_id = 0;
    header.payload_length = 0;
    header.payload_checksum_lsb = 0; // calculated online
    header.payload_checksum_msb = 0;
    ASSERT_TRUE(header.isValid());
}

TEST(protocol_Header, isPacketValid_returns_true_if_the_packet_size_and_checksum_match)
{
    Header header;
    header.payload_length = PAYLOAD_SIZE;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_TRUE(header.isPacketValid(payload, payload + 7));
}

TEST(protocol_Header, isPacketValid_returns_false_if_the_checksum_matches_but_the_header_size_is_higher)
{
    Header header;
    header.payload_length = PAYLOAD_SIZE + 1;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_FALSE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
}

TEST(protocol_Header, isPacketValid_returns_false_if_the_checksum_matches_but_the_header_size_is_smaller)
{
    Header header;
    header.payload_length = PAYLOAD_SIZE - 1;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_FALSE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
}

TEST(protocol_Header, isPacketValid_returns_false_if_the_length_matches_but_not_the_checksum)
{
    Header header;
    uint8_t payload[7] = { '0', '1', '2', '3', '4', '5', '7' };
    header.payload_length = PAYLOAD_SIZE;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_FALSE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
}

TEST(protocol_Acknowledge, isMatching_returns_true_if_the_id_and_checksum_match)
{
    Header header(1, nullptr, nullptr);
    Acknowledge ack = { 1, header.payload_checksum_lsb, header.payload_checksum_msb, ACK_SUCCESS };
    ASSERT_TRUE(ack.isMatching(header));
}

TEST(protocol_Acknowledge, isMatching_returns_false_if_the_id_does_not_match)
{
    Header header(1, nullptr, nullptr);
    Acknowledge ack = { 2, header.payload_checksum_lsb, header.payload_checksum_msb, ACK_SUCCESS };
    ASSERT_FALSE(ack.isMatching(header));
}

TEST(protocol_Acknowledge, isMatching_returns_false_if_the_checksum_lsb_does_not_match)
{
    Header header(1, nullptr, nullptr);
    Acknowledge ack = { 1, 0x10, header.payload_checksum_msb, ACK_SUCCESS };
    ASSERT_FALSE(ack.isMatching(header));
}

TEST(protocol_Acknowledge, isMatching_returns_false_if_the_checksum_msb_does_not_match)
{
    Header header(1, nullptr, nullptr);
    Acknowledge ack = { 1, header.payload_checksum_lsb, 0x10, ACK_SUCCESS };
    ASSERT_FALSE(ack.isMatching(header));
}


template<typename Predicate>
void validateAcknowledgePredicate(std::list<ACK_RESULTS> expected_true, Predicate predicate)
{
    for (uint8_t i = 0; i < 8; ++i)
    {
        Acknowledge ack = { 1, 0, 0, i };
        if (expected_true.front() == i)
        {
            ASSERT_TRUE(predicate(ack));
            expected_true.pop_front();
        }
        else
        {
            ASSERT_FALSE(predicate(ack));
        }

    }
}

TEST(protocol_Acknowledge, isSuccess)
{
    validateAcknowledgePredicate({ ACK_SUCCESS },
            [](Acknowledge ack){ return ack.isSuccess(); });
}

TEST(protocol_Acknowledge, isPacketValidationFailure)
{
    validateAcknowledgePredicate({ ACK_FAILED_PACKET_VALIDATION_CRC, ACK_FAILED_PACKET_VALIDATION_SIZE },
            [](Acknowledge ack){ return ack.isPacketValidationFailure(); });
}

TEST(protocol_Acknowledge, isProtocolError)
{
    validateAcknowledgePredicate({ ACK_FAILED_OUT_OF_RANGE, ACK_FAILED_UNKNOWN_PACKET },
            [](Acknowledge ack){ return ack.isProtocolError(); });
}

TEST(protocol_Acknowledge, isNotReady)
{
    validateAcknowledgePredicate({ ACK_FAILED_SYSTEM_NOT_READY },
            [](Acknowledge ack){ return ack.isNotReady(); });
}

TEST(protocol_Acknowledge, isSystemError)
{
    validateAcknowledgePredicate({ ACK_FAILED_SYSTEM_FLASH_FAILURE },
            [](Acknowledge ack){ return ack.isSystemError(); });
}

TEST(protocol_Acknowledge, unmarshal)
{
    uint8_t data[5] = { 1, 2, 3, ACK_SUCCESS };
    Acknowledge ack = Acknowledge::unmarshal(data, data + 4);
    ASSERT_EQ(1, ack.acked_packet_id);
    ASSERT_EQ(2, ack.acked_payload_checksum_lsb);
    ASSERT_EQ(3, ack.acked_payload_checksum_msb);
    ASSERT_EQ(ACK_SUCCESS, ack.result);
}

TEST(protocol_Acknowledge, unmarshal_fails_if_too_little_data_is_provided_and_does_not_access_any_of_it)
{
    uint8_t* ptr = nullptr;
    ASSERT_THROW(Acknowledge::unmarshal(ptr, ptr + 3), std::length_error);
}

TEST(protocol_Acknowledge, unmarshal_fails_if_too_much_data_is_provided_and_does_not_access_any_of_it)
{
    uint8_t* ptr = nullptr;
    ASSERT_THROW(Acknowledge::unmarshal(ptr, ptr + 5), std::length_error);
}

TEST(protocol_Request, marshal_copies_the_packet_ids_to_the_output)
{
    vector<uint8_t> out{ 0, 0, 0 };
    PACKET_IDS const packet_ids[3] = { ID_PACKET_TIMER_PERIOD, ID_VELOCITY_BODY, ID_VELOCITY_NED };
    auto out_end = Request().marshal(out.begin(), packet_ids, packet_ids + 3);
    ASSERT_EQ(out_end, out.end());
    ASSERT_THAT(out, ElementsAre(ID_PACKET_TIMER_PERIOD, ID_VELOCITY_BODY, ID_VELOCITY_NED));
}

TEST(protocol_BootMode, marshal)
{
    vector<uint8_t> out{ 0xFF };
    auto out_end = BootMode{ BOOT_TO_BOOTLOADER }.marshal(out.begin());
    ASSERT_EQ(out_end, out.end());
    ASSERT_THAT(out, ElementsAre(BOOT_TO_BOOTLOADER));
}

TEST(protocol_BootMode, unmarshal)
{
    vector<uint8_t> out{ BOOT_TO_PROGRAM };
    BootMode mode = BootMode::unmarshal(out.begin(), out.end());
    ASSERT_EQ(BOOT_TO_PROGRAM, mode.boot_mode);
}

TEST(protocol_BootMode, unmarshal_fails_if_too_little_data_is_provided_and_does_not_access_any_of_it)
{
    uint8_t* ptr = nullptr;
    ASSERT_THROW(BootMode::unmarshal(ptr, ptr), std::length_error);
}

TEST(protocol_BootMode, unmarshal_fails_if_too_much_data_is_provided_and_does_not_access_any_of_it)
{
    uint8_t* ptr = nullptr;
    ASSERT_THROW(BootMode::unmarshal(ptr, ptr + 2), std::length_error);
}

TEST(protocol_DeviceInformation, unmarshal)
{
    uint8_t device_info[24];
    std::iota(device_info, device_info + 24, 0);
    DeviceInformation info = DeviceInformation::unmarshal(device_info, device_info + 24);
    ASSERT_EQ(0x03020100, info.software_version);
    ASSERT_EQ(0x07060504, info.device_id);
    ASSERT_EQ(0x0b0a0908, info.hardware_revision);
    ASSERT_EQ(0x0f0e0d0c, info.serial_number_part0);
    ASSERT_EQ(0x13121110, info.serial_number_part1);
    ASSERT_EQ(0x17161514, info.serial_number_part2);
}

TEST(protocol_DeviceInformation, unmarshal_fails_if_too_little_data_is_provided_and_does_not_access_any_of_it)
{
    uint8_t* ptr = nullptr;
    ASSERT_THROW(DeviceInformation::unmarshal(ptr, ptr + 23), std::length_error);
}

TEST(protocol_DeviceInformation, unmarshal_fails_if_too_much_data_is_provided_and_does_not_access_any_of_it)
{
    uint8_t* ptr = nullptr;
    ASSERT_THROW(DeviceInformation::unmarshal(ptr, ptr + 25), std::length_error);
}

void RAW_SET(uint8_t* begin, std::vector<uint8_t> bytes)
{
    std::copy(bytes.begin(), bytes.end(), begin);
}

TEST(protocol_SystemState, unmarshal)
{
    uint8_t marshalled[100];
    SystemState expected;
    expected.system_status = 0x0201;
    RAW_SET(marshalled,      { 0x01, 0x02 } );
    expected.filter_status = 0x0403;
    RAW_SET(marshalled + 2,  { 0x03, 0x04 } );
    expected.unix_time_seconds = 0x08070605;
    RAW_SET(marshalled + 4,  { 0x05, 0x06, 0x07, 0x08 } );
    expected.unix_time_microseconds = 0x0c0b0a09;
    RAW_SET(marshalled + 8,  { 0x09, 0x0a, 0x0b, 0x0c } );

    // Floating-point values below have been generated by taking a  seed value
    // (the first one, starting with the 123456... sequence) and then adding one
    // to the first and last byte, checking the corresponding FP value
    //
    // Found FP-to-binary converters online
    expected.lat_lon_z[0] = 1.2345678910111214e16;
    RAW_SET(marshalled + 12, { 0xF7, 0x5E, 0xF9, 0x2E, 0x2A, 0xEE, 0x45, 0x43 } );
    expected.lat_lon_z[1] = 8.09086413053048651776E20;
    RAW_SET(marshalled + 20, { 0xF8, 0x5E, 0xF9, 0x2E, 0x2A, 0xEE, 0x45, 0x44 } );
    expected.lat_lon_z[2] = 5.3024287165844605032726528E25;
    RAW_SET(marshalled + 28, { 0xF9, 0x5E, 0xF9, 0x2E, 0x2A, 0xEE, 0x45, 0x45 } );
    expected.velocity_ned[0] = 1.23456792E8;
    RAW_SET(marshalled + 36, { 0xA3, 0x79, 0xEB, 0x4C } );
    expected.velocity_ned[1] = 4.938272E8;
    RAW_SET(marshalled + 40, { 0xA4, 0x79, 0xEB, 0x4D } );
    expected.velocity_ned[2] = 1.975308928E9;
    RAW_SET(marshalled + 44, { 0xA5, 0x79, 0xEB, 0x4E } );
    expected.body_acceleration_xyz[0] = 7.901236224E9;
    RAW_SET(marshalled + 48, { 0xA6, 0x79, 0xEB, 0x4F } );
    expected.body_acceleration_xyz[1] = 3.1604946944E10;
    RAW_SET(marshalled + 52, { 0xA7, 0x79, 0xEB, 0x50 } );
    expected.body_acceleration_xyz[2] = 1.26419795968E11;
    RAW_SET(marshalled + 56, { 0xA8, 0x79, 0xEB, 0x51 } );
    expected.g = 5.0567921664E11;
    RAW_SET(marshalled + 60, { 0xA9, 0x79, 0xEB, 0x52 } );
    expected.rpy[0] = 2.022716997632E12;
    RAW_SET(marshalled + 64, { 0xAA, 0x79, 0xEB, 0x53 } );
    expected.rpy[1] = 8.090868514816E12;
    RAW_SET(marshalled + 68, { 0xAB, 0x79, 0xEB, 0x54 } );
    expected.rpy[2] = 3.2363476156416E13;
    RAW_SET(marshalled + 72, { 0xAC, 0x79, 0xEB, 0x55 } );
    expected.angular_velocity[0] = 1.29453913014272E14;
    RAW_SET(marshalled + 76, { 0xAD, 0x79, 0xEB, 0x56 } );
    expected.angular_velocity[1] = 5.1781568561152E14;
    RAW_SET(marshalled + 80, { 0xAE, 0x79, 0xEB, 0x57 } );
    expected.angular_velocity[2] = 2.071262876663808E15;
    RAW_SET(marshalled + 84, { 0xAF, 0x79, 0xEB, 0x58 } );
    expected.lat_lon_z_stddev[0] = 8.285052043526144E15;
    RAW_SET(marshalled + 88, { 0xB0, 0x79, 0xEB, 0x59 } );
    expected.lat_lon_z_stddev[1] = 1.35742301477225365504E20;
    RAW_SET(marshalled + 92, { 0xB1, 0x79, 0xEB, 0x60 } );
    expected.lat_lon_z_stddev[2] = 5.42969241093273550848E20;
    RAW_SET(marshalled + 96, { 0xB2, 0x79, 0xEB, 0x61 } );

    SystemState unmarshalled = SystemState::unmarshal(marshalled, marshalled + 50);
    ASSERT_FALSE(std::memcmp(&expected, &unmarshalled, sizeof(SystemState)));
}

