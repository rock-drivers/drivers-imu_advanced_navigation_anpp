#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include <advanced_navigation_anpp/Protocol.hpp>

using namespace std;
using namespace advanced_navigation_anpp;

TEST(protocol_Header, it_is_invalid_when_constructed)
{
    protocol::Header header;
    ASSERT_FALSE(header.isValid());
    uint8_t packet[0];
    ASSERT_FALSE(header.isPacketValid(packet, packet));
}

static const int PAYLOAD_SIZE = 7;
static const uint8_t payload[7] = { '0', '1', '2', '3', '4', '5', '6' };

TEST(protocol_Header, it_initializes_a_valid_packet_from_data)
{
    protocol::Header header(5, payload, payload + PAYLOAD_SIZE);
    ASSERT_TRUE(header.isValid());
    ASSERT_TRUE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
    ASSERT_EQ(5, header.packet_id);
    ASSERT_EQ(PAYLOAD_SIZE, header.payload_length);
}

TEST(protocol_Header, isValid_returns_true_if_the_header_checksum_matches)
{
    protocol::Header header;
    header.header_checksum = 0;
    header.packet_id = 0;
    header.payload_length = 0;
    header.payload_checksum_lsb = 0; // calculated online
    header.payload_checksum_msb = 0;
    ASSERT_TRUE(header.isValid());
}

TEST(protocol_Header, isPacketValid_returns_true_if_the_packet_size_and_checksum_match)
{
    protocol::Header header;
    header.payload_length = PAYLOAD_SIZE;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_TRUE(header.isPacketValid(payload, payload + 7));
}

TEST(protocol_Header, isPacketValid_returns_false_if_the_checksum_matches_but_the_header_size_is_higher)
{
    protocol::Header header;
    header.payload_length = PAYLOAD_SIZE + 1;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_FALSE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
}

TEST(protocol_Header, isPacketValid_returns_false_if_the_checksum_matches_but_the_header_size_is_smaller)
{
    protocol::Header header;
    header.payload_length = PAYLOAD_SIZE - 1;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_FALSE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
}

TEST(protocol_Header, isPacketValid_returns_false_if_the_length_matches_but_not_the_checksum)
{
    protocol::Header header;
    uint8_t payload[7] = { '0', '1', '2', '3', '4', '5', '7' };
    header.payload_length = PAYLOAD_SIZE;
    header.payload_checksum_lsb = 0xA7; // calculated online
    header.payload_checksum_msb = 0x88;
    ASSERT_FALSE(header.isPacketValid(payload, payload + PAYLOAD_SIZE));
}
