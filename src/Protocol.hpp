#ifndef ADVANCED_NAVIGATION_ANPP_HEADER_HPP
#define ADVANCED_NAVIGATION_ANPP_HEADER_HPP

#include <cstdint>

namespace advanced_navigation_anpp
{
    namespace protocol
    {
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
    }
}

#endif
