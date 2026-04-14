/****************************************************************************
 *
 * Copyright (c) 2023, libmav development team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name libmav nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mav/BufferParser.h"
#include "mav/Message.h"
#include "mav/MessageSet.h"
#include "mav/MessageDefinition.h"
#include "mav/utils.h"
#include <cstring>
#include <array>

namespace mav {

    // MAVLink magic bytes
    static constexpr uint8_t MAVLINK_V1_MAGIC = 0xFE;
    static constexpr uint8_t MAVLINK_V2_MAGIC = 0xFD;

    // MAVLink v1 wire format constants
    // [magic(1), len(1), seq(1), sysid(1), compid(1), msgid(1), payload(N), crc(2)]
    static constexpr int V1_HEADER_SIZE = 6;
    static constexpr int V1_CHECKSUM_SIZE = 2;

    BufferParser::BufferParser(const MessageSet& message_set) noexcept :
        _message_set(message_set) {}

    std::optional<Message> BufferParser::parseMessage(
        const uint8_t* buffer,
        size_t buffer_size,
        size_t& bytes_consumed) const noexcept {

        bytes_consumed = 0;

        // Scan for the first MAVLink magic byte (v1 or v2)
        size_t start_pos = 0;
        while (start_pos < buffer_size &&
               buffer[start_pos] != MAVLINK_V2_MAGIC &&
               buffer[start_pos] != MAVLINK_V1_MAGIC) {
            start_pos++;
        }

        if (start_pos >= buffer_size) {
            bytes_consumed = buffer_size; // Consumed all bytes looking for magic
            return std::nullopt;
        }

        if (buffer[start_pos] == MAVLINK_V2_MAGIC) {
            return parseV2Message(buffer, buffer_size, start_pos, bytes_consumed);
        } else {
            return parseV1Message(buffer, buffer_size, start_pos, bytes_consumed);
        }
    }

    std::optional<Message> BufferParser::parseV2Message(
        const uint8_t* buffer,
        size_t buffer_size,
        size_t start_pos,
        size_t& bytes_consumed) const noexcept {

        // Check if we have enough bytes for complete v2 header
        if (start_pos + MessageDefinition::HEADER_SIZE > buffer_size) {
            bytes_consumed = start_pos; // Consumed bytes before magic, not including it
            return std::nullopt;
        }

        // Parse header
        std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> backing_memory{};
        std::memcpy(backing_memory.data(), buffer + start_pos, MessageDefinition::HEADER_SIZE);

        Header header{backing_memory.data()};
        const bool message_is_signed = header.isSigned();
        const int wire_length = MessageDefinition::HEADER_SIZE + header.len() +
                                MessageDefinition::CHECKSUM_SIZE +
                                (message_is_signed ? MessageDefinition::SIGNATURE_SIZE : 0);

        // Check if we have the complete message
        if (start_pos + static_cast<size_t>(wire_length) > buffer_size) {
            bytes_consumed = start_pos; // Consumed bytes up to magic byte
            return std::nullopt;
        }

        // Copy the complete message
        std::memcpy(backing_memory.data(), buffer + start_pos, static_cast<size_t>(wire_length));

        const int crc_offset = MessageDefinition::HEADER_SIZE + header.len();

        // Get message definition
        auto definition_opt = _message_set.getMessageDefinition(header.msgId());
        if (!definition_opt) {
            // Unknown message, skip it
            bytes_consumed = start_pos + static_cast<size_t>(wire_length);
            return std::nullopt;
        }
        auto& definition = definition_opt.get();

        // Validate CRC
        CRC crc;
        crc.accumulate(backing_memory.begin() + 1, backing_memory.begin() + crc_offset);
        crc.accumulate(definition.crcExtra());
        auto crc_received = deserialize<uint16_t>(backing_memory.data() + crc_offset, sizeof(uint16_t));

        if (crc_received != crc.crc16()) {
            // CRC error, skip this message
            bytes_consumed = start_pos + static_cast<size_t>(wire_length);
            return std::nullopt;
        }

        // Create Message object
        ConnectionPartner partner{0, 0, false};
        bytes_consumed = start_pos + static_cast<size_t>(wire_length);

        return Message::_instantiateFromMemory(definition, partner, crc_offset, std::move(backing_memory));
    }

    std::optional<Message> BufferParser::parseV1Message(
        const uint8_t* buffer,
        size_t buffer_size,
        size_t start_pos,
        size_t& bytes_consumed) const noexcept {

        // MAVLink v1 header: [magic(1), len(1), seq(1), sysid(1), compid(1), msgid(1)]
        // Check if we have enough bytes for the complete v1 header
        if (start_pos + static_cast<size_t>(V1_HEADER_SIZE) > buffer_size) {
            bytes_consumed = start_pos; // Consumed bytes before magic, not including it
            return std::nullopt;
        }

        const uint8_t payload_len = buffer[start_pos + 1];
        const int wire_length = V1_HEADER_SIZE + payload_len + V1_CHECKSUM_SIZE;

        // Check if we have the complete message
        if (start_pos + static_cast<size_t>(wire_length) > buffer_size) {
            bytes_consumed = start_pos; // Wait for more data
            return std::nullopt;
        }

        const uint8_t seq    = buffer[start_pos + 2];
        const uint8_t sysid  = buffer[start_pos + 3];
        const uint8_t compid = buffer[start_pos + 4];
        const uint8_t msgid  = buffer[start_pos + 5]; // v1: 8-bit message ID

        // Get message definition
        auto definition_opt = _message_set.getMessageDefinition(static_cast<int>(msgid));
        if (!definition_opt) {
            // Unknown message, skip it
            bytes_consumed = start_pos + static_cast<size_t>(wire_length);
            return std::nullopt;
        }
        auto& definition = definition_opt.get();

        // Validate CRC.
        // v1 CRC covers bytes [1..V1_HEADER_SIZE-1+payload_len] relative to start_pos,
        // i.e., everything after the magic byte up through the last payload byte.
        CRC crc;
        crc.accumulate(
            buffer + start_pos + 1,
            buffer + start_pos + V1_HEADER_SIZE + payload_len);
        crc.accumulate(definition.crcExtra());

        const size_t crc_pos = start_pos + static_cast<size_t>(V1_HEADER_SIZE) + payload_len;
        const uint16_t crc_received = static_cast<uint16_t>(buffer[crc_pos]) |
                                      (static_cast<uint16_t>(buffer[crc_pos + 1]) << 8);

        if (crc_received != crc.crc16()) {
            // CRC error, skip this message
            bytes_consumed = start_pos + static_cast<size_t>(wire_length);
            return std::nullopt;
        }

        // Convert to v2 wire format in backing_memory so Message::_instantiateFromMemory
        // can be reused. v2 header is 10 bytes; payload starts at offset HEADER_SIZE.
        std::array<uint8_t, MessageDefinition::MAX_MESSAGE_SIZE> backing_memory{};
        backing_memory[0] = MAVLINK_V2_MAGIC; // magic (converted)
        backing_memory[1] = payload_len;
        backing_memory[2] = 0;      // incompat flags (none in v1)
        backing_memory[3] = 0;      // compat flags  (none in v1)
        backing_memory[4] = seq;
        backing_memory[5] = sysid;
        backing_memory[6] = compid;
        backing_memory[7] = msgid;  // low byte of 24-bit msgid
        backing_memory[8] = 0;      // mid byte
        backing_memory[9] = 0;      // high byte
        // Copy payload starting at v2 header offset
        std::memcpy(
            backing_memory.data() + MessageDefinition::HEADER_SIZE,
            buffer + start_pos + V1_HEADER_SIZE,
            payload_len);

        const int crc_offset = MessageDefinition::HEADER_SIZE + payload_len;
        bytes_consumed = start_pos + static_cast<size_t>(wire_length);

        ConnectionPartner partner{0, 0, false};
        return Message::_instantiateFromMemory(definition, partner, crc_offset, std::move(backing_memory));
    }

} // namespace mav
